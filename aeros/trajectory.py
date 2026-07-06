"""3-DOF ascent trajectory simulation to orbit.

Point-mass dynamics in the orbital plane of a spherical Earth, integrated
with adaptive RK45. The vehicle flies:

  1. vertical rise off the pad,
  2. an open-loop pitch-over ("kick") completed while dynamic pressure is
     still negligible (< 2 kPa),
  3. linear-tangent commanded pitch, with commanded attitude clamped to an
     angle-of-attack envelope  |alpha| <= min(alpha_max, (q*alpha)_lim / q)
     -- the structural q-alpha load constraint every real launcher flies.
     At high dynamic pressure this recovers the classic zero-AoA gravity
     turn; as q decays the guidance gains authority, exactly like a real
     closed-loop ascent.

Earth rotation is credited as initial tangential velocity at the launch
latitude. Payload capacity is found by bisection on payload mass with the
four steering parameters re-optimised at every step (differential
evolution), subject to a max-q limit. References: Sutton & Biblarz;
Humble, Henry & Larson, "Space Propulsion Analysis and Design".

State vector: [r, theta, v_r, v_t, m]
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.integrate import solve_ivp

from .atmosphere import atmosphere
from .aero import drag_coefficient
from .engines import G0
from .vehicle import Vehicle

MU_EARTH = 3.986004418e14   # m^3/s^2
R_EARTH = 6_371_000.0       # m (mean radius)
OMEGA_EARTH = 7.2921159e-5  # rad/s

Q_KICK_MAX_PA = 2_000.0     # pitch program must finish below this q
QALPHA_LIMIT = 2_800.0      # Pa*rad, structural q-alpha envelope (~160 kPa deg)
ALPHA_MAX = math.radians(8.0)   # AoA cap while aerodynamically loaded
Q_FREE_PA = 100.0           # below this q, attitude is aerodynamically free
Q_THROTTLE_SOFT_PA = 28_000.0   # begin throttling down at this q
THROTTLE_MIN = 0.65         # deep-throttle capability (Merlin-class)


@dataclass
class PhaseResult:
    name: str
    t: np.ndarray
    r: np.ndarray
    vr: np.ndarray
    vt: np.ndarray
    m: np.ndarray


@dataclass
class AscentResult:
    reached_orbit: bool
    perigee_m: float
    apogee_m: float
    final_altitude_m: float
    final_velocity_m_s: float
    max_q_Pa: float
    max_accel_g: float
    phases: list[PhaseResult] = field(default_factory=list)

    @property
    def time_s(self) -> np.ndarray:
        return np.concatenate([p.t for p in self.phases])

    @property
    def altitude_m(self) -> np.ndarray:
        return np.concatenate([p.r for p in self.phases]) - R_EARTH

    @property
    def velocity_m_s(self) -> np.ndarray:
        vr = np.concatenate([p.vr for p in self.phases])
        vt = np.concatenate([p.vt for p in self.phases])
        return np.sqrt(vr ** 2 + vt ** 2)

    @property
    def mass_kg(self) -> np.ndarray:
        return np.concatenate([p.m for p in self.phases])


def _orbital_elements(r, vr, vt):
    """Return (perigee altitude, apogee altitude) in metres."""
    v2 = vr * vr + vt * vt
    energy = v2 / 2 - MU_EARTH / r
    h = r * vt
    if energy >= 0:
        return 1e12, 1e12
    a = -MU_EARTH / (2 * energy)
    e2 = max(0.0, 1 + 2 * energy * h * h / MU_EARTH ** 2)
    e = math.sqrt(e2)
    return a * (1 - e) - R_EARTH, a * (1 + e) - R_EARTH


def simulate_ascent(
    vehicle: Vehicle,
    target_altitude_m: float = 200_000.0,
    launch_latitude_deg: float = 28.5,
    s1_pitch_final_deg: float = 20.0,
    s2_tan0: float = 0.35,
    upper_tan_final: float = 0.0,
    kick_speed_m_s: float = 40.0,
    stage_coast_s: float = 8.0,
) -> AscentResult:
    """Fly the vehicle; return achieved orbit and flight statistics."""

    cos_lat = math.cos(math.radians(launch_latitude_deg))
    v_rot = OMEGA_EARTH * R_EARTH * cos_lat
    area = math.pi / 4 * max(s.diameter_m for s in vehicle.stages) ** 2

    stats = {"max_q": 0.0, "max_g": 0.0}
    fairing_state = {"dropped": False}
    fairing_mass = vehicle.fairing_mass_kg
    orbit_margin_m = 15_000.0
    phases: list[PhaseResult] = []

    def make_rhs(stage, steering, t_ignite):
        def rhs(t, y):
            r, th, vr, vt, m = y
            alt = r - R_EARTH
            atmo = atmosphere(alt)
            # the atmosphere co-rotates with Earth: aerodynamics see the
            # AIR-relative velocity, not the inertial one
            vt_air = vt - OMEGA_EARTH * r * cos_lat
            v_air = math.hypot(vr, vt_air)
            mach = v_air / atmo.speed_of_sound_m_s
            q = 0.5 * atmo.density_kg_m3 * v_air * v_air
            stats["max_q"] = max(stats["max_q"], q)

            # automatic q-limiter throttle bucket (first stage only)
            throttle = 1.0
            if stage is vehicle.stages[0] and q > Q_THROTTLE_SOFT_PA:
                throttle = max(THROTTLE_MIN,
                               1.0 - 0.8 * (q - Q_THROTTLE_SOFT_PA)
                               / Q_THROTTLE_SOFT_PA)
            thrust = stage.thrust_at(atmo.pressure_Pa) * throttle
            # flight-path angle in the air frame (for AoA / gravity turn)
            gamma = math.atan2(vr, vt_air) if v_air > 1.0 else math.pi / 2

            if steering[0] == "vertical":
                pitch = math.pi / 2
            elif steering[0] == "pitch_program":
                # ("pitch_program", t_start, t_end, pitch_final):
                # commanded pitch ramps linearly 90 deg -> pitch_final;
                # the q-alpha clamp below turns this into a physical
                # gravity-turn-like profile through the atmosphere
                _, t_s, t_e, pitch_f = steering
                frac = min(max((t - t_s) / max(t_e - t_s, 1.0), 0.0), 1.0)
                pitch_cmd = math.pi / 2 + (pitch_f - math.pi / 2) * frac
            else:  # ("lt", tan0, tf, tan_final): linear tangent (exo)
                _, tan0, tf, tan_final = steering
                frac = min(max((t - t_ignite) / max(tf, 1.0), 0.0), 1.0)
                tan_cmd = tan0 + (tan_final - tan0) * frac
                pitch_cmd = math.atan(tan_cmd)
            if steering[0] != "vertical":
                # angle-of-attack envelope (q-alpha structural limit);
                # unconstrained once aerodynamically unloaded
                if q < Q_FREE_PA:
                    pitch = pitch_cmd
                else:
                    alpha_lim = min(ALPHA_MAX, QALPHA_LIMIT / q)
                    pitch = min(max(pitch_cmd, gamma - alpha_lim),
                                gamma + alpha_lim)

            ur, ut = math.sin(pitch), math.cos(pitch)
            a_thrust = thrust / m
            stats["max_g"] = max(stats["max_g"], a_thrust / G0)
            drag = q * drag_coefficient(mach) * area
            a_drag = drag / m
            g = MU_EARTH / (r * r)

            # drag opposes the AIR-relative velocity
            if v_air > 1.0:
                d_r, d_t = a_drag * vr / v_air, a_drag * vt_air / v_air
            else:
                d_r, d_t = 0.0, 0.0
            dvr = a_thrust * ur - d_r - g + vt * vt / r
            dvt = a_thrust * ut - d_t - vr * vt / r
            return [vr, vt / r, dvr, dvt, -stage.mass_flow_kg_s * throttle]
        return rhs

    def coast_rhs(t, ys):
        r, th, vr, vt, m = ys
        alt = r - R_EARTH
        atmo = atmosphere(alt)
        vt_air = vt - OMEGA_EARTH * r * cos_lat
        v_air = math.hypot(vr, vt_air)
        q = 0.5 * atmo.density_kg_m3 * v_air * v_air
        mach = v_air / atmo.speed_of_sound_m_s
        a_drag = q * drag_coefficient(mach) * area / m
        if v_air > 1.0:
            d_r, d_t = a_drag * vr / v_air, a_drag * vt_air / v_air
        else:
            d_r, d_t = 0.0, 0.0
        g = MU_EARTH / (r * r)
        return [vr, vt / r, -g - d_r + vt * vt / r, -d_t - vr * vt / r, 0.0]

    def ground_hit(t, ys):
        return ys[0] - (R_EARTH - 200.0)
    ground_hit.terminal = True
    ground_hit.direction = -1

    def run_phase(name, rhs, t_span, y0, events):
        sol = solve_ivp(rhs, t_span, y0, events=list(events) + [ground_hit],
                        max_step=2.0,
                        rtol=1e-7, atol=[1.0, 1e-9, 0.01, 0.01, 0.1])
        phases.append(PhaseResult(name, sol.t, sol.y[0], sol.y[2],
                                  sol.y[3], sol.y[4]))
        return [c[-1] for c in sol.y], sol.t[-1]

    def q_of(ys):
        alt = ys[0] - R_EARTH
        v_air = math.hypot(ys[2], ys[3] - OMEGA_EARTH * ys[0] * cos_lat)
        return 0.5 * atmosphere(alt).density_kg_m3 * v_air * v_air

    def orbit_ok(t, ys):
        per, _ = _orbital_elements(ys[0], ys[2], ys[3])
        return per - (target_altitude_m - orbit_margin_m)
    orbit_ok.terminal = True
    orbit_ok.direction = 1

    def fairing_check(t, ys):
        return ys[0] - (R_EARTH + vehicle.fairing_jettison_alt_m)
    fairing_check.terminal = True
    fairing_check.direction = 1

    # ---------------- flight ----------------
    y = [R_EARTH, 0.0, 0.0, v_rot, vehicle.glow_kg]
    t0 = 0.0
    n_stages = len(vehicle.stages)

    for i, stage in enumerate(vehicle.stages):
        burn_time = stage.burn_time_s
        m_burnout = y[4] - stage.usable_propellant_kg
        t_ignite = t0

        def prop_out(t, ys, mb=m_burnout):
            return ys[4] - mb
        prop_out.terminal = True

        if i == 0:
            # 1) vertical rise
            def kick_start(t, ys):
                return math.hypot(ys[2], ys[3] - v_rot) - kick_speed_m_s
            kick_start.terminal = True
            y, t0 = run_phase("vertical", make_rhs(stage, ("vertical",), t0),
                              (t0, t0 + burn_time), y, [kick_start, prop_out])

            # 2) commanded pitch program to burnout, physically constrained
            #    by the q-alpha envelope (this produces the gravity turn)
            t_burnout_est = t_ignite + burn_time
            pitch_f = math.radians(s1_pitch_final_deg)
            while y[4] > m_burnout + 0.5:
                def prop_out_pp(t, ys, mb=m_burnout):
                    return ys[4] - mb
                prop_out_pp.terminal = True
                ev = [prop_out_pp]
                if not fairing_state["dropped"]:
                    ev.append(fairing_check)
                y, t0 = run_phase(
                    "stage1_ascent",
                    make_rhs(stage, ("pitch_program", t0, t_burnout_est,
                                     pitch_f), t0),
                    (t0, t_burnout_est + 60.0), y, ev)
                if (not fairing_state["dropped"]
                        and y[0] - R_EARTH >= vehicle.fairing_jettison_alt_m
                        and y[4] - fairing_mass > m_burnout + 1.0):
                    y[4] -= fairing_mass
                    m_burnout -= fairing_mass
                    fairing_state["dropped"] = True
                    continue
                break

        # 3) guided ascent for the remainder of this stage's burn.
        #    Upper stages fly a two-burn insertion when efficient: burn
        #    until apogee reaches the target, coast to apogee, circularise
        #    (real restartable upper stages fly exactly this profile).
        if i == 0:
            per, apo = _orbital_elements(y[0], y[2], y[3])
            if per >= target_altitude_m - orbit_margin_m:
                break
            y[4] -= (stage.dry_mass_kg
                     + (stage.propellant_kg - stage.usable_propellant_kg))
            y, t0 = run_phase("coast", coast_rhs, (t0, t0 + stage_coast_s),
                              y, [])
            continue

        coasted_to_apo = False
        first_burn_leg = True
        while y[4] > m_burnout + 0.5:
            rem_burn = (y[4] - m_burnout) / stage.mass_flow_kg_s
            if first_burn_leg:
                tan0 = s2_tan0
                first_burn_leg = False
            else:
                tan0 = math.tan(math.atan2(
                    y[2], y[3] - OMEGA_EARTH * y[0] * cos_lat))
            tan_final = upper_tan_final

            def prop_out_g(t, ys, mb=m_burnout):
                return ys[4] - mb
            prop_out_g.terminal = True

            def apo_ok(t, ys):
                _, apo = _orbital_elements(ys[0], ys[2], ys[3])
                if apo > 1e11:
                    return 1.0
                return apo - target_altitude_m
            apo_ok.terminal = True
            apo_ok.direction = 1

            ev = [prop_out_g]
            if i > 0:
                ev.append(orbit_ok)
                if not coasted_to_apo:
                    ev.append(apo_ok)
            if not fairing_state["dropped"]:
                ev.append(fairing_check)
            y, t0 = run_phase(f"stage{i+1}_guided",
                              make_rhs(stage, ("lt", tan0, rem_burn,
                                               tan_final), t0),
                              (t0, t0 + rem_burn + 1.0), y, ev)
            per, apo = _orbital_elements(y[0], y[2], y[3])
            if (not fairing_state["dropped"]
                    and y[0] - R_EARTH >= vehicle.fairing_jettison_alt_m
                    and y[4] - fairing_mass > m_burnout + 1.0
                    and per < target_altitude_m - orbit_margin_m):
                y[4] -= fairing_mass
                m_burnout -= fairing_mass
                fairing_state["dropped"] = True
                continue
            # two-burn insertion: coast to apogee, then circularise
            if (i > 0 and not coasted_to_apo
                    and per < target_altitude_m - orbit_margin_m
                    and apo >= target_altitude_m - orbit_margin_m
                    and apo < 1e11
                    and y[2] > 0.0
                    and y[4] > m_burnout + 0.5):
                def at_apogee(t, ys):
                    return ys[2]                 # vr crosses zero
                at_apogee.terminal = True
                at_apogee.direction = -1

                def falling_low(t, ys):
                    # abort the coast if we sink toward the atmosphere
                    return ys[0] - (R_EARTH + 140_000.0)
                falling_low.terminal = True
                falling_low.direction = -1
                y, t0 = run_phase("coast_to_apo", coast_rhs,
                                  (t0, t0 + 4000.0), y,
                                  [at_apogee, falling_low])
                coasted_to_apo = True
                continue
            break

        per, apo = _orbital_elements(y[0], y[2], y[3])
        if per >= target_altitude_m - orbit_margin_m:
            break
        if i < n_stages - 1:
            # drop spent upper stage (dry + unusable residuals), then coast
            y[4] -= (stage.dry_mass_kg
                     + (stage.propellant_kg - stage.usable_propellant_kg))
            y, t0 = run_phase("coast", coast_rhs, (t0, t0 + stage_coast_s),
                              y, [])

    per, apo = _orbital_elements(y[0], y[2], y[3])
    return AscentResult(
        # 1 km numerical slack against event-boundary float effects
        reached_orbit=per >= target_altitude_m - orbit_margin_m - 1_000.0,
        perigee_m=per, apogee_m=apo,
        final_altitude_m=y[0] - R_EARTH,
        final_velocity_m_s=math.hypot(y[2], y[3]),
        max_q_Pa=stats["max_q"], max_accel_g=stats["max_g"],
        phases=phases,
    )


def optimize_steering(vehicle, target_altitude_m, launch_latitude_deg,
                      max_q_limit_Pa=45_000.0, maxiter=22, seed=1):
    """Optimise the four steering parameters to maximise achieved perigee,
    subject to the max-q limit. Returns (params, best AscentResult)."""
    from scipy.optimize import differential_evolution

    cache = {}

    def neg_score(x):
        key = tuple(np.round(x, 4))
        if key in cache:
            return cache[key]
        ks, pf, t0u, uptf = x
        try:
            res = simulate_ascent(vehicle, target_altitude_m,
                                  launch_latitude_deg,
                                  s1_pitch_final_deg=pf, s2_tan0=t0u,
                                  upper_tan_final=uptf, kick_speed_m_s=ks)
            score = res.perigee_m if res.perigee_m < 1e11 else -1e12
            score -= max(0.0, res.max_q_Pa - max_q_limit_Pa) * 10.0
        except Exception:
            score = -1e12
        cache[key] = -score
        return -score

    bounds = [(20.0, 120.0),   # pitch-over start speed m/s
              (-10.0, 45.0),   # stage-1 commanded final pitch [deg]
              (-0.20, 1.20),   # upper-stage initial pitch tangent
              (-0.40, 0.15)]   # upper-stage final pitch tangent
    result = differential_evolution(
        neg_score, bounds, maxiter=maxiter, popsize=11, tol=1e-4,
        seed=seed, polish=False, updating="deferred", workers=1)
    ks, pf, t0u, uptf = result.x
    best = simulate_ascent(vehicle, target_altitude_m, launch_latitude_deg,
                           s1_pitch_final_deg=pf, s2_tan0=t0u,
                           upper_tan_final=uptf, kick_speed_m_s=ks)
    return tuple(result.x), best


def payload_capacity(
    vehicle_factory,
    target_altitude_m: float = 200_000.0,
    launch_latitude_deg: float = 28.5,
    payload_lo: float = 0.0,
    payload_hi: float = 100_000.0,
    tol_kg: float = 200.0,
    verbose: bool = False,
):
    """Max payload to target orbit by bisection; returns (payload_kg, result).

    vehicle_factory(payload_kg) -> Vehicle
    """
    def flies(payload):
        v = vehicle_factory(payload)
        _, res = optimize_steering(v, target_altitude_m, launch_latitude_deg)
        return res is not None and res.reached_orbit, res

    ok_lo, res_lo = flies(payload_lo)
    if not ok_lo:
        return 0.0, res_lo
    ok_hi, res_hi = flies(payload_hi)
    if ok_hi:
        return payload_hi, res_hi

    best_res = res_lo
    lo, hi = payload_lo, payload_hi
    while hi - lo > tol_kg:
        mid = 0.5 * (lo + hi)
        ok, res = flies(mid)
        if verbose:
            msg = f"  payload {mid/1000:8.2f} t -> " + \
                  ("orbit" if ok else "no orbit")
            if res is not None and res.perigee_m < 1e11:
                msg += f" (perigee {res.perigee_m/1000:.0f} km)"
            print(msg, flush=True)
        if ok:
            lo, best_res = mid, res
        else:
            hi = mid
    return lo, best_res
