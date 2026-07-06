"""Publication-quality figures: vehicle drawings, ascent profiles, validation.

Everything renders with matplotlib only - no CAD kernel required - so the
full visual output works on any machine. (STEP-file CAD export is available
separately via the optional cadquery extra.)
"""

from __future__ import annotations

import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .materials import PROPELLANTS
from .trajectory import AscentResult, R_EARTH
from .vehicle import Vehicle

STAGE_COLORS = ["#4878CF", "#6ACC65", "#D65F5F", "#B47CC7"]


def draw_vehicle(vehicle: Vehicle, ax=None, annotate=True):
    """Scale engineering drawing of the vehicle (side view)."""
    own_fig = ax is None
    if own_fig:
        fig, ax = plt.subplots(figsize=(4, 10))

    y = 0.0
    total_len = 0.0
    sections = []
    for i, s in enumerate(vehicle.stages):
        prop = PROPELLANTS[s.engine.propellants]
        vol = s.propellant_kg / prop.bulk_density * 1.05
        area = math.pi / 4 * s.diameter_m ** 2
        length = vol / area + 0.6 * s.diameter_m   # tanks + inter-tank/engine bay
        sections.append((f"Stage {i+1}", length, s.diameter_m,
                         STAGE_COLORS[i % len(STAGE_COLORS)], s))
        total_len += length

    dia_max = max(s.diameter_m for s in vehicle.stages)
    fairing_len = 2.2 * dia_max
    total_len += fairing_len

    y = 0.0
    for name, length, dia, color, s in sections:
        rect = plt.Rectangle((-dia / 2, y), dia, length,
                             facecolor=color, edgecolor="k", lw=1.2, alpha=0.85)
        ax.add_patch(rect)
        # engines
        n_show = min(s.n_engines, 5)
        eng_w = dia / (n_show * 1.6)
        for k in range(n_show):
            x0 = -dia / 2 + (k + 0.5) * dia / n_show - eng_w / 2
            bell = plt.Polygon([(x0, y), (x0 + eng_w, y),
                                (x0 + eng_w * 0.8, y - dia * 0.18),
                                (x0 + eng_w * 0.2, y - dia * 0.18)],
                               facecolor="#333", edgecolor="k", lw=0.6)
            if y == 0.0 or True:
                pass
            ax.add_patch(bell)
        if annotate:
            ax.annotate(
                f"{name}\n{s.n_engines}x {s.engine.name}\n"
                f"{s.propellant_kg/1000:.1f} t prop",
                xy=(dia / 2, y + length / 2), xytext=(dia_max * 0.9, y + length / 2),
                fontsize=8, va="center",
                arrowprops=dict(arrowstyle="-", lw=0.7))
        y += length

    # fairing: ogive-ish nose
    xs = np.linspace(-dia_max / 2, dia_max / 2, 60)
    nose = y + fairing_len * (1 - (np.abs(xs) / (dia_max / 2)) ** 1.7)
    ax.fill_between(xs, y, nose, facecolor="#C4C4C4", edgecolor="k", lw=1.2)
    if annotate and vehicle.payload_kg:
        ax.annotate(f"payload\n{vehicle.payload_kg/1000:.2f} t",
                    xy=(0, y + fairing_len * 0.4), fontsize=8,
                    ha="center", va="center")

    ax.set_xlim(-dia_max * 2.2, dia_max * 3.2)
    ax.set_ylim(-dia_max * 0.6, (y + fairing_len) * 1.04)
    ax.set_aspect("equal")
    ax.set_title(f"{vehicle.name}\nGLOW {vehicle.glow_kg/1000:.1f} t, "
                 f"{y + fairing_len:.0f} m", fontsize=10)
    ax.axis("off")
    if own_fig:
        return fig
    return ax


def plot_ascent(result: AscentResult, title="Ascent profile"):
    """Four-panel ascent trajectory figure."""
    t = result.time_s
    alt = result.altitude_m / 1000
    vel = result.velocity_m_s / 1000
    m = result.mass_kg / 1000

    from .atmosphere import atmosphere
    q = np.array([0.5 * atmosphere(a * 1000).density_kg_m3 * (v * 1000) ** 2
                  for a, v in zip(alt, vel)]) / 1000

    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    axes[0, 0].plot(t, alt, lw=1.8)
    axes[0, 0].set_ylabel("altitude [km]")
    axes[0, 1].plot(t, vel, lw=1.8, color="#D65F5F")
    axes[0, 1].set_ylabel("velocity [km/s]")
    axes[1, 0].plot(t, q, lw=1.8, color="#6ACC65")
    axes[1, 0].set_ylabel("dynamic pressure [kPa]")
    axes[1, 0].set_xlabel("time [s]")
    axes[1, 1].plot(t, m, lw=1.8, color="#B47CC7")
    axes[1, 1].set_ylabel("mass [t]")
    axes[1, 1].set_xlabel("time [s]")
    for ax in axes.flat:
        ax.grid(alpha=0.3)
    fig.suptitle(
        f"{title}\nperigee {result.perigee_m/1000:.0f} km / apogee "
        f"{result.apogee_m/1000:.0f} km | max-q {result.max_q_Pa/1000:.0f} kPa"
        f" | max {result.max_accel_g:.1f} g", fontsize=11)
    fig.tight_layout()
    return fig


def plot_validation(rows: list[dict]):
    """Predicted vs published payload bar chart."""
    fig, ax = plt.subplots(figsize=(8, 4.5))
    names = [r["vehicle"].replace(" (expendable)", "\n(expendable)")
             for r in rows]
    x = np.arange(len(rows))
    pub = [r["published_kg"] / 1000 for r in rows]
    pred = [r["predicted_kg"] / 1000 for r in rows]
    ax.bar(x - 0.18, pub, 0.36, label="published", color="#4878CF")
    ax.bar(x + 0.18, pred, 0.36, label="AEROS predicted", color="#D65F5F")
    for i, r in enumerate(rows):
        ax.text(i + 0.18, pred[i] * 1.02, f"{r['error_pct']:+.1f}%",
                ha="center", fontsize=9)
    ax.set_xticks(x, names, fontsize=9)
    ax.set_ylabel("payload to LEO [t]")
    ax.set_yscale("log")
    ax.set_title("Trajectory engine validation against flown vehicles")
    ax.legend()
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    return fig


def design_sheet(design, path=None):
    """One-page design summary: drawing + ascent + key numbers."""
    fig = plt.figure(figsize=(14, 9))
    gs = fig.add_gridspec(2, 3, width_ratios=[1.0, 1.4, 1.4])

    ax_v = fig.add_subplot(gs[:, 0])
    draw_vehicle(design.vehicle, ax=ax_v)

    res = design.ascent
    t = res.time_s
    ax1 = fig.add_subplot(gs[0, 1])
    ax1.plot(t, res.altitude_m / 1000, lw=1.8)
    ax1.set_ylabel("altitude [km]")
    ax1.grid(alpha=0.3)
    ax2 = fig.add_subplot(gs[0, 2])
    ax2.plot(t, res.velocity_m_s / 1000, lw=1.8, color="#D65F5F")
    ax2.set_ylabel("velocity [km/s]")
    ax2.grid(alpha=0.3)
    ax3 = fig.add_subplot(gs[1, 1])
    ax3.plot(t, res.mass_kg / 1000, lw=1.8, color="#B47CC7")
    ax3.set_ylabel("mass [t]")
    ax3.set_xlabel("time [s]")
    ax3.grid(alpha=0.3)

    ax4 = fig.add_subplot(gs[1, 2])
    ax4.axis("off")
    v = design.vehicle
    txt = [f"MISSION  {design.mission.payload_kg/1000:.2f} t -> "
           f"{design.mission.target_altitude_m/1000:.0f} km",
           f"VERIFIED {'YES - orbit achieved' if design.verified else 'NO'}",
           f"GLOW     {v.glow_kg/1000:.1f} t",
           f"T/W      {v.liftoff_twr:.2f} at lift-off",
           f"perigee  {res.perigee_m/1000:.0f} km",
           f"max-q    {res.max_q_Pa/1000:.0f} kPa",
           f"max g    {res.max_accel_g:.1f}", ""]
    for s in v.stages:
        txt.append(f"{s.name}: {s.n_engines}x {s.engine.name}")
        txt.append(f"   prop {s.propellant_kg/1000:.1f} t | "
                   f"dry {s.dry_mass_kg/1000:.2f} t")
    ax4.text(0.02, 0.98, "\n".join(txt), family="monospace", fontsize=10,
             va="top", transform=ax4.transAxes)

    fig.suptitle("AEROS autonomous design - verification sheet", fontsize=13)
    fig.tight_layout()
    if path:
        fig.savefig(path, dpi=150)
    return fig
