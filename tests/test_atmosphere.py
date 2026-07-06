"""Validate the atmosphere model against published USSA-1976 table values."""
import pytest
from aeros.atmosphere import atmosphere


# altitude [m], temperature [K], pressure [Pa], density [kg/m3]
USSA_TABLE = [
    (0,      288.15, 101_325.0, 1.2250),
    (5_000,  255.68, 54_048.0,  0.73643),
    (11_000, 216.65, 22_632.0,  0.36392),
    (20_000, 216.65, 5_474.9,   0.088035),
    (32_000, 228.65, 868.02,    0.013225),
    (47_000, 270.65, 110.91,    0.0014275),
    (71_000, 214.65, 3.9564,    0.000064211),
]


@pytest.mark.parametrize("alt,T,P,rho", USSA_TABLE)
def test_ussa1976_table(alt, T, P, rho):
    # table altitudes are geopotential; convert to geometric input
    R = 6_356_766.0
    z = R * alt / (R - alt)
    st = atmosphere(z)
    assert st.temperature_K == pytest.approx(T, rel=2e-3)
    assert st.pressure_Pa == pytest.approx(P, rel=5e-3)
    assert st.density_kg_m3 == pytest.approx(rho, rel=5e-3)


def test_monotonic_pressure():
    last = atmosphere(0).pressure_Pa
    for alt in range(1000, 120_000, 1000):
        p = atmosphere(alt).pressure_Pa
        assert p < last
        last = p
