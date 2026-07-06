"""Engine database consistency: the nozzle relation must reproduce the
published sea-level Isp of each engine within 3%."""
import pytest
from aeros.engines import ENGINES


@pytest.mark.parametrize("name", sorted(ENGINES))
def test_nozzle_relation_consistency(name):
    e = ENGINES[name]
    if e.thrust_sl_N is None or e.isp_sl_s is None:
        return
    isp_sl_model = e.isp_at(101_325.0)
    assert isp_sl_model == pytest.approx(e.isp_sl_s, rel=0.03), (
        f"{name}: model sea-level Isp {isp_sl_model:.1f}s vs "
        f"published {e.isp_sl_s}s")


def test_vacuum_thrust_exceeds_sl():
    for e in ENGINES.values():
        if e.thrust_sl_N is not None:
            assert e.thrust_vac_N > e.thrust_sl_N
