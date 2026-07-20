"""Cost model invariants."""
from aeros.validate import falcon9_expendable, electron
from aeros.cost import first_unit_cost


def test_cost_positive_and_ordered():
    c_f9 = first_unit_cost(falcon9_expendable(22800)).total_usd
    c_el = first_unit_cost(electron(300)).total_usd
    assert c_el > 0 and c_f9 > c_el          # bigger rocket costs more


def test_business_factor_scales_hardware():
    v = falcon9_expendable(22800)
    c1 = first_unit_cost(v, business_factor=1.0)
    c2 = first_unit_cost(v, business_factor=0.5)
    assert c2.total_usd < 0.6 * c1.total_usd
