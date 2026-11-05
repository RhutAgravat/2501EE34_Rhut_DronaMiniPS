def test_circle_too_small_is_rejected():
    from planner import circle_slots
    from validator import validate_slots
    assert not validate_slots(circle_slots(4,.1,1.)).ok

def test_valid_four_circle():
    from planner import circle_slots
    from validator import validate_slots
    assert validate_slots(circle_slots(4,.42,1.)).ok
