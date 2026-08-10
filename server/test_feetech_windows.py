"""Fixtures for intersect_safe_window — the absolute-grounding of joint ranges.

The soft box is relative to wherever the arm woke up; the EEPROM window is the
servo's own absolute truth. These pin the intersection semantics, especially
the fail-closed fallbacks (bad reading → soft box, never a wider range).
Values mirror the real arm's 2026-08-10 startup readings where noted.
"""
from server.feetech_controller import TICKS_PER_REV, intersect_safe_window


def test_eeprom_narrower_than_soft_box_wins():
    # shoulder_pitch on the real arm: home 2081, EEPROM [2042, 2165] (~11°).
    assert intersect_safe_window(2081, 1200, 2042, 2165) == (2042, 2165)


def test_soft_box_narrower_than_eeprom_wins():
    # wrist_roll-ish: wide-open EEPROM, soft box governs.
    assert intersect_safe_window(2195, 1200, 0, 4095) == (995, 3395)


def test_partial_overlap_intersects():
    # elbow on the real arm: home 2708, EEPROM [1725, 2721] — soft box
    # [1508, 3908] loses its top to the EEPROM ceiling.
    assert intersect_safe_window(2708, 1200, 1725, 2721) == (1725, 2721)


def test_home_at_eeprom_edge():
    # wrist_pitch: home 127 == EEPROM min — window must still contain home.
    lo, hi = intersect_safe_window(127, 1200, 127, 2143)
    assert lo == 127 and hi == 1327


def test_missing_eeprom_falls_back_to_soft_box():
    assert intersect_safe_window(2048, 1200, None, None) == (848, 3248)


def test_degenerate_eeprom_window_ignored():
    # min >= max is garbage — fall back to the soft box, don't invert.
    assert intersect_safe_window(2048, 1200, 3000, 1000) == (848, 3248)


def test_home_outside_eeprom_window_falls_back():
    # Torque-off manual posing can park the arm outside its EEPROM goal
    # window; an intersection would be empty. Fall back to the soft box.
    assert intersect_safe_window(3500, 1200, 1000, 2000) == (2300, 4095)


def test_bounds_clamped_to_valid_ticks():
    lo, hi = intersect_safe_window(100, 1200, None, None)
    assert lo == 0 and hi == 1300
    lo, hi = intersect_safe_window(4000, 1200, None, None)
    assert hi == TICKS_PER_REV - 1
