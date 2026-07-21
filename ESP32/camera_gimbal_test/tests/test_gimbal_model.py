import sys
import unittest
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parents[1]
WORKSPACE = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(PROJECT_DIR))

from gimbal_model import (
    PAN_MAX_ANGLE,
    PAN_MIN_ANGLE,
    TILT_MAX_ANGLE,
    TILT_MIN_ANGLE,
    build_sweep_sequence,
    clamp_pan_angle,
    clamp_tilt_angle,
)


class GimbalRangeTest(unittest.TestCase):
    def test_control_firmwares_boot_at_the_calibrated_position(self):
        pwm_source = (WORKSPACE / "Car22" / "ESP32" / "car_ctrl_esp32_pwm" / "car_ctrl_esp32_pwm.ino").read_text(encoding="utf-8")
        gmr_source = (WORKSPACE / "Car22" / "ESP32" / "car_ctrl_esp32_gmr" / "car_ctrl_esp32_gmr.ino").read_text(encoding="utf-8")

        self.assertIn("const int16_t PAN_CENTER = 93;", pwm_source)
        self.assertIn("const int16_t TILT_CENTER = 162;", pwm_source)
        self.assertIn("int16_t initAngles[2] = {PAN_CENTER, TILT_CENTER};", pwm_source)
        self.assertIn("int16_t initAngles[2] = {93, 162};", gmr_source)
        self.assertNotIn("initAngles[2] = {90, 65}", gmr_source)

    def test_clamps_each_axis_to_its_installed_range(self):
        self.assertEqual(clamp_pan_angle(-5), PAN_MIN_ANGLE)
        self.assertEqual(clamp_pan_angle(200), PAN_MAX_ANGLE)
        self.assertEqual(clamp_tilt_angle(0), TILT_MIN_ANGLE)
        self.assertEqual(clamp_tilt_angle(180), TILT_MAX_ANGLE)

    def test_sweep_visits_full_range_then_returns_to_custom_center(self):
        sequence = build_sweep_sequence(93, 162)

        self.assertEqual(sequence, [(0, 162), (180, 162), (93, 162), (93, 90), (93, 162), (93, 162)])


if __name__ == "__main__":
    unittest.main()
