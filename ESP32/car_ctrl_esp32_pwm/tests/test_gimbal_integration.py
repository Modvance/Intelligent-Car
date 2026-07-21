import re
import unittest
from pathlib import Path


SKETCH = Path(__file__).resolve().parents[1] / "car_ctrl_esp32_pwm.ino"


class GimbalIntegrationContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SKETCH.read_text(encoding="utf-8")

    def test_uses_calibrated_centers_and_ranges(self):
        expected_constants = (
            "const int16_t PAN_CENTER = 93;",
            "const int16_t TILT_CENTER = 162;",
            "const int16_t PAN_MIN_ANGLE = 0;",
            "const int16_t PAN_MAX_ANGLE = 180;",
            "const int16_t TILT_MIN_ANGLE = 90;",
            "const int16_t TILT_MAX_ANGLE = 162;",
        )
        for declaration in expected_constants:
            self.assertIn(declaration, self.source)

    def test_clamps_packet_angles_per_axis(self):
        self.assertRegex(
            self.source,
            r"panAngle\s*=\s*clampServoAngle\(angles\[0\],\s*PAN_MIN_ANGLE,\s*PAN_MAX_ANGLE\)",
        )
        self.assertRegex(
            self.source,
            r"tiltAngle\s*=\s*clampServoAngle\(angles\[1\],\s*TILT_MIN_ANGLE,\s*TILT_MAX_ANGLE\)",
        )

    def test_starts_at_calibrated_center_without_sweep(self):
        self.assertRegex(
            self.source,
            r"initAngles\[2\]\s*=\s*\{PAN_CENTER,\s*TILT_CENTER\}",
        )
        self.assertNotIn("AUTORUN_SWEEP", self.source)
        self.assertNotIn("startSweep", self.source)

    def test_preserves_huawei_binary_packet_contract(self):
        self.assertIn("const uint8_t PACKET_SHORTS = 7;", self.source)
        self.assertIn("const int16_t CHECK_VAL = -12345;", self.source)
        self.assertTrue(re.search(r"packet\[CHECK\]\s*!=\s*CHECK_VAL", self.source))


if __name__ == "__main__":
    unittest.main()
