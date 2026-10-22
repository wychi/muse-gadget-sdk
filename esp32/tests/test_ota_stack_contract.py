"""OTA reachability contract.

The 1.2.4 walkie build's runtime floor was ~20 KB free internal RAM,
which left no contiguous 16 KB block for the OTA task's stack -- every
device.ota was refused with "could not start ota task" and both
gadgets were stranded until a USB flash. Pin the budget so a future
edit cannot silently regress it: the OTA task stack stays at 8192
(Espressif's esp_https_ota example size), and a failed creation logs
the largest free internal block so the boot diary shows the margin.
"""

import re
import unittest
from pathlib import Path

OTA_C = Path(__file__).resolve().parent.parent / "main" / "ota.c"


class OtaStackContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.src = OTA_C.read_text(encoding="utf-8")

    def test_ota_task_stack_within_budget(self):
        m = re.search(r'xTaskCreate\(\s*ota_task\s*,\s*"ota"\s*,\s*(\d+)', self.src)
        self.assertIsNotNone(m, "ota_start must create the OTA task")
        self.assertLessEqual(
            int(m.group(1)),
            8192,
            "OTA task stack must stay <= 8192 so a contiguous block always "
            "exists in the running firmware's free internal RAM",
        )

    def test_failure_logs_heap_headroom(self):
        self.assertIn("heap_caps_get_largest_free_block", self.src)


if __name__ == "__main__":
    unittest.main()
