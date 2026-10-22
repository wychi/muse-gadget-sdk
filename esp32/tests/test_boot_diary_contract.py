# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
OPTION = "CONFIG_HOMEHUB_BOOT_DIARY"


def gated_block(text, needle):
    """Assert the needle lives inside one of the option's #if blocks:
    the nearest #if before it is the option's, with no #endif between."""
    pos = text.index(needle)
    gate = text.rindex(f"#if {OPTION}", 0, pos)
    assert "#endif" not in text[gate:pos], \
        f"{needle!r} not inside the {OPTION} gate"


class BootDiaryContractTest(unittest.TestCase):
    def test_command_advertised_and_dispatched_under_the_option(self):
        noise = (ROOT / "main/noise_control.cpp").read_text()
        gated_block(noise, 'add_command(commands, "device.bootlog"')
        gated_block(noise, 'strcmp(command, "device.bootlog")')
        gated_block(noise, "send_device_bootlog")

    def test_diary_installed_early_under_the_option(self):
        main = (ROOT / "main/main.c").read_text()
        gated_block(main, "boot_diary_init()")
        # The diary installs before the support log, so the support
        # log's sink chains onto the diary's and both see every line.
        self.assertLess(main.index("boot_diary_init()"),
                        main.index("diagnostic_log_init()"))

    def test_wiring(self):
        cmake = (ROOT / "main/CMakeLists.txt").read_text()
        self.assertIn('"boot_diary.c"', cmake)
        kconfig = (ROOT / "main/Kconfig.projbuild").read_text()
        self.assertIn("config HOMEHUB_BOOT_DIARY", kconfig)
        overlay = (ROOT / "devices/sdkconfig.walkie").read_text()
        self.assertIn("CONFIG_HOMEHUB_BOOT_DIARY=y", overlay)
        diary = (ROOT / "main/boot_diary.c").read_text()
        self.assertIn("RTC_NOINIT_ATTR", diary)
        self.assertIn("esp_log_set_vprintf", diary)


if __name__ == "__main__":
    unittest.main()
