# SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: CC0-1.0
import pytest
from pytest_embedded_qemu.dut import QemuDut


@pytest.mark.esp32  # we only support qemu on esp32 for now
@pytest.mark.host_test
@pytest.mark.qemu
def test_hello_world_host(dut: QemuDut) -> None:
    dut.expect("Hello world!")
