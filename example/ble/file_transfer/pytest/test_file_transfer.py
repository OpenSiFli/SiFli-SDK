# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""End-to-end peer path for the file_transfer example.

The DUT is a GATT server on legacy connectable advertising. It does not bring
up the classic stack. Notifications come from a 5-second CCCD timer (4-byte
packets) and the 0x1111 burst trigger (20 packets of 10 bytes).

The suite also covers the example's headline feature: a file pushed over the
``sifli_sd`` serial-transport service with the watchface download protocol.
``test_peer_pushes_a_file`` subscribes to the data characteristic, calls
``file_push`` (START / FILE_START / DATA / END / FINISH, with the CRC the DUT
checks), and then asserts the path the DUT logged and that the name appears in
the receive directory.
"""

from __future__ import annotations

import re
import time

import pytest

from sdk_test.suite_helpers import sifli_app
from sdk_test.instruments.bt_peer import CCCD_UUID
from sdk_test.instruments.errors import InstrumentTimeout

#: The serial-transport service ("sifli_sd") and its data characteristic, as
#: the DUT declares them (bf0_sibles_serial_trans_service.h).
SIFLI_SD_UUID = b"sifli_sd" + b"\x00" * 8
SIFLI_SD_DATA_UUID = b"sifli_sd" + bytes([0x00, 0x02, 0x00, 0x00]) + b"\x00" * 4

#: One 4-byte aligned payload; the instrument appends the CRC word.
FILE_NAME = "peer_test.bin"
FILE_PAYLOAD = bytes(range(32))

STATE: dict = {}


def _require(key: str):
    value = STATE.get(key)
    if value is None:
        pytest.skip(f"{key} was not set: an earlier test did not complete")
    return value


def test_dut_boots_and_advertises(dut):
    # The example starts the legacy connectable advertisement (running mode 0);
    # it also logs a "Broadcast ADV start resutl" line, which the helper skips.
    sifli_app.expect_ready(dut, expected_mode=0)


def test_peer_instrument_is_ready(bt_peer):
    bt_peer.ble.reset()


def test_peer_connects_to_the_dut(dut, bt_peer):
    STATE["link"] = sifli_app.connect_to_dut(dut, bt_peer)


def test_peer_discovers_the_service(bt_peer):
    STATE["service"] = sifli_app.discover_service(bt_peer, _require("link"))


def test_peer_writes_and_reads_the_value(dut, bt_peer):
    sifli_app.write_then_read(
        dut, bt_peer, _require("link"), _require("service")
    )


def test_peer_subscribes_to_notifications(dut, bt_peer):
    sifli_app.subscribe(dut, bt_peer, _require("link"), _require("service"))
    STATE["subscribed"] = True


def test_notification_burst_reaches_the_peer(dut, bt_peer):
    """The 0x1111 trigger starts the example's 20-packet notification loop.

    The CCCD timer also sends 4-byte packets; they are skipped by length, and
    ``since_seq`` keeps a packet from before the trigger out of the burst.
    """
    link = _require("link")
    service = _require("service")
    _require("subscribed")

    start = bt_peer.console.last_seq
    bt_peer.ble.write(link, service.value_handle(), b"\x11\x11")
    dut.expect(r"example of ble notification", timeout=10)

    deadline = time.monotonic() + 15.0
    payloads = []
    while len(payloads) < 20 and time.monotonic() < deadline:
        try:
            payload = sifli_app.wait_notification(
                bt_peer,
                link,
                service,
                since_seq=start,
                timeout=max(0.1, deadline - time.monotonic()),
            )
        except InstrumentTimeout:
            break
        if len(payload) == 10:
            payloads.append(payload)

    assert len(payloads) == 20, f"got {len(payloads)} burst packets"
    for index, payload in enumerate(payloads):
        assert payload == bytes([index]) * 10, (
            f"burst packet {index} carried {payload.hex()}"
        )


def test_peer_pushes_a_file(dut, bt_peer):
    """The watchface protocol writes a file the DUT then reports on disk.

    The data characteristic carries both directions: requests are writes, every
    response is a notification, so the CCCD is subscribed before the push. The
    instrument checks each response's result word, which is what makes a
    rejected step a failure here instead of a timeout later.
    """
    link = _require("link")

    service = bt_peer.ble.discover(link, SIFLI_SD_UUID)
    data = service.characteristic(SIFLI_SD_DATA_UUID)
    bt_peer.ble.subscribe(link, data.descriptor_handle(CCCD_UUID))

    bt_peer.ble.file_push(link, data.value_handle, FILE_NAME, FILE_PAYLOAD)

    # The DUT logs the path it opened, then the length check of the whole
    # transfer; neither line appears unless the CRC and the sizes agreed.
    name_pattern = re.escape(FILE_NAME)
    dut.expect(rf"file: /file_recv/{name_pattern}", timeout=15)
    dut.expect(r"file recv success!", timeout=15)

    # The log says it wrote; the file system says it is there.
    dut.send_line("ls /file_recv")
    dut.expect(name_pattern, timeout=10)


#: Larger than one instrument chunk, so the push has to split the file and keep
#: the running CRC across packets.
LARGE_NAME = "peer_multi.bin"
LARGE_PAYLOAD = bytes((index * 7) & 0xFF for index in range(52))


def test_peer_pushes_a_multi_chunk_file(dut, bt_peer):
    """The same exchange with a file that does not fit one packet.

    Each data request carries an index the DUT checks for order, and the CRC is
    only valid if every chunk landed in sequence, so a green run here says the
    chunking and the reassembly agree.
    """
    link = _require("link")

    service = bt_peer.ble.discover(link, SIFLI_SD_UUID)
    data = service.characteristic(SIFLI_SD_DATA_UUID)

    bt_peer.ble.file_push(link, data.value_handle, LARGE_NAME, LARGE_PAYLOAD)

    dut.expect(rf"file: /file_recv/{re.escape(LARGE_NAME)}", timeout=20)
    dut.expect(r"file recv success!", timeout=20)

    dut.send_line("ls /file_recv")
    dut.expect(rf"{re.escape(LARGE_NAME)}\s+52", timeout=10)
