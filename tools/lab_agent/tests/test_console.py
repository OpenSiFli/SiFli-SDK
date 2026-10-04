# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Tests for the console stream: cursor arithmetic and chunk tolerance.

This is the piece that makes a lab run reproducible. If it loses or duplicates
text, a test fails for reasons that have nothing to do with the firmware, and the
symptom looks like a flaky device.
"""

from __future__ import annotations

import time
import unittest

from _support import FakeSerialTransport
from _support import LabTestCase

from lab_agent.errors import TransportError
from lab_agent.testbed import ConsoleStream

BOOT_LOG = " | / - RT - Thread\r\nHello world!\r\nmsh />"


def make_stream(script: str = "", chunk: int = 1, max_buffer: int = 4096, poll: float = 0.005):
    transport = FakeSerialTransport(script=script, chunk=chunk)
    stream = ConsoleStream(transport, max_buffer=max_buffer, poll=poll)
    return transport, stream


def wait_for(stream: ConsoleStream, cursor: int, timeout: float = 3.0):
    """Read until something new arrives, or give up."""
    deadline = time.monotonic() + timeout
    data = ""
    while time.monotonic() < deadline:
        chunk, cursor = stream.read_from(cursor, 0.05)
        data += chunk
        if data:
            return data, cursor
    return data, cursor


class TestLifecycle(LabTestCase):
    def test_start_opens_the_transport_and_stop_closes_it(self):
        transport, stream = make_stream("hi")
        stream.start()
        self.assertTrue(stream.is_open)
        self.assertEqual(transport.opened, 1)
        stream.stop()
        self.assertFalse(stream.is_open)
        self.assertEqual(transport.closed, 1)

    def test_start_is_idempotent(self):
        transport, stream = make_stream("hi")
        stream.start()
        stream.start()
        self.assertEqual(transport.opened, 1)
        stream.stop()

    def test_stop_without_start_is_harmless(self):
        transport, stream = make_stream("hi")
        stream.stop()
        self.assertEqual(transport.closed, 1)  # close() is idempotent at the port


class TestChunkTolerance(LabTestCase):
    def test_a_transcript_dripped_one_character_at_a_time_reassembles(self):
        _transport, stream = make_stream(BOOT_LOG, chunk=1)
        stream.start()
        self.addCleanup(stream.stop)

        data, cursor = wait_for(stream, 0)
        self.assertIn("Hello world!", data)
        self.assertEqual(cursor, len(BOOT_LOG))

    def test_larger_chunks_produce_identical_text(self):
        _transport, stream = make_stream(BOOT_LOG, chunk=64)
        stream.start()
        self.addCleanup(stream.stop)

        data, cursor = wait_for(stream, 0)
        self.assertEqual(data, BOOT_LOG)

    def test_reads_are_incremental_and_never_repeat(self):
        _transport, stream = make_stream("abcdef", chunk=2)
        stream.start()
        self.addCleanup(stream.stop)

        first, cursor = wait_for(stream, 0)
        self.assertTrue(first)
        # Reading again from the advanced cursor must not return what we saw.
        second, cursor2 = wait_for(stream, cursor)
        self.assertFalse(second.startswith(first))
        self.assertEqual(first + second, "abcdef")
        self.assertEqual(cursor2, 6)


class TestCursorSemantics(LabTestCase):
    def test_a_read_with_nothing_new_returns_an_empty_string_and_the_same_cursor(self):
        _transport, stream = make_stream("done")
        stream.start()
        self.addCleanup(stream.stop)
        _data, cursor = wait_for(stream, 0)

        data, again = stream.read_from(cursor, 0.05)
        self.assertEqual(data, "")
        self.assertEqual(again, cursor)

    def test_a_zero_timeout_does_not_block(self):
        _transport, stream = make_stream("")
        stream.start()
        self.addCleanup(stream.stop)

        start = time.monotonic()
        data, cursor = stream.read_from(0, 0.0)
        self.assertEqual(data, "")
        self.assertEqual(cursor, 0)
        self.assertLess(time.monotonic() - start, 1.0)

    def test_cursor_is_the_end_of_everything_received(self):
        _transport, stream = make_stream("hello")
        stream.start()
        self.addCleanup(stream.stop)
        wait_for(stream, 0)
        self.assertEqual(stream.cursor, 5)

    def test_transcript_exposes_the_retained_text(self):
        _transport, stream = make_stream("hello")
        stream.start()
        self.addCleanup(stream.stop)
        wait_for(stream, 0)
        self.assertEqual(stream.transcript, "hello")


class TestTrimming(LabTestCase):
    def test_cursors_stay_absolute_when_the_buffer_is_trimmed(self):
        # A chatty device must not be able to grow the runner's memory without
        # bound, and trimming must not make a later read repeat old text.
        _transport, stream = make_stream("abcdefghij", chunk=1, max_buffer=4)
        stream.start()
        self.addCleanup(stream.stop)

        _data, cursor = wait_for(stream, 0)
        self.assertEqual(cursor, 10)
        self.assertLessEqual(len(stream.transcript), 4)
        # The tail is retained...
        self.assertTrue(stream.transcript.endswith("j"))

        # ...and reading from an offset that has been trimmed away resumes at
        # the oldest retained byte instead of raising or replaying.
        data, new_cursor = stream.read_from(0, 0.05)
        self.assertEqual(new_cursor, 10)
        self.assertEqual(data, stream.transcript)


class TestWrite(LabTestCase):
    def test_writes_reach_the_transport_verbatim(self):
        transport, stream = make_stream("")
        stream.start()
        self.addCleanup(stream.stop)

        stream.write("unity_run lz4\r\n")
        self.assertEqual(transport.sent, "unity_run lz4\r\n")


class TestErrors(LabTestCase):
    def test_a_transport_failure_is_raised_to_the_reader(self):
        transport, stream = make_stream("")
        stream.start()
        self.addCleanup(stream.stop)

        transport.fail_reads = True
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            try:
                stream.read_from(0, 0.05)
            except TransportError as exc:
                self.assertIn("Console read failed", str(exc))
                return
        self.fail("the pump thread's error never reached the reader")

    def test_a_waiting_reader_is_woken_by_stop(self):
        _transport, stream = make_stream("")
        stream.start()
        start = time.monotonic()
        reader_done = []

        import threading

        def reader():
            stream.read_from(0, 5.0)
            reader_done.append(time.monotonic())

        thread = threading.Thread(target=reader, daemon=True)
        thread.start()
        time.sleep(0.1)
        stream.stop()
        thread.join(timeout=2.0)

        self.assertTrue(reader_done, "stop() should wake a blocked reader")
        self.assertLess(reader_done[0] - start, 3.0)


if __name__ == "__main__":
    unittest.main()
