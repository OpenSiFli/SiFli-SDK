# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Helpers shared by suites for one *family* of examples.

Everything here knows about a particular set of DUTs: the log lines they print,
the service they register, the shell commands they export. That is deliberate --
these helpers exist so a suite does not have to re-derive it -- and it is why
they live outside :mod:`sdk_test.instruments`, which knows only the instrument
wire protocol and the host API built on it. An instrument must stay usable from
a suite that shares no helper at all (a Classic suite, a power meter).
"""
