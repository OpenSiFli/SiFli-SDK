#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


def write_report(status, output, name, project, board):
    failed = status == 'failed'
    suite = ET.Element(
        'testsuite',
        name='SiFli SDK builds',
        tests='1',
        failures='1' if failed else '0',
        errors='0',
        skipped='0',
    )
    testcase = ET.SubElement(
        suite,
        'testcase',
        classname=project,
        name=f'{board or "default board"}: {name}',
    )
    if failed:
        failure = ET.SubElement(testcase, 'failure', message='SDK build failed', type='build')
        failure.text = f'Project: {project}\nBoard: {board or "default board"}\nJob: {name}'

    report = ET.Element('testsuites')
    report.append(suite)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(report).write(output, encoding='utf-8', xml_declaration=True)


def main():
    parser = argparse.ArgumentParser(description='Write a JUnit report for one SDK build')
    parser.add_argument('--status', choices=('success', 'failed'), required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--name', required=True)
    parser.add_argument('--project', required=True)
    parser.add_argument('--board', default='')
    args = parser.parse_args()
    write_report(args.status, args.output, args.name, args.project, args.board)


if __name__ == '__main__':
    main()
