# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

from contextlib import redirect_stderr
from http.server import BaseHTTPRequestHandler, HTTPServer
import io
import json
import os
from pathlib import Path
import queue
import socket
import sqlite3
import time
import sys
import threading
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch
from urllib.error import HTTPError, URLError
from urllib.parse import parse_qs, unquote, urlsplit
from urllib.request import Request, urlopen

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools/ci'))
import ci_failure_notify as notify

PROJECT = 38
PROJECT_URL = 'http://gitlab.example/sw/SiFli-SDK'


def job(job_id, name, allow_failure=False):
    return {'id': job_id, 'name': name, 'status': 'failed', 'allow_failure': allow_failure,
            'web_url': f'{PROJECT_URL}/-/jobs/{job_id}'}


def bridge(job_id, name, child_id=None, child_status='failed', status='failed', **extra):
    child = {'id': child_id, 'status': child_status} if child_id else None
    if child and 'child_project' in extra:
        child['project_id'] = extra.pop('child_project')
    return dict({'id': job_id, 'name': name, 'status': status, 'allow_failure': False,
                 'downstream_pipeline': child, 'web_url': f'{PROJECT_URL}/-/jobs/{job_id}'}, **extra)


class FakeGitLab(notify.GitLab):
    """Routes API paths to canned data, paginates lists and records every call."""

    def __init__(self, routes, traces=None):
        super().__init__('http://gitlab.example', 'read-api-token')
        self.routes = routes
        self.traces = traces or {}
        self.calls = []

    def request(self, path, raw=False):
        self.calls.append(path)
        if path == 'version':
            return self.routes.get('version', {'version': '19.1.0'})
        project, _, path = path[len('projects/'):].partition('/')
        if raw:
            parts = path.split('/', 3)  # jobs/<id>/trace or jobs/<id>/artifacts/<path>
            key = (int(project), int(parts[1]))
            if parts[2] == 'artifacts':
                key += (unquote(parts[3]),)
            data = self.traces.get(key, notify.GitLabError('HTTP 404', 404))
            if isinstance(data, Exception):
                raise data
            return data
        parts = urlsplit(path)
        query = parse_qs(parts.query)
        page = int(query.pop('page', ['1'])[0])
        per_page = int(query.pop('per_page', ['20'])[0])
        key = parts.path + ('?' + '&'.join(f'{k}={v[0]}' for k, v in sorted(query.items()))
                            if query else '')
        if parts.path.endswith('/trigger_jobs') and key not in self.routes:
            raise notify.GitLabError('HTTP 404', 404)
        if int(project) != PROJECT:
            key = f'{project}:{key}'
        # A pipeline is still failed unless a test says otherwise.
        default = {'status': 'failed'} if parts.path.count('/') == 1 else []
        data = self.routes.get(key, default)
        if isinstance(data, Exception):
            raise data
        if isinstance(data, list):
            return data[(page - 1) * per_page:page * per_page]
        return data


def main_routes():
    """The real shape of both branches: parent -> one child holding matrix build jobs."""
    return {
        'pipelines/100/bridges': [
            bridge(90, 'trigger_child_pipeline', 190, retried=True),  # an older attempt
            bridge(101, 'trigger_child_pipeline', 200),
        ],
        'pipelines/100/jobs?scope[]=failed': [],
        'pipelines/100/jobs?scope[]=success': [job(150, 'generate_dynamic_pipeline'),
                                                job(151, 'artifact_catalog')],
        'pipelines/200/bridges': [],
        'pipelines/200/jobs?scope[]=failed': [
            job(211, 'build_batch_002: [example_get_started_sf32lb52-lcd_n16r8]'),
            job(210, 'build_batch_001: [example_hello_world_eh-lb563]'),
            job(230, 'collect_batch_001'),
            job(212, 'build_batch_001: [smoke_expected_failure]', allow_failure=True),
        ],
        'pipelines/190/jobs?scope[]=failed': [job(199, 'must_not_be_listed')],
    }


TRACES = {
    (PROJECT, 210): (
        '\x1b[0Ksection_start:1700000000:build\r\x1b[0K\x1b[32;1mRunning scons\x1b[0;m\n'
        'src/main.c:12:5: warning: unused variable [-Wunused-variable]\n'
        "src/main.c:15:3: \x1b[01;31merror: \x1b[m'foo' undeclared (first use in this function)\n"
        'scons: *** [build/main.o] Error 1\n'
        'ERROR: Job failed: exit code 1\n'),
    (PROJECT, 211): 'Compiling...\nERROR: Job failed: exit code 1\n',
}


def payload(**attrs):
    base = {'id': 100, 'ref': 'main', 'tag': False, 'sha': 'abcdef1234567890', 'status': 'failed',
            'source': 'schedule', 'url': f'{PROJECT_URL}/-/pipelines/100', 'variables': []}
    base.update(attrs)
    return {'object_kind': 'pipeline', 'object_attributes': base,
            'project': {'id': PROJECT, 'web_url': PROJECT_URL},
            'commit': {'title': '[bug][dfu] fix `pingpong` dfu issue'}}


def listed(message):
    """Link texts of the numbered job lines."""
    return [line.split('[', 1)[1].rsplit('](', 1)[0]
            for line in message.splitlines() if line[:1].isdigit()]


class Quiet(unittest.TestCase):
    def setUp(self):
        self.stderr = redirect_stderr(io.StringIO())
        self.log = self.stderr.__enter__()
        self.addCleanup(self.stderr.__exit__, None, None, None)


class NotifierTests(Quiet):
    def setUp(self):
        super().setUp()
        tmp = TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.state_path = Path(tmp.name) / 'state' / 'notified.json'
        self.sent = []

    def notifier(self, gitlab, send=None):
        return notify.Notifier(gitlab, 'bot-key', {'main', 'release/v2.5'},
                               notify.State(self.state_path),
                               send or (lambda key, content: self.sent.append((key, content))))

    def test_child_matrix_failures_are_listed_once(self):
        gitlab = FakeGitLab(main_routes(), TRACES)
        notifier = self.notifier(gitlab)

        self.assertTrue(notifier.handle(payload()))
        self.assertEqual(len(self.sent), 1)
        key, message = self.sent[0]
        self.assertEqual(key, 'bot-key')
        self.assertIn('main', message.splitlines()[0])
        self.assertIn('[#100](http://gitlab.example/sw/SiFli-SDK/-/pipelines/100)', message)
        self.assertIn('`abcdef1234`', message)
        # Backticks in the commit title must not break the inline code markup.
        self.assertIn("fix 'pingpong' dfu issue", message)
        self.assertIn('失败任务：<font color="warning">3</font> 个', message)
        # Matrix brackets would end the markdown link text early.
        self.assertEqual(listed(message), [
            'build_batch_001: (example_hello_world_eh-lb563)',
            'build_batch_002: (example_get_started_sf32lb52-lcd_n16r8)',
            'collect_batch_001',
        ])
        self.assertIn(f'[build_batch_001: (example_hello_world_eh-lb563)]({PROJECT_URL}/-/jobs/210)',
                      message)
        self.assertNotIn('smoke_expected_failure', message)
        self.assertNotIn('must_not_be_listed', message)
        self.assertIn("`src/main.c:15:3: error: 'foo' undeclared (first use in this function)`",
                      message)
        self.assertNotIn('Job failed', message)
        self.assertIn(f'{PROJECT_URL}/-/jobs/151/artifacts/browse/artifact_catalog/', message)
        self.assertFalse(any('pipelines/190' in call for call in gitlab.calls))

        self.assertFalse(notifier.handle(payload()))
        self.assertFalse(self.notifier(FakeGitLab(main_routes(), TRACES)).handle(payload()))
        self.assertEqual(len(self.sent), 1)
        self.assertEqual(json.loads(self.state_path.read_text()), {'sent': ['100'], 'pending': []})

    def test_long_lists_are_truncated_within_limit(self):
        failed = [job(1000 + i, f'build_batch_{i // 50 + 1:03d}: [example_project_{i:03d}_sf32lb52-lcd_n16r8]')
                  for i in range(250)]
        routes = {
            'pipelines/100/bridges': [bridge(101, 'trigger_child_pipeline', 200)],
            'pipelines/200/jobs?scope[]=failed': failed,
        }
        notifier = self.notifier(FakeGitLab(routes))

        self.assertTrue(notifier.handle(payload(ref='release/v2.5', source='web')))
        message = self.sent[0][1]
        self.assertLessEqual(len(message.encode('utf-8')), notify.MAX_MESSAGE_BYTES)
        self.assertIn('失败任务：<font color="warning">250</font> 个', message)
        shown = len(listed(message))
        self.assertGreater(shown, 10)
        self.assertIn(f'另有 {250 - shown} 个失败任务未列出', message)
        self.assertNotIn('产物目录', message)

    def test_message_byte_limit_includes_unusually_long_header_and_footer(self):
        info = {'id': 1, 'ref': '分支' * 5000, 'sha': 'abc', 'source': 'web',
                'web_url': 'https://gitlab.example/' + 'x' * 5000,
                'commit_title': '标题' * 5000}
        message = notify.build_message(info, [], {}, 'https://gitlab.example/' + 'c' * 5000)
        self.assertLessEqual(len(message.encode('utf-8')), notify.MAX_MESSAGE_BYTES)

    def test_payload_filters_skip_without_api_calls(self):
        cases = [
            {'object_kind': 'push'},
            payload(status='success'),
            payload(status='running'),
            payload(source='merge_request_event'),
            payload(source='push'),
            payload(source='parent_pipeline'),
            payload(ref='feature/x'),
            payload(ref='main', tag=True),
            payload(variables=[{'key': 'CATALOG_SMOKE_TEST', 'value': '1'}]),
        ]
        for event in cases:
            with self.subTest(event=event.get('object_attributes', event)):
                gitlab = FakeGitLab(main_routes(), TRACES)
                self.assertFalse(self.notifier(gitlab).handle(event))
                self.assertEqual(gitlab.calls, [])
        self.assertEqual(self.sent, [])
        self.assertFalse(self.state_path.exists())

    def test_pipeline_rerun_since_the_event_is_not_reported(self):
        for status in ['running', 'success']:
            with self.subTest(status=status):
                routes = main_routes()
                routes['pipelines/100'] = {'status': status}
                self.assertFalse(self.notifier(FakeGitLab(routes, TRACES)).handle(payload()))
        self.assertEqual(self.sent, [])
        self.assertFalse(self.state_path.exists())

    def test_transient_child_error_fails_the_event_for_retry(self):
        routes = main_routes()
        routes['pipelines/200/jobs?scope[]=failed'] = notify.GitLabError('HTTP 502', 502)
        with self.assertRaises(notify.GitLabError):
            self.notifier(FakeGitLab(routes, TRACES)).handle(payload())
        self.assertEqual(self.sent, [])
        self.assertFalse(self.state_path.exists())

    def test_pipeline_without_child_build_is_not_reported(self):
        routes = {'pipelines/100/bridges': [], 'pipelines/100/jobs?scope[]=failed': [job(1, 'sync')]}
        self.assertFalse(self.notifier(FakeGitLab(routes)).handle(payload(source='api')))
        self.assertEqual(self.sent, [])
        self.assertFalse(self.state_path.exists())

    def test_failed_send_leaves_pipeline_unrecorded(self):
        def broken(key, content):
            raise ValueError('cannot reach WeCom: URLError')

        with self.assertRaises(ValueError):
            self.notifier(FakeGitLab(main_routes(), TRACES), broken).handle(payload())
        self.assertFalse(self.state_path.exists())
        # The worker's retry calls handle again; it must still send.
        self.assertTrue(self.notifier(FakeGitLab(main_routes(), TRACES)).handle(payload()))
        self.assertEqual(len(self.sent), 1)

    def test_state_is_reserved_before_send(self):
        notifier = self.notifier(FakeGitLab(main_routes(), TRACES))
        with patch.object(notifier.state, 'add', side_effect=OSError('disk full')):
            with self.assertRaises(OSError):
                notifier.handle(payload())
        self.assertEqual(self.sent, [])

    def test_unreadable_log_and_catalog_do_not_block_the_message(self):
        routes = main_routes()
        routes['pipelines/100/jobs?scope[]=success'] = notify.GitLabError('HTTP 403', 403)
        self.assertTrue(self.notifier(FakeGitLab(routes, {})).handle(payload()))
        message = self.sent[0][1]
        self.assertIn('example_hello_world_eh-lb563', message)
        self.assertNotIn('`src/main.c', message)
        self.assertNotIn('产物目录', message)


class FailedJobsTests(Quiet):
    def names(self, routes, traces=None):
        return [job['name'] for job in notify.failed_jobs(FakeGitLab(routes, traces), PROJECT, 100)]

    def test_nested_bridges_are_expanded_parents_first(self):
        routes = {
            'pipelines/100/bridges': [bridge(101, 'trigger_child_pipeline', 200)],
            'pipelines/200/jobs?scope[]=failed': [job(220, 'collect')],
            'pipelines/200/bridges': [
                bridge(201, 'nested_a', 300),
                bridge(202, 'nested_ok', 301, child_status='success'),
                bridge(203, 'nested_no_child'),  # trigger failed, no downstream pipeline
                bridge(204, 'nested_allowed', 302, allow_failure=True),
            ],
            'pipelines/300/jobs?scope[]=failed': [job(311, 'deep_b'), job(310, 'deep_a')],
            'pipelines/301/jobs?scope[]=failed': [job(399, 'from_successful_child')],
            'pipelines/302/jobs?scope[]=failed': [job(398, 'from_allowed_child')],
        }
        self.assertEqual(self.names(routes), ['nested_no_child', 'collect', 'deep_a', 'deep_b'])

    def test_depth_limit_and_unreadable_child_keep_the_bridge(self):
        for code in (403, 404):
            routes = {'pipelines/100/bridges': [bridge(101, 'trigger_child_pipeline', 200)],
                      'pipelines/200/jobs?scope[]=failed': notify.GitLabError(f'HTTP {code}', code)}
            self.assertEqual(self.names(routes), ['trigger_child_pipeline'])
        for code in (500, None):
            routes['pipelines/200/jobs?scope[]=failed'] = notify.GitLabError('transient', code)
            with self.assertRaises(notify.GitLabError):
                self.names(routes)
        with patch.object(notify, 'MAX_DEPTH', 0):
            self.assertEqual(self.names({'pipelines/100/bridges': [
                bridge(101, 'trigger_child_pipeline', 200)]}), ['trigger_child_pipeline'])

    def test_nonterminal_child_keeps_failed_bridge(self):
        for child_status in ('canceled', 'skipped', None, 'running'):
            with self.subTest(child_status=child_status):
                routes = {
                    'pipelines/100/bridges': [bridge(101, 'trigger_child_pipeline', 200,
                                                     child_status=child_status)],
                    'pipelines/200/jobs?scope[]=failed': [],
                }
                self.assertEqual(self.names(routes), ['trigger_child_pipeline'])

    def test_trigger_jobs_api_is_selected_for_gitlab_19_2(self):
        routes = {
            'version': {'version': '19.2.0-ee'},
            'pipelines/100/trigger_jobs': [bridge(101, 'trigger_child_pipeline')],
        }
        gitlab = FakeGitLab(routes)
        self.assertTrue(notify.is_full_build(gitlab, PROJECT, 100))
        self.assertTrue(any('pipelines/100/trigger_jobs' in call for call in gitlab.calls))
        self.assertFalse(any('pipelines/100/bridges' in call for call in gitlab.calls))

    def test_cross_project_child_and_its_logs_use_that_project(self):
        routes = {
            'pipelines/100/bridges': [bridge(101, 'downstream', 500, child_project=77)],
            '77:pipelines/500/jobs?scope[]=failed': [job(510, 'other_build')],
        }
        gitlab = FakeGitLab(routes, {(77, 510): 'x.c:1: error: boom'})
        info = {'id': 100, 'ref': 'main', 'sha': 'abc', 'source': 'web',
                'web_url': f'{PROJECT_URL}/-/pipelines/100'}
        message = notify.collect(gitlab, PROJECT, info, PROJECT_URL)
        self.assertEqual(listed(message), ['other_build'])
        self.assertIn('`x.c:1: error: boom`', message)
        self.assertIn('projects/77/jobs/510/trace', gitlab.calls)


class ManualNotifyTests(Quiet):
    def test_pipeline_info_without_repository_access(self):
        routes = {'pipelines/100': {'id': 100, 'ref': 'main', 'sha': 'abc', 'source': 'schedule',
                                    'web_url': f'{PROJECT_URL}/-/pipelines/100'},
                  'repository/commits/abc': notify.GitLabError('HTTP 403', 403)}
        info = notify.pipeline_info(FakeGitLab(routes), PROJECT, 100)
        self.assertIsNone(info['commit_title'])
        routes['repository/commits/abc'] = notify.GitLabError('HTTP 502', 502)
        with self.assertRaises(notify.GitLabError):
            notify.pipeline_info(FakeGitLab(routes), PROJECT, 100)

    def test_failed_pipeline_without_failed_jobs_points_at_pages(self):
        info = {'id': 100, 'ref': 'main', 'sha': 'abc', 'source': 'schedule',
                'web_url': f'{PROJECT_URL}/-/pipelines/100'}
        message = notify.build_message(info, [], {})
        self.assertIn('失败任务：<font color="warning">0</font> 个', message)
        self.assertIn('pages:deploy', message)


class BuildLogTests(Quiet):
    def test_build_log_path(self):
        cases = {
            'build_batch_001: [example_hello_world_eh-lb563]': 'ci_build_logs/example_hello_world_eh-lb563.log',
            'build_example_get_started_sf32lb52_lcd_n16r8': 'ci_build_logs/example_get_started_sf32lb52_lcd_n16r8.log',
            'build_batch_001': None,
            'collect_batch_001': None,
            'generate_dynamic_pipeline': None,
        }
        for name, expected in cases.items():
            with self.subTest(name=name):
                self.assertEqual(notify.build_log_path(name), expected)

    def test_build_log_artifact_is_preferred_and_falls_back_to_job_log(self):
        name = 'build_example_a_board'
        log = 'ci_build_logs/example_a_board.log'
        trace = 'x.c:1: error: from job log'
        cases = [
            ({(PROJECT, 7, log): 'CC x.o\nlv_gpu.c:63:24: error: from artifact\n', (PROJECT, 7): trace},
             'lv_gpu.c:63:24: error: from artifact'),
            ({(PROJECT, 7, log): 'no errors here', (PROJECT, 7): trace}, trace),
            ({(PROJECT, 7, log): notify.GitLabError('HTTP 404', 404), (PROJECT, 7): trace}, trace),
            ({(PROJECT, 7, log): notify.GitLabError('HTTP 403', 403), (PROJECT, 7): trace}, trace),
        ]
        for traces, expected in cases:
            with self.subTest(expected=expected):
                gitlab = FakeGitLab({}, traces)
                self.assertEqual(notify.job_error(gitlab, PROJECT, job(7, name)), expected)
        gitlab = FakeGitLab({}, {(PROJECT, 7, log): notify.GitLabError('HTTP 502', 502), (PROJECT, 7): trace})
        with self.assertRaises(notify.GitLabError):
            notify.job_error(gitlab, PROJECT, job(7, name))

    def test_message_uses_build_log_of_matrix_job(self):
        traces = dict(TRACES)
        traces[(PROJECT, 211, 'ci_build_logs/example_get_started_sf32lb52-lcd_n16r8.log')] = (
            '/data/gitlab-runner/builds/ZzoyCm86M/7/sw/SiFli-SDK/src/app.c:3:1: error: boom\n')
        notifier = notify.Notifier(FakeGitLab(main_routes(), traces), 'k', {'main'},
                                   notify.State(Path(self.tmp()) / 'n.json'),
                                   lambda key, content: self.sent.append(content))
        self.sent = []
        notifier.handle(payload())
        self.assertIn('`src/app.c:3:1: error: boom`', self.sent[0])

    def tmp(self):
        tmp = TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        return tmp.name


class FirstErrorTests(unittest.TestCase):
    def test_gitlab_timestamps_and_runner_paths_are_stripped(self):
        trace = ('2026-11-15T19:02:01.430942Z 01O HEAD is now at 9dac34e [int] fix nvds error\n'
                 '2026-11-15T19:02:20.100000Z 01O+warning: something\n'
                 '2026-11-15T19:02:20.200000Z 01E /data/gitlab-runner/builds/ZzoyCm86M/7/sw/SiFli-SDK/'
                 "middleware/lv_gpu.c:63:24: error: implicit declaration of function 'lv_area_is_on'\n"
                 '2026-11-15T19:02:31.669083Z 00O ERROR: Job failed: exit status 1\n')
        self.assertEqual(notify.first_error(trace),
                         "middleware/lv_gpu.c:63:24: error: implicit declaration of function 'lv_area_is_on'")

    def test_error_patterns(self):
        cases = [
            'a.c:1:1: error: expected ;',
            "b.c:2: error: unused variable 'x' [-Werror=unused-variable]",
            'c.c:3:10: fatal error: foo.h: No such file or directory',
            'main.o: in function `f`: undefined reference to `g`',
            'scons: *** [build/x.o] Error 1',
        ]
        for trace in cases:
            with self.subTest(trace=trace):
                self.assertEqual(notify.first_error('ok\n' + trace + '\nmore'), trace)

    def test_non_errors(self):
        for trace in ['Build finished, 0 errors, 3 warnings', 'ERROR: Job failed: exit code 1',
                      'warning: error: looks scary but is a warning', 'compiling error_handler.c',
                      'CFLAGS=-Werror -Wall', '']:
            with self.subTest(trace=trace):
                self.assertIsNone(notify.first_error(trace))

    def test_long_line_is_truncated(self):
        error = notify.first_error('x.c:1: error: ' + 'y' * 500)
        self.assertEqual(len(error), notify.MAX_ERROR_CHARS)
        self.assertTrue(error.endswith('...'))


class StateTests(Quiet):
    def test_corrupt_or_wrong_state_starts_empty(self):
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / 'notified.json'
            for content in ['', '[1, 2', '{"a": 1}']:
                with self.subTest(content=content):
                    path.write_text(content)
                    state = notify.State(path)
                    self.assertNotIn(1, state)
                    state.add(5)
                    self.assertEqual(json.loads(path.read_text()), {'sent': [], 'pending': ['5']})

    def test_failed_write_rolls_back_in_memory_reservation(self):
        with TemporaryDirectory() as tmp:
            state = notify.State(Path(tmp) / 'notified.json')
            with patch.object(state, '_write', side_effect=OSError('disk full')):
                with self.assertRaises(OSError):
                    state.add(5)
            self.assertIsNone(state.status(5))
            state.add(5)
            self.assertEqual(state.status(5), 'pending')

    def test_failed_state_transitions_restore_pending_status(self):
        with TemporaryDirectory() as tmp:
            state = notify.State(Path(tmp) / 'notified.json')
            state.add(6)
            state.mark_sent(6)
            state.add(5)
            with patch.object(state, '_write', side_effect=OSError('disk full')):
                with self.assertRaises(OSError):
                    state.mark_sent(5)
                self.assertEqual(state.status(5), 'pending')
                with self.assertRaises(OSError):
                    state.remove(5)
            self.assertEqual(state.status(5), 'pending')

    def test_failed_pending_file_removal_restores_status(self):
        with TemporaryDirectory() as tmp:
            state = notify.State(Path(tmp) / 'notified.json')
            state.add(5)
            with patch.object(Path, 'unlink', side_effect=OSError('permission denied')):
                with self.assertRaises(OSError):
                    state.remove(5)
            self.assertEqual(state.status(5), 'pending')

    def test_state_keeps_all_notified_ids(self):
        with TemporaryDirectory() as tmp:
            state = notify.State(Path(tmp) / 'notified.json')
            for pipeline_id in range(503):
                state.add(pipeline_id)
            reloaded = notify.State(Path(tmp) / 'notified.json')
            self.assertIn(0, reloaded)
            self.assertIn(502, reloaded)


class DurableQueueTests(Quiet):
    def test_pending_events_and_retry_deadlines_survive_restart(self):
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / 'queue.sqlite3'
            jobs = notify.DurableQueue(path)
            jobs.put_nowait((payload(), 0))
            event, attempt, event_id = jobs.get()
            self.assertEqual((event['object_attributes']['id'], attempt), (100, 0))
            jobs.retry(event_id, 1, 0)

            restarted = notify.DurableQueue(path)
            event, attempt, event_id = restarted.get()
            self.assertEqual((event['object_attributes']['id'], attempt), (100, 1))
            restarted.complete(event_id)
            with sqlite3.connect(path) as db:
                self.assertEqual(db.execute('SELECT COUNT(*) FROM events').fetchone()[0], 0)

    def test_interrupted_delivery_is_quarantined_and_capacity_is_bounded(self):
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / 'queue.sqlite3'
            jobs = notify.DurableQueue(path, maxsize=1)
            jobs.put_nowait((payload(), 0))
            jobs.get()
            with self.assertRaises(queue.Full):
                jobs.put_nowait((payload(id=101), 0))

            with sqlite3.connect(path) as db:
                db.execute('UPDATE events SET claimed_at=?',
                           (time.time() - notify.PROCESSING_LEASE_SECONDS - 1,))
            notify.DurableQueue(path, maxsize=1)
            with sqlite3.connect(path) as db:
                status = db.execute('SELECT status FROM events').fetchone()[0]
            self.assertEqual(status, 'uncertain')

    def test_concurrent_consumers_claim_distinct_events(self):
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / 'queue.sqlite3'
            select_count = 0
            select_lock = threading.Lock()
            both_selected = threading.Event()

            class GatedCursor(sqlite3.Cursor):
                def execute(self, sql, parameters=()):
                    self.is_claim_select = sql.startswith(
                        'SELECT id, payload, attempt FROM events')
                    return super().execute(sql, parameters)

                def fetchone(self):
                    row = super().fetchone()
                    callback = getattr(self.connection, 'after_claim_select', None)
                    if row and self.is_claim_select and callback:
                        callback()
                    return row

            class GatedConnection(sqlite3.Connection):
                def cursor(self, factory=None):
                    return super().cursor(factory or GatedCursor)

            class RacingQueue(notify.DurableQueue):
                def _connect(self):
                    db = sqlite3.connect(self.path, timeout=30, factory=GatedConnection)
                    db.execute('PRAGMA synchronous=FULL')

                    def synchronize_claim():
                        nonlocal select_count
                        with select_lock:
                            select_count += 1
                            if select_count == 2:
                                both_selected.set()
                        both_selected.wait(0.5)

                    db.after_claim_select = synchronize_claim
                    return db

            first = RacingQueue(path)
            second = RacingQueue(path)
            first.put_nowait((payload(), 0))
            start = threading.Barrier(3)
            results = queue.Queue()

            def consume(jobs):
                start.wait()
                try:
                    results.put(jobs.get())
                except Exception as exc:
                    results.put(exc)

            threads = [threading.Thread(target=consume, args=(jobs,), daemon=True)
                       for jobs in (first, second)]
            for thread in threads:
                thread.start()
            start.wait()
            event_a = results.get(timeout=3)
            self.assertNotIsInstance(event_a, Exception)
            with self.assertRaises(queue.Empty):
                results.get(timeout=0.1)
            self.assertFalse(both_selected.is_set())

            first.put_nowait((payload(id=101), 0))
            event_b = results.get(timeout=3)
            self.assertNotIsInstance(event_b, Exception)
            for thread in threads:
                thread.join(1)
                self.assertFalse(thread.is_alive())
            self.assertEqual({event_a[0]['object_attributes']['id'],
                              event_b[0]['object_attributes']['id']}, {100, 101})

    def test_old_schema_migration_preserves_events(self):
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / 'queue.sqlite3'
            with sqlite3.connect(path) as db:
                db.execute('''CREATE TABLE events (
                    id INTEGER PRIMARY KEY,
                    event_key TEXT NOT NULL UNIQUE,
                    payload TEXT NOT NULL,
                    attempt INTEGER NOT NULL,
                    due REAL NOT NULL,
                    status TEXT NOT NULL
                )''')
                for event_id, status in enumerate(('queued', 'processing', 'uncertain'), 1):
                    event = payload(id=99 + event_id)
                    db.execute('''INSERT INTO events(event_key, payload, attempt, due, status)
                                  VALUES (?, ?, 0, 0, ?)''',
                               (f'{PROJECT}:{99 + event_id}', json.dumps(event), status))

            jobs = notify.DurableQueue(path)
            with sqlite3.connect(path) as db:
                rows = db.execute('SELECT status, claimed_at FROM events ORDER BY id').fetchall()
                columns = {row[1] for row in db.execute('PRAGMA table_info(events)')}
            self.assertIn('claimed_at', columns)
            self.assertEqual([row[0] for row in rows], ['queued', 'uncertain', 'uncertain'])
            self.assertIsNone(rows[1][1])
            event, _, _ = jobs.get()
            self.assertEqual(event['object_attributes']['id'], 100)

    def test_uncertain_records_count_toward_total_storage_limit(self):
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / 'queue.sqlite3'
            jobs = notify.DurableQueue(path, maxsize=10, max_records=1)
            jobs.put_nowait((payload(), 0))
            with sqlite3.connect(path) as db:
                db.execute("UPDATE events SET status='uncertain'")
            with self.assertRaises(queue.Full):
                jobs.put_nowait((payload(id=101), 0))

    def test_worker_persists_retry_and_completes_event(self):
        with TemporaryDirectory() as tmp:
            completed = threading.Event()

            class TrackingQueue(notify.DurableQueue):
                def get(self):
                    if completed.is_set():
                        return None
                    return super().get()

                def complete(self, event_id):
                    super().complete(event_id)
                    completed.set()

            class FlakyNotifier:
                calls = 0

                def handle(self, event):
                    self.calls += 1
                    if self.calls == 1:
                        raise ValueError('temporary failure')

            jobs = TrackingQueue(Path(tmp) / 'queue.sqlite3')
            jobs.put_nowait((payload(), 0))
            notifier = FlakyNotifier()
            thread_errors = []
            original_hook = threading.excepthook
            threading.excepthook = lambda args: thread_errors.append(args.exc_value)
            self.addCleanup(setattr, threading, 'excepthook', original_hook)
            thread = threading.Thread(target=notify.worker, args=(notifier, jobs, (0,)), daemon=True)
            thread.start()
            self.assertTrue(completed.wait(5))
            self.assertEqual(notifier.calls, 2)
            thread.join(2)
            self.assertFalse(thread.is_alive())
            self.assertEqual(thread_errors, [])


class WorkerTests(Quiet):
    def test_failures_are_retried_with_backoff_then_given_up(self):
        delays = []

        class ImmediateTimer:
            def __init__(self, delay, function, args):
                delays.append(delay)
                self.call = lambda: function(*args)
                self.daemon = False

            def start(self):
                self.call()

        class Flaky:
            def __init__(self, failures):
                self.failures = failures
                self.handled = []

            def handle(self, event):
                self.handled.append(event['object_attributes']['id'])
                if len(self.handled) <= self.failures:
                    raise ValueError('cannot reach WeCom: URLError')
                return True

        for failures, expected_calls in [(2, 3), (5, 4)]:
            with self.subTest(failures=failures):
                delays.clear()
                jobs = queue.Queue()
                notifier = Flaky(failures)
                threading.Thread(target=notify.worker, args=(notifier, jobs, (1, 2, 3), ImmediateTimer),
                                 daemon=True).start()
                jobs.put((payload(), 0))
                jobs.join()
                self.assertEqual(len(notifier.handled), expected_calls)
                self.assertEqual(delays, [1, 2, 3][:expected_calls - 1])
        self.assertIn('giving up', self.log.getvalue())
        self.assertIn('--pipeline 100', self.log.getvalue())

    def test_worker_survives_unexpected_errors(self):
        class Broken:
            calls = 0

            def handle(self, event):
                Broken.calls += 1
                raise KeyError('project')

        jobs = queue.Queue()
        threading.Thread(target=notify.worker, args=(Broken(), jobs, ()), daemon=True).start()
        jobs.put(({}, 0))
        jobs.put(({}, 0))
        jobs.join()
        self.assertEqual(Broken.calls, 2)


class WeComTests(unittest.TestCase):
    class Opener:
        def __init__(self, *responses):
            self.responses = list(responses)
            self.requests = []

        def open(self, request, timeout):
            self.requests.append(request)
            response = self.responses.pop(0)
            if isinstance(response, Exception):
                raise response
            return io.BytesIO(response if isinstance(response, bytes) else json.dumps(response).encode())

    def test_send_markdown(self):
        opener = self.Opener({'errcode': 0, 'errmsg': 'ok'})
        notify.send_wecom('k/ey', '## hi', opener)
        request = opener.requests[0]
        self.assertEqual(request.full_url,
                         'https://qyapi.weixin.qq.com/cgi-bin/webhook/send?key=k%2Fey')
        self.assertEqual(json.loads(request.data),
                         {'msgtype': 'markdown', 'markdown': {'content': '## hi'}})

    def test_rejection_is_not_retried_and_does_not_leak_key(self):
        opener = self.Opener({'errcode': 93000, 'errmsg': 'invalid webhook url'})
        with self.assertRaises(ValueError) as caught:
            notify.send_wecom('secret-key', '## hi', opener)
        self.assertIn('93000', str(caught.exception))
        self.assertNotIn('secret-key', str(caught.exception))
        self.assertEqual(len(opener.requests), 1)

    def test_network_errors_are_left_to_the_worker(self):
        url = 'https://qyapi.weixin.qq.com/cgi-bin/webhook/send?key=secret-key'
        for error, name in [(URLError(url), 'URLError'), (HTTPError(url, 502, url, {}, None), 'HTTPError'),
                            (TimeoutError(), 'TimeoutError')]:
            with self.subTest(error=name):
                opener = self.Opener(error, {'errcode': 0})
                with self.assertRaises(ValueError) as caught:
                    notify.send_wecom('secret-key', '## hi', opener)
                self.assertEqual(str(caught.exception), f'WeCom delivery is uncertain: {name}')
                self.assertEqual(len(opener.requests), 1)


class LocalServer:
    """A tiny HTTP server whose handler is a function of the request."""

    def __init__(self, test, respond, host='127.0.0.1'):
        self.requests = []
        server = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                server.requests.append((self.path, self.headers))
                code, headers, body = respond(self.path)
                self.send_response(code)
                for name, value in headers.items():
                    self.send_header(name, value)
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *args):
                pass

        self.httpd = HTTPServer((host, 0), Handler)
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        test.addCleanup(self.httpd.server_close)
        test.addCleanup(self.httpd.shutdown)
        self.port = self.httpd.server_address[1]


class NoProxy:
    """Keep local test servers off any proxy configured in the environment."""

    def setUp(self):
        super().setUp()
        env = patch.dict(os.environ, {'no_proxy': '*', 'NO_PROXY': '*'})
        env.start()
        self.addCleanup(env.stop)


class GitLabClientTests(NoProxy, unittest.TestCase):
    @patch.object(notify.time, 'sleep')
    def test_retry_errors_and_pagination(self, sleep):
        hits = []

        def respond(path):
            hits.append(path)
            if len(hits) == 1:
                return 502, {}, b''
            if 'denied' in path:
                return 401, {}, b'{"message":"401 Unauthorized read-api-token"}'
            page = int(parse_qs(urlsplit(path).query)['page'][0])
            items = [{'id': i} for i in range(notify.PER_PAGE)] if page == 1 else [{'id': -1}]
            return 200, {'Content-Type': 'application/json'}, json.dumps(items).encode()

        server = LocalServer(self, respond)
        gitlab = notify.GitLab(f'http://127.0.0.1:{server.port}', 'read-api-token')
        items = gitlab.get_all(PROJECT, 'pipelines/1/jobs?scope[]=failed')
        self.assertEqual(len(items), notify.PER_PAGE + 1)
        self.assertEqual(server.requests[0][1].get('PRIVATE-TOKEN'), 'read-api-token')
        with self.assertRaises(ValueError) as caught:
            gitlab.get(PROJECT, 'denied')
        self.assertIn('HTTP 401', str(caught.exception))
        self.assertNotIn('read-api-token', str(caught.exception))

    def test_cross_host_redirect_drops_token(self):
        target = LocalServer(self, lambda path: (200, {}, b'log text'), host='127.0.0.1')
        # `localhost` and `127.0.0.1` are different hosts to the redirect handler.
        origin = LocalServer(self, lambda path: (
            302, {'Location': f'http://localhost:{target.port}/signed'}, b''))
        gitlab = notify.GitLab(f'http://127.0.0.1:{origin.port}', 'read-api-token')
        self.assertEqual(gitlab.trace(PROJECT, 5), 'log text')
        self.assertEqual(origin.requests[0][1].get('PRIVATE-TOKEN'), 'read-api-token')
        headers = {name.lower() for name in target.requests[0][1].keys()}
        self.assertNotIn('private-token', headers)

    def test_https_redirect_to_http_is_rejected_even_on_same_host(self):
        request = Request('https://gitlab.example/api/v4/projects/38/jobs/5/trace',
                          headers={'PRIVATE-TOKEN': 'read-api-token'})
        redirected = notify.SafeRedirect().redirect_request(
            request, None, 302, 'Found', {},
            'http://gitlab.example/api/v4/projects/38/jobs/5/trace')
        self.assertIsNone(redirected)

    def test_https_redirect_to_https_on_same_host_is_allowed(self):
        request = Request('https://gitlab.example/api/v4/projects/38/jobs/5/trace',
                          headers={'PRIVATE-TOKEN': 'read-api-token'})
        redirected = notify.SafeRedirect().redirect_request(
            request, None, 302, 'Found', {},
            'https://gitlab.example/api/v4/projects/38/jobs/5/trace?download=1')
        self.assertIsNotNone(redirected)
        self.assertEqual(redirected.get_header('Private-token'), 'read-api-token')


class WebhookServerTests(NoProxy, Quiet):
    def setUp(self):
        super().setUp()
        self.jobs = queue.Queue()
        self.server = notify.make_server('127.0.0.1', 0, 'hook-secret', self.jobs)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)
        self.url = f'http://127.0.0.1:{self.server.server_address[1]}'

    def post(self, path, body, token, headers=None):
        request = Request(self.url + path, data=body, method='POST',
                          headers=dict({'X-Gitlab-Token': token,
                                        'Content-Type': 'application/json'}, **(headers or {})))
        try:
            with urlopen(request, timeout=5) as response:
                return response.status
        except HTTPError as exc:
            return exc.code

    def test_token_path_and_body_are_checked(self):
        body = json.dumps(payload()).encode('utf-8')
        self.assertEqual(self.post('/gitlab', body, 'wrong'), 401)
        self.assertEqual(self.post('/other', body, 'hook-secret'), 404)
        self.assertEqual(self.post('/gitlab', b'not json', 'hook-secret'), 400)
        self.assertEqual(self.post('/gitlab', b'[1]', 'hook-secret'), 400)
        self.assertEqual(self.post('/gitlab', b'{}', 'hook-secret',
                                   {'Content-Length': str(notify.MAX_BODY_BYTES + 1)}), 413)
        self.assertTrue(self.jobs.empty())
        self.assertEqual(self.post('/gitlab', body, 'hook-secret'), 200)
        event, attempt = self.jobs.get_nowait()
        self.assertEqual((event['object_attributes']['id'], attempt), (100, 0))
        with urlopen(self.url + '/health', timeout=5) as response:
            self.assertEqual(response.read(), b'ok')

    def test_full_queue_returns_retryable_status(self):
        jobs = queue.Queue(maxsize=1)
        server = notify.make_server('127.0.0.1', 0, 'hook-secret', jobs)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        self.url = f'http://127.0.0.1:{server.server_address[1]}'
        self.assertEqual(self.post('/gitlab', json.dumps(payload()).encode('utf-8'),
                                   'hook-secret'), 200)
        with patch.object(jobs, 'put', side_effect=queue.Full):
            self.assertEqual(self.post('/gitlab', json.dumps(payload(id=101)).encode('utf-8'),
                                       'hook-secret'), 503)

    def test_queue_persistence_error_returns_retryable_status(self):
        with patch.object(self.jobs, 'put_nowait', side_effect=sqlite3.OperationalError('disk full')):
            self.assertEqual(self.post('/gitlab', json.dumps(payload()).encode('utf-8'),
                                       'hook-secret'), 503)

    def test_durable_event_is_committed_before_success_response(self):
        with TemporaryDirectory() as tmp:
            committed = threading.Event()
            release = threading.Event()

            class BarrierQueue(notify.DurableQueue):
                def put_nowait(self, item):
                    super().put_nowait(item)
                    committed.set()
                    release.wait(5)

            jobs = BarrierQueue(Path(tmp) / 'queue.sqlite3')
            server = notify.make_server('127.0.0.1', 0, 'hook-secret', jobs)
            threading.Thread(target=server.serve_forever, daemon=True).start()
            self.addCleanup(server.server_close)
            self.addCleanup(server.shutdown)
            self.addCleanup(release.set)
            self.url = f'http://127.0.0.1:{server.server_address[1]}'
            response = []
            completed = threading.Event()

            def send():
                response.append(self.post('/gitlab', json.dumps(payload()).encode('utf-8'),
                                          'hook-secret'))
                completed.set()

            thread = threading.Thread(target=send, daemon=True)
            thread.start()
            self.assertTrue(committed.wait(3))
            self.assertFalse(completed.wait(0.1))
            release.set()
            self.assertTrue(completed.wait(3))
            thread.join(1)
            self.assertEqual(response, [200])
            with sqlite3.connect(jobs.path) as db:
                row = db.execute('SELECT payload, status FROM events').fetchone()
            self.assertEqual(json.loads(row[0])['object_attributes']['id'], 100)
            self.assertEqual(row[1], 'queued')

    def test_idle_connection_does_not_block_other_requests(self):
        idle = socket.create_connection(self.server.server_address)
        self.addCleanup(idle.close)
        with urlopen(self.url + '/health', timeout=3) as response:
            self.assertEqual(response.read(), b'ok')

    def test_idle_connection_is_closed_after_timeout(self):
        with patch.object(notify, 'REQUEST_TIMEOUT', 0.3):
            server = notify.make_server('127.0.0.1', 0, 'hook-secret', queue.Queue())
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        idle = socket.create_connection(server.server_address)
        self.addCleanup(idle.close)
        idle.settimeout(5)
        started = time.monotonic()
        self.assertEqual(idle.recv(1), b'')  # the server hung up
        self.assertLess(time.monotonic() - started, 3)


if __name__ == '__main__':
    unittest.main()
