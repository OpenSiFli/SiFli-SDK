# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

import copy
from contextlib import redirect_stderr, redirect_stdout
from datetime import datetime, timedelta, timezone
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import Mock, patch
from urllib.request import Request
from urllib.parse import parse_qs, urlsplit

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools/ci'))
import artifact_catalog as catalog
import artifact_catalog_smoke as smoke
from collect_artifacts_simple import SimpleArtifactsCollector


class ArtifactCatalogTests(unittest.TestCase):
    def setUp(self):
        self.env = patch.dict(os.environ, {
            'CI_API_V4_URL': 'https://gitlab.example/api/v4', 'CI_PROJECT_ID': '38',
            'CI_PIPELINE_ID': '123', 'CI_JOB_ID': '999', 'CI_JOB_TOKEN': 'test-secret',
            'CI_COMMIT_SHA': 'abc123', 'CI_JOB_NAME': 'collect_batch_001',
            'GITLAB_API_READ_TOKEN': 'api-read-secret',
            'CI_DEFAULT_BRANCH': 'main', 'CI_COMMIT_REF_NAME': 'main',
            'CI_PIPELINE_CREATED_AT': '2026-11-04T00:00:00Z',
        })
        self.env.start()
        self.addCleanup(self.env.stop)
        self.now = datetime(2026, 11, 4, 6, tzinfo=timezone.utc)
        self.client = catalog.GitLab()
        self.pointer = {'pipeline_id': '123', 'project_id': '38', 'commit': 'abc123',
                        'batch': 'collect_batch_001', 'job_id': '789'}
        self.index = {'pipeline_id': '123', 'project_id': '38', 'merge_strategy': 'all',
                      'jobs': {'target': {'status': 'success',
                                         'logs': [{'path': 'target/ci_build_logs/target.log', 'size': 10}],
                                         'artifacts': [{'path': 'target/artifacts/target/sub/core #1.bin', 'size': 42}]}}}
        self.generator = Mock(batch_size=50)
        self.generator.get_build_jobs.return_value = [('build_target', 'example/uart/project', 'board_1', True)]

    def test_fetches_only_pipeline_pointer_and_exact_collector_index(self):
        self.client.json_request = Mock(side_effect=[self.pointer, self.index])
        self.assertEqual(self.client.load_index('collect_batch_001'), ('789', self.index))
        paths = [call.args[0] for call in self.client.json_request.call_args_list]
        self.assertEqual(paths, [f'packages/generic/{catalog.PACKAGE_NAME}/123/collect_batch_001.json',
                                 'jobs/789/artifacts/merged_artifacts/file_index.json'])

    def test_current_collector_job_list_ignores_retried_attempts(self):
        self.client.json_request = Mock(return_value=[
            {'id': 710, 'name': 'collect_batch_001', 'retried': True, 'status': 'failed'},
            {'id': 789, 'name': 'collect_batch_001', 'retried': False, 'status': 'success'},
            {'id': 900, 'name': 'build_batch_001', 'retried': False, 'status': 'success'},
        ])
        self.assertEqual(self.client.current_collector_jobs(), {
            'collect_batch_001': {'id': '789', 'status': 'success', 'started_at': None, 'finished_at': None}})
        self.assertEqual(self.client.json_request.call_args.args[0],
                         'pipelines/123/jobs?per_page=100&page=1')

    def test_collector_index_falls_back_to_read_token_for_permission_errors(self):
        for status in (401, 403, 404):
            with self.subTest(status=status):
                self.client.json_request = Mock(side_effect=[
                    self.pointer, ValueError(f'GitLab request failed (HTTP {status})'), self.index])
                self.assertEqual(self.client.load_index('collect_batch_001'), ('789', self.index))
                self.assertEqual(self.client.json_request.call_args.args,
                                 ('jobs/789/artifacts/merged_artifacts/file_index.json',))
                self.assertEqual(self.client.json_request.call_args.kwargs,
                                 {'api_token': 'api-read-secret'})

    def test_index_fallback_does_not_hide_missing_files_or_invalid_identity(self):
        self.client.json_request = Mock(side_effect=[
            self.pointer, ValueError('GitLab request failed (HTTP 404)'),
            ValueError('GitLab request failed (HTTP 404)')])
        with self.assertRaisesRegex(ValueError, 'HTTP 404'):
            self.client.load_index('collect_batch_001')
        self.assertEqual(self.client.json_request.call_count, 3)
        self.client.json_request = Mock(side_effect=[
            self.pointer, ValueError('GitLab request failed (HTTP 404)'),
            dict(self.index, pipeline_id='old')])
        with self.assertRaisesRegex(ValueError, 'another pipeline'):
            self.client.load_index('collect_batch_001')

    def test_index_api_does_not_fallback_for_connection_errors_or_without_token(self):
        self.client.json_request = Mock(side_effect=[
            self.pointer, ValueError('GitLab request timed out')])
        with self.assertRaisesRegex(ValueError, 'timed out'):
            self.client.load_index('collect_batch_001')
        self.assertEqual(self.client.json_request.call_count, 2)
        for token in ('', '$GITLAB_API_READ_TOKEN'):
            with self.subTest(token=token), patch.dict(os.environ, {'GITLAB_API_READ_TOKEN': token}):
                self.client.json_request = Mock(side_effect=[
                    self.pointer, ValueError('GitLab request failed (HTTP 404)')])
                with self.assertRaisesRegex(ValueError, 'configure GITLAB_API_READ_TOKEN'):
                    self.client.load_index('collect_batch_001')
                self.assertEqual(self.client.json_request.call_count, 2)

    def test_jobs_api_falls_back_to_read_token_for_permission_errors(self):
        for status in (401, 403, 404):
            with self.subTest(status=status):
                self.client.json_request = Mock(side_effect=[
                    ValueError(f'GitLab request failed (HTTP {status})'), []])
                self.assertEqual(self.client.current_collector_jobs(), {})
                self.assertEqual(self.client.json_request.call_args_list[1].kwargs,
                                 {'api_token': 'api-read-secret'})

    def test_jobs_api_does_not_fallback_for_other_errors_or_without_token(self):
        self.client.json_request = Mock(side_effect=ValueError('GitLab request timed out'))
        with self.assertRaisesRegex(ValueError, 'timed out'):
            self.client.current_collector_jobs()
        self.assertEqual(self.client.json_request.call_count, 1)

        with patch.dict(os.environ, {'GITLAB_API_READ_TOKEN': ''}):
            self.client.json_request = Mock(side_effect=ValueError('GitLab request failed (HTTP 403)'))
            with self.assertRaisesRegex(ValueError, 'configure GITLAB_API_READ_TOKEN'):
                self.client.current_collector_jobs()
            self.assertEqual(self.client.json_request.call_count, 1)

    def test_jobs_api_paginates_and_selects_latest_valid_exact_job(self):
        page_one = ([{'id': 710, 'name': 'artifact_catalog', 'retried': True, 'status': 'failed'},
                     {'id': 711, 'name': 'artifact_catalog_retry', 'retried': False, 'status': 'success'}]
                    + [{'id': 800 + i, 'name': f'unrelated_{i}', 'retried': False,
                        'status': 'success'} for i in range(98)])
        page_two = [{'id': 720, 'name': 'artifact_catalog', 'retried': False, 'status': 'failed'},
                    {'id': 719, 'name': 'artifact_catalog', 'retried': False, 'status': 'success'},
                    {'id': 'bad', 'name': 'artifact_catalog', 'retried': False, 'status': 'success'}]
        self.client.json_request = Mock(side_effect=[page_one, page_two])
        self.assertEqual(self.client.latest_job('123', 'artifact_catalog'),
                         {'id': '720', 'status': 'failed', 'started_at': None, 'finished_at': None})
        self.assertEqual([call.args[0] for call in self.client.json_request.call_args_list], [
            'pipelines/123/jobs?per_page=100&page=1',
            'pipelines/123/jobs?per_page=100&page=2',
        ])

    def test_stale_or_failed_collector_attempt_is_not_shown_as_current(self):
        self.client.load_index = Mock(return_value=('710', self.index))
        self.client.current_collector_jobs = Mock(return_value={
            'collect_batch_001': {'id': '789', 'status': 'failed'}})
        result = catalog.build_catalog(self.generator, self.client, 'https://gitlab.example/sw/sdk')
        self.assertEqual(result['rows'][0]['status'], 'unavailable')
        self.assertEqual(result['rows'][0]['files'], [])
        self.assertTrue(any('latest attempt' in issue for issue in result['issues']))

    def test_rejects_pointer_from_another_pipeline_commit_or_batch(self):
        for field, value in [('pipeline_id', '122'), ('commit', 'old'), ('project_id', '1'),
                             ('batch', 'collect_batch_002'), ('job_id', '../123')]:
            pointer = dict(self.pointer, **{field: value})
            self.client.json_request = Mock(return_value=pointer)
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.client.load_index('collect_batch_001')
            self.assertEqual(self.client.json_request.call_count, 1)

    def test_checks_file_index_identity_after_artifacts_upload(self):
        for field, value in [('pipeline_id', '122'), ('project_id', '1'), ('merge_strategy', 'failed_only')]:
            self.client.json_request = Mock(side_effect=[self.pointer, dict(self.index, **{field: value})])
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.client.load_index('collect_batch_001')

    def build(self, index=None):
        self.client.load_index = Mock(return_value=('789', index or self.index))
        self.client.current_collector_jobs = Mock(return_value={
            'collect_batch_001': {'id': '789', 'status': 'success'}})
        return catalog.build_catalog(self.generator, self.client, 'https://gitlab.example/sw/sdk')

    def test_downloads_point_to_exact_job_and_escape_subproject_file_paths(self):
        result = self.build()
        row = result['rows'][0]
        self.assertEqual(result['issues'], [])
        self.assertEqual(row['board'], 'board_1')
        self.assertEqual(row['files'][0]['name'], 'sub/core #1.bin')
        self.assertIn('/-/jobs/789/artifacts/', row['files'][0]['url'])
        self.assertTrue(row['files'][0]['url'].endswith('/target/artifacts/target/sub/core%20%231.bin?inline=false'))
        self.assertIn('/artifacts/raw/', row['files'][0]['url'])
        self.assertNotIn('test-secret', json.dumps(result))

    def test_failed_build_has_logs_but_never_offers_stale_firmware(self):
        index = copy.deepcopy(self.index)
        index['jobs']['target']['status'] = 'failed'
        row = self.build(index)['rows'][0]
        self.assertEqual(row['status'], 'failed')
        self.assertEqual(row['files'], [])
        self.assertEqual(len(row['logs']), 1)

    def test_missing_group_and_missing_record_keep_all_expected_targets_visible(self):
        self.client.load_index = Mock(side_effect=ValueError('GitLab request failed (HTTP 404)'))
        result = catalog.build_catalog(self.generator, self.client, 'https://gitlab.example/sw/sdk')
        self.assertEqual(len(result['rows']), 1)
        self.assertEqual(result['rows'][0]['status'], 'unavailable')
        self.assertTrue(result['issues'])
        result = self.build(dict(self.index, jobs={}))
        self.assertEqual(result['rows'][0]['files'], [])
        self.assertTrue(result['issues'])

    def test_does_not_accept_paths_from_another_target_or_traversal(self):
        for path in ['another/artifacts/another/core.bin', 'target/artifacts/target/../../core.bin']:
            index = copy.deepcopy(self.index)
            index['jobs']['target']['artifacts'][0]['path'] = path
            result = self.build(index)
            self.assertEqual(result['rows'][0]['status'], 'unavailable')
            self.assertTrue(result['issues'])

    def test_generated_html_embeds_data_safely_and_needs_no_network_for_search(self):
        result = self.build()
        result['rows'][0]['project'] = '</script><script>alert(1)</script>'
        with TemporaryDirectory() as temp:
            output = Path(temp)
            catalog.write_catalog(result, output)
            page = (output / 'index.html').read_text()
            self.assertNotIn('</script><script>alert(1)</script>', page)
            self.assertIn('\\u003c/script', page)
            self.assertIn('id="history-data">null</script>', page)
            self.assertEqual(json.loads((output / 'catalog.json').read_text()), result)
            self.assertTrue((output / 'README.md').exists())

    def test_placeholder_text_inside_catalog_data_is_not_substituted_again(self):
        result = self.build()
        result['rows'][0]['project'] = 'demo/__HISTORY_DATA__/project'
        history = {'days': 7, 'pipelines': [], 'default_pipeline_id': '123'}
        with TemporaryDirectory() as temp:
            output = Path(temp)
            catalog.write_catalog(result, output, history=history)
            page = (output / 'index.html').read_text()
            embedded = page.split('id="data">', 1)[1].split('</script>', 1)[0]
            self.assertEqual(json.loads(embedded)['rows'][0]['project'], 'demo/__HISTORY_DATA__/project')
            embedded_history = page.split('id="history-data">', 1)[1].split('</script>', 1)[0]
            self.assertEqual(json.loads(embedded_history), history)

    def test_generated_html_preserves_unavailable_explicit_ref(self):
        template = Path(catalog.__file__).with_name('artifact_catalog.html').read_text()
        self.assertIn("target?.ref || (params.get('ref') || 'main')", template)
        self.assertIn('if (params.has(\'ref\') && !refs.includes(requestedRef)) refs.push(requestedRef);', template)
        self.assertIn("byId('branch-filter').hidden = refs.length < 2;", template)

    def test_shared_catalog_without_ref_stays_on_main_when_only_release_is_fresh(self):
        template = Path(catalog.__file__).with_name('artifact_catalog.html').read_text()
        open_shared_build = template.split('function openSharedBuild() {', 1)[1].split('\n  }', 1)[0]
        self.assertIn("if (!refs.includes('main')) refs.unshift('main');", open_shared_build)
        self.assertIn("selectedBranch = target?.ref || (params.get('ref') || 'main');", open_shared_build)

    def test_invalid_shared_date_clears_old_selection_and_retries_current_url(self):
        template = Path(catalog.__file__).with_name('artifact_catalog.html').read_text()
        open_shared_build = template.split('function openSharedBuild() {', 1)[1].split('\n  }', 1)[0]
        self.assertLess(open_shared_build.index("selectedId = '';"), open_shared_build.index("if (params.has('date') && !dateEntry)"))
        self.assertIn("setMessage('链接中的日期没有可用构建；为避免展示错误版本，未切换到其他构建。', true, openSharedBuild);",
                      open_shared_build)
        self.assertIn('retry.addEventListener(\'click\', retryAction);', template)

    def test_empty_build_selection_clears_build_url_parameters_before_returning(self):
        template = Path(catalog.__file__).with_name('artifact_catalog.html').read_text()
        select_build = template.split('async function selectBuild(id, updateUrl = false) {', 1)[1].split(
            '\n  function openSharedBuild()', 1)[0]
        self.assertLess(select_build.index("url.searchParams.delete('date')"), select_build.index('if (!entry) {'))
        self.assertLess(select_build.index("url.searchParams.delete('pipeline')"), select_build.index('if (!entry) {'))
        self.assertIn("url.searchParams.set('ref', selectedBranch);", select_build)

    def test_open_shared_link_invalidates_inflight_build_requests(self):
        template = Path(catalog.__file__).with_name('artifact_catalog.html').read_text()
        open_shared_build = template.split('function openSharedBuild() {', 1)[1].split('\n  }', 1)[0]
        self.assertTrue(open_shared_build.lstrip().startswith('request++;'))
        self.assertLess(open_shared_build.index('request++;'), open_shared_build.index("if (params.has('pipeline') && !target)"))
        self.assertLess(open_shared_build.index('request++;'), open_shared_build.index("if (params.has('date') && !dateEntry)"))

    def test_generated_html_renders_without_history_entry(self):
        template = Path(catalog.__file__).with_name('artifact_catalog.html').read_text()
        self.assertIn('} else { restoreUrls(data); displayCatalog(data); }', template)

    def test_history_skips_pipelines_without_catalog_job_silently(self):
        recent = self.now.isoformat()
        self.client.read_json = Mock(side_effect=lambda path, resource: (
            [{'id': 122, 'source': 'push', 'sha': 'good', 'created_at': recent}]
            if path.startswith('pipelines?') and 'ref=main' in path else []))
        self.client.latest_job = Mock(return_value=None)

        class FrozenDateTime(datetime):
            @classmethod
            def now(cls, tz=None):
                return self.now if tz else self.now.replace(tzinfo=None)

        with patch.object(catalog, 'datetime', FrozenDateTime):
            with redirect_stderr(io.StringIO()) as warnings:
                self.assertEqual(catalog.load_history(self.client), [])
        self.assertEqual(warnings.getvalue(), '')

    def test_malformed_file_entries_do_not_prevent_partial_catalog_generation(self):
        for entries in [42, [None], [{'path': 42, 'size': 10}],
                        [{'path': 'target/artifacts/target/a.bin', 'size': -1}],
                        [{'path': 'target/artifacts/target/a.bin', 'size': '10'}]]:
            index = copy.deepcopy(self.index)
            index['jobs']['target']['artifacts'] = entries
            with self.subTest(entries=entries):
                result = self.build(index)
                self.assertEqual(result['rows'][0]['status'], 'unavailable')
                self.assertEqual(result['rows'][0]['files'], [])
                self.assertTrue(result['issues'])

    def test_publish_uploads_small_pointer_and_no_firmware(self):
        self.client.json_request = Mock()
        catalog.publish_pointer(self.client)
        path, pointer = self.client.json_request.call_args.args
        self.assertEqual(path, self.client.pointer_path('collect_batch_001'))
        self.assertLess(len(json.dumps(pointer)), 512)
        self.assertEqual(pointer['job_id'], '999')

    def test_parent_reuses_only_the_exact_child_catalog_json(self):
        result = self.build()
        self.client.json_request = Mock()
        with patch.dict(os.environ, {'SDK_PARENT_PIPELINE_ID': '122'}):
            catalog.publish_parent_pointer(self.client)
        path, pointer = self.client.json_request.call_args.args
        self.assertEqual(path, f'packages/generic/{catalog.PACKAGE_NAME}/122/artifact_catalog.json')
        self.assertLess(len(json.dumps(pointer)), 512)
        with patch.dict(os.environ, {'CI_PIPELINE_ID': '122'}):
            parent_client = catalog.GitLab()
        bridges = [{'name': 'trigger_child_pipeline', 'retried': False,
                    'downstream_pipeline': {'id': 123}}]
        catalog_jobs = [{'name': 'artifact_catalog', 'id': 999,
                         'status': 'success', 'retried': False}]
        parent_client.json_request = Mock(side_effect=[pointer, bridges, catalog_jobs, result])
        self.assertEqual(catalog.load_parent_catalog(parent_client), result)
        self.assertEqual(parent_client.json_request.call_args_list[1].kwargs, {})
        self.assertEqual(parent_client.json_request.call_args.args[0],
                         'jobs/999/artifacts/artifact_catalog/catalog.json')
        for field, value in [('parent_pipeline_id', '121'), ('commit', 'old'),
                             ('project_id', 'another'), ('kind', 'collector')]:
            parent_client.json_request = Mock(return_value=dict(pointer, **{field: value}))
            with self.subTest(field=field), self.assertRaises(ValueError):
                catalog.load_parent_catalog(parent_client)
            self.assertEqual(parent_client.json_request.call_count, 2)
        parent_client.json_request = Mock(side_effect=[
            pointer, [{'name': 'trigger_child_pipeline', 'retried': False,
                       'downstream_pipeline': {'id': 124}}]])
        with self.assertRaisesRegex(ValueError, 'current child pipeline'):
            catalog.load_parent_catalog(parent_client)
        self.assertEqual(parent_client.json_request.call_count, 2)
        parent_client.json_request = Mock(side_effect=[pointer, bridges, catalog_jobs,
                                                       dict(result, pipeline_id='old')])
        with self.assertRaises(ValueError):
            catalog.load_parent_catalog(parent_client)

        stale_jobs = [{'name': 'artifact_catalog', 'id': 1000, 'status': 'failed'}]
        parent_client.json_request = Mock(side_effect=[pointer, bridges, stale_jobs])
        with self.assertRaisesRegex(ValueError, 'latest child catalog attempt'):
            catalog.load_parent_catalog(parent_client)
        self.assertEqual(parent_client.json_request.call_count, 3)

        partial = dict(result, issues=['collect_batch_001: collector index missing'])
        parent_client.json_request = Mock(side_effect=[pointer, bridges,
                                                       [{'name': 'artifact_catalog', 'id': 999,
                                                         'status': 'failed'}], partial])
        self.assertEqual(catalog.load_parent_catalog(parent_client), partial)
        self.assertEqual(parent_client.json_request.call_args.args[0],
                         'jobs/999/artifacts/artifact_catalog/catalog.json')

        for latest_jobs, message in [
                ([{'name': 'artifact_catalog', 'id': 999, 'status': 'running'}], 'not finished: running'),
                ([{'name': 'artifact_catalog', 'id': 999, 'status': 'canceled'}], 'not finished: canceled'),
                ([], 'job is missing')]:
            parent_client.json_request = Mock(side_effect=[pointer, bridges, latest_jobs])
            with self.subTest(latest_jobs=latest_jobs), self.assertRaisesRegex(ValueError, message):
                catalog.load_parent_catalog(parent_client)
            self.assertEqual(parent_client.json_request.call_count, 3)

    def test_parent_entry_writes_partial_catalog_and_fails_when_it_has_issues(self):
        partial = {'pipeline_id': '123', 'project_id': '38', 'commit': 'abc123',
                   'rows': [], 'issues': ['collect_batch_001: collector index missing']}
        with TemporaryDirectory() as temp, patch.dict(os.environ, {
                'CI_PROJECT_URL': 'https://gitlab.example/sw/sdk', 'CI_JOB_ID': '555'}), \
                patch('sys.argv', ['artifact_catalog.py', 'entry', '--output', temp]), \
                patch.object(catalog, 'GitLab'), patch.object(catalog, 'load_parent_catalog',
                                                              return_value=partial), \
                redirect_stdout(io.StringIO()):
            self.assertEqual(catalog.main(), 1)
            self.assertTrue((Path(temp) / 'index.html').is_file())
            self.assertTrue((Path(temp) / 'catalog.json').is_file())

    def write_pages_source(self, root, issues=(), commit='abc123', index=None):
        source = Path(root) / 'artifact_catalog'
        source.mkdir()
        result = self.build(index)
        result.update(pipeline_id='124', commit=commit, issues=list(issues))
        result['rows'][0]['status'] = 'failed'
        catalog.write_catalog(result, source)
        return source

    CATALOG_JOB_PATH = 'pipelines/123/jobs?per_page=100&page=1'
    CATALOG_FINISHED = '2026-11-04T05:00:00Z'
    CATALOG_FINISHED_QUERY = '2026-11-04T05:00:00+00:00'
    CATALOG_PASSED = [{'id': 1230, 'name': 'artifact_catalog', 'status': 'success', 'retried': False,
                       'started_at': '2026-11-04T04:59:00Z', 'finished_at': CATALOG_FINISHED}]
    DEPLOY_CREATED = '2026-11-04T05:45:00Z'

    @staticmethod
    def status_path(pipeline_id, name):
        name = name.replace(':', '%3A')
        return (f'repository/commits/def456/statuses?pipeline_id={pipeline_id}'
                f'&name={name}&all=true&per_page=100&page=1')

    @staticmethod
    def candidate(pipeline_id, source='schedule', created_at='2026-11-04T04:00:00Z'):
        return {'id': int(pipeline_id), 'source': source, 'sha': 'def456', 'created_at': created_at}

    @staticmethod
    def pages_job(job_id, status, started_at, retried=False):
        return {'id': job_id, 'name': 'pages', 'status': status, 'started_at': started_at, 'retried': retried}

    @classmethod
    def deploy_status(cls, value):
        status, created_at = value if isinstance(value, tuple) else (value, cls.DEPLOY_CREATED)
        return {'name': 'pages:deploy', 'status': status, 'created_at': created_at}

    def frozen_clock(self):
        moment = self.now

        class FrozenDateTime(datetime):
            @classmethod
            def now(cls, tz=None):
                return moment if tz else moment.replace(tzinfo=None)
        return patch.object(catalog, 'datetime', FrozenDateTime)

    def fake_pages_api(self, catalog_jobs=CATALOG_PASSED, candidates=None, pages_jobs=None,
                       statuses=None, history=(), extra=None):
        """Serve the GitLab responses that publish_pages reads.

        catalog_jobs: jobs of pipeline 123, or {pipeline_id: jobs} for other pipelines.
        candidates: {branch: pipelines} returned for the skip check (updated_after == CATALOG_FINISHED).
        history: {branch: pipelines} (or one list for every branch) returned for load_history.
        pages_jobs: {pipeline_id: jobs} returned with include_retried=true.
        statuses: {(pipeline_id, 'pages:deploy'): values}; each value is a status or
                  (status, created_at), and a list of such lists answers successive polls.
        """
        if isinstance(catalog_jobs, list):
            catalog_jobs = {'123': catalog_jobs}
        candidates = candidates or {}
        pages_jobs = pages_jobs or {}
        responses = {f'pipelines/{pid}/jobs?per_page=100&page=1': jobs for pid, jobs in catalog_jobs.items()}
        for items in candidates.values():
            for item in items:
                responses.setdefault(f'pipelines/{item["id"]}/jobs?include_retried=true&per_page=100&page=1', [])
        for pid, jobs in pages_jobs.items():
            responses[f'pipelines/{pid}/jobs?include_retried=true&per_page=100&page=1'] = jobs
        responses.update(extra or {})
        polls = {}
        for (pipeline_id, _name), values in (statuses or {}).items():
            path = self.status_path(pipeline_id, 'pages:deploy')
            if values and isinstance(values[0], list):
                polls[path] = iter([[self.deploy_status(value) for value in poll] for poll in values])
            else:
                responses[path] = [self.deploy_status(value) for value in values]

        def respond(path, **_kwargs):
            if path.startswith('pipelines?') and 'updated_after=' in path:
                query = parse_qs(urlsplit(path).query)
                branch = query['ref'][0]
                if query['updated_after'][0] == self.CATALOG_FINISHED_QUERY:
                    items = candidates.get(branch, [])
                else:
                    if isinstance(history, Exception):
                        raise history
                    items = history.get(branch, []) if isinstance(history, dict) else history
                page = int(query['page'][0])
                return copy.deepcopy(list(items)[(page - 1) * 100:page * 100])
            if path not in polls and path not in responses:
                raise AssertionError(f'unexpected GitLab request: {path}')
            values = next(polls[path]) if path in polls else responses[path]
            if isinstance(values, Exception):
                raise values
            return copy.deepcopy(values)
        self.client.json_request = Mock(side_effect=respond)

    def requested_paths(self):
        return [call.args[0] for call in self.client.json_request.call_args_list]

    def test_pages_publishes_complete_catalog_with_failed_builds(self):
        with TemporaryDirectory() as temp, self.frozen_clock():
            source = self.write_pages_source(temp)
            output = Path(temp) / 'public'
            (output / 'stale').mkdir(parents=True)
            self.fake_pages_api()
            self.assertIsNone(catalog.publish_pages(self.client, source, output))
            # The default build lives in the root catalog.json and index.html only.
            self.assertEqual(sorted(p.name for p in output.iterdir()),
                             ['README.md', 'catalog.json', 'history.json', 'index.html'])
            self.assertEqual(self.requested_paths()[0], self.CATALOG_JOB_PATH)
            self.assertTrue(any('ref=release%2Fv2.5' in path for path in self.requested_paths()))
            history = json.loads((output / 'history.json').read_text())
            self.assertEqual(history, [{'pipeline_id': '123', 'ref': 'main', 'commit': 'abc123',
                                       'created_at': '2026-11-04T00:00:00Z',
                                       'success_count': 0, 'failed_count': 1}])
            self.assertEqual(json.loads((output / 'catalog.json').read_text()),
                             catalog.compact_catalog(json.loads((source / 'catalog.json').read_text())))
            self.assertFalse((output / 'stale').exists())

    def test_load_history_queries_each_branch_filters_boundaries_and_validates_catalogs(self):
        recent = self.now.isoformat()
        at_boundary = (self.now - timedelta(days=catalog.HISTORY_DAYS)).isoformat()
        outside = (self.now - timedelta(days=catalog.HISTORY_DAYS, seconds=1)).isoformat()
        def valid_catalog(commit, status='success'):
            value = self.build()
            value.update(project_id='38', commit=commit, issues=[])
            value['rows'][0]['status'] = status
            return value

        pipelines = {
            'main': [
                {'id': 122, 'source': 'schedule', 'sha': 'good', 'created_at': recent},
                {'id': 121, 'source': 'parent_pipeline', 'sha': 'good', 'created_at': recent},
                {'id': 120, 'source': 'schedule', 'sha': 'good', 'created_at': at_boundary},
                {'id': 119, 'source': 'schedule', 'sha': 'good', 'created_at': outside},
                {'id': 118, 'source': 'schedule', 'sha': 'smoke', 'created_at': recent},
                {'id': 117, 'source': 'schedule', 'sha': 'issues', 'created_at': recent},
                {'id': 116, 'source': 'schedule', 'sha': 'wrong', 'created_at': recent},
                {'id': 124, 'source': 'schedule', 'sha': 'newer', 'created_at': recent},
            ],
            'release/v2.5': [
                {'id': 125, 'source': 'web', 'sha': 'release', 'created_at': recent},
            ],
        }
        jobs = {str(i): {'id': str(i * 10), 'status': 'success'}
                for i in (122, 120, 118, 117, 116, 124, 125)}
        catalogs = {
            '122': valid_catalog('good'), '120': valid_catalog('good', 'failed'),
            '118': valid_catalog('smoke'), '117': valid_catalog('issues'),
            '116': valid_catalog('another'), '124': valid_catalog('newer'),
            '125': valid_catalog('release'),
        }
        catalogs['118']['smoke_test'] = True
        catalogs['117']['issues'] = ['bad']
        self.client.read_json = Mock(side_effect=lambda path, resource: (
            pipelines[parse_qs(urlsplit(path).query)['ref'][0]] if path.startswith('pipelines?')
            else catalogs[next(key for key, job in jobs.items() if path.startswith(f'jobs/{job["id"]}/'))]))
        self.client.latest_job = Mock(side_effect=lambda pipeline_id, name: jobs.get(pipeline_id))
        class FrozenDateTime(datetime):
            @classmethod
            def now(cls, tz=None):
                return self.now if tz else self.now.replace(tzinfo=None)

        with patch.object(catalog, 'datetime', FrozenDateTime):
            with redirect_stderr(io.StringIO()) as warnings:
                history = catalog.load_history(self.client)
        self.assertEqual([(item['pipeline_id'], item['ref']) for item in history],
                         [('122', 'main'), ('120', 'main'), ('124', 'main'), ('125', 'release/v2.5')])
        self.assertEqual(warnings.getvalue().count('Warning:'), 3)
        listing_refs = [parse_qs(urlsplit(call.args[0]).query).get('ref', [''])[0]
                        for call in self.client.read_json.call_args_list
                        if call.args[0].startswith('pipelines?')]
        self.assertEqual(listing_refs, ['main', 'release/v2.5'])

    def test_load_history_reads_candidate_catalogs_with_bounded_concurrency(self):
        recent = self.now.isoformat()
        pipelines = [{'id': 200 + index, 'source': 'schedule', 'sha': f'sha-{index}',
                      'created_at': recent} for index in range(8)]
        jobs = {str(item['id']): {'id': f"{item['id']}0", 'status': 'success'} for item in pipelines}
        catalogs = {}
        for item in pipelines:
            value = self.build()
            value.update(project_id='38', commit=item['sha'], issues=[])
            catalogs[jobs[str(item['id'])]['id']] = value

        active = 0
        max_active = 0
        lock = threading.Lock()

        def latest_job(pipeline_id, _name):
            nonlocal active, max_active
            with lock:
                active += 1
                max_active = max(max_active, active)
            time.sleep(0.02)
            with lock:
                active -= 1
            return jobs.get(pipeline_id)

        def read_json(path, _resource):
            if path.startswith('pipelines?'):
                ref = parse_qs(urlsplit(path).query)['ref'][0]
                return pipelines if ref == 'main' else []
            job_id = path.split('/')[1]
            return catalogs[job_id]

        self.client.latest_job = latest_job
        self.client.read_json = read_json

        class FrozenDateTime(datetime):
            @classmethod
            def now(cls, tz=None):
                return self.now if tz else self.now.replace(tzinfo=None)

        with patch.object(catalog, 'datetime', FrozenDateTime):
            history = catalog.load_history(self.client)
        self.assertEqual(len(history), len(pipelines))
        self.assertGreater(max_active, 1)
        self.assertLessEqual(max_active, catalog.HISTORY_WORKERS)

    def test_pipeline_list_failure_is_fatal_to_history(self):
        self.client.read_json = Mock(side_effect=ValueError('pipeline listing denied'))
        class FrozenDateTime(datetime):
            @classmethod
            def now(cls, tz=None):
                return self.now if tz else self.now.replace(tzinfo=None)

        with patch.object(catalog, 'datetime', FrozenDateTime):
            with self.assertRaisesRegex(ValueError, 'pipeline listing denied'):
                catalog.load_history(self.client)

    def test_smoke_pages_publish_stays_in_single_pipeline_mode_without_real_history(self):
        with TemporaryDirectory() as temp:
            source = self.write_pages_source(temp)
            current = json.loads((source / 'catalog.json').read_text())
            current['smoke_test'] = True
            catalog.write_catalog(current, source)
            self.fake_pages_api()
            output = Path(temp) / 'public'
            catalog.publish_pages(self.client, source, output)
            page = (output / 'index.html').read_text()
            self.assertIn('id="history-data">null</script>', page)
            self.assertIn('"smoke_test":true', page)
            self.assertEqual(json.loads((output / 'history.json').read_text()), [])

    def test_smoke_pages_does_not_replace_main_when_only_release_history_exists(self):
        with TemporaryDirectory() as temp, patch.dict(os.environ, {
                'CI_PIPELINE_CREATED_AT': '2026-11-04T00:00:00Z'}):
            source = self.write_pages_source(temp)
            current = json.loads((source / 'catalog.json').read_text())
            current['smoke_test'] = True
            catalog.write_catalog(current, source)
            release_catalog = dict(current, smoke_test=False, project_id='38',
                                   commit='release-sha', pipeline_id='1250', issues=[])
            release_pipeline = {
                'id': 125, 'source': 'web', 'sha': 'release-sha',
                'created_at': '2026-11-19T00:00:00Z',
            }
            self.client.latest_job = Mock(return_value={'id': '1250', 'status': 'success'})
            self.fake_pages_api(
                history={'release/v2.5': [release_pipeline]},
                extra={'jobs/1250/artifacts/artifact_catalog/catalog.json': release_catalog},
            )

            class FrozenDateTime(datetime):
                @classmethod
                def now(cls, tz=None):
                    value = datetime(2026, 11, 20, tzinfo=timezone.utc)
                    return value if tz else value.replace(tzinfo=None)

            output = Path(temp) / 'public'
            with patch.object(catalog, 'datetime', FrozenDateTime):
                with self.assertRaisesRegex(ValueError, 'no unexpired complete catalog for main'):
                    catalog.publish_pages(self.client, source, output)
            self.assertFalse(output.exists())

    def test_pages_keeps_main_as_default_when_release_build_is_newer(self):
        with TemporaryDirectory() as temp:
            source = self.write_pages_source(temp)
            current = json.loads((source / 'catalog.json').read_text())
            current['smoke_test'] = True
            catalog.write_catalog(current, source)
            main_time = '2026-11-03T00:00:00Z'
            release_time = '2026-11-04T00:00:00Z'
            main_catalog = json.loads((source / 'catalog.json').read_text())
            main_catalog['smoke_test'] = False
            main_catalog.update(commit='main-sha', pipeline_id='9991')
            release_catalog = dict(main_catalog, commit='release-sha', pipeline_id='9992')
            history = {
                'main': [{'id': 120, 'source': 'schedule', 'sha': 'main-sha', 'created_at': main_time}],
                'release/v2.5': [{'id': 125, 'source': 'web', 'sha': 'release-sha', 'created_at': release_time}],
            }
            self.client.latest_job = Mock(side_effect=lambda pipeline_id, _name: {
                'id': '1200' if pipeline_id == '120' else '1250', 'status': 'success'})
            self.client.read_json = Mock(side_effect=lambda path, _resource: (
                history[parse_qs(urlsplit(path).query)['ref'][0]] if path.startswith('pipelines?')
                else main_catalog if path.startswith('jobs/1200/') else release_catalog))
            output = Path(temp) / 'public'
            # The skip check has its own tests; this one covers default selection only.
            with patch.object(catalog, 'newer_published_pipeline', return_value=None), self.frozen_clock():
                catalog.publish_pages(self.client, source, output)
            embedded = json.loads((output / 'catalog.json').read_text())
            history_data = json.loads((output / 'index.html').read_text().split(
                'id="history-data">', 1)[1].split('</script>', 1)[0])
            self.assertEqual(embedded['commit'], 'main-sha')
            self.assertEqual(history_data['default_pipeline_id'], '120')

    def test_pages_does_not_use_release_as_default_when_main_history_is_expired(self):
        with TemporaryDirectory() as temp, patch.dict(os.environ, {
                'CI_PIPELINE_CREATED_AT': '2026-11-04T00:00:00Z'}):
            source = self.write_pages_source(temp)
            release_catalog = json.loads((source / 'catalog.json').read_text())
            release_catalog.update(project_id='38', commit='release-sha', pipeline_id='1250', issues=[])
            release_pipeline = {
                'id': 125, 'source': 'web', 'sha': 'release-sha',
                'created_at': '2026-11-19T00:00:00Z',
            }
            self.client.latest_job = Mock(return_value={'id': '1250', 'status': 'success'})
            self.fake_pages_api(
                history={'release/v2.5': [release_pipeline]},
                extra={'jobs/1250/artifacts/artifact_catalog/catalog.json': release_catalog},
            )

            class FrozenDateTime(datetime):
                @classmethod
                def now(cls, tz=None):
                    value = datetime(2026, 11, 20, tzinfo=timezone.utc)
                    return value if tz else value.replace(tzinfo=None)

            output = Path(temp) / 'public'
            with patch.object(catalog, 'datetime', FrozenDateTime):
                with self.assertRaisesRegex(ValueError, 'no unexpired complete catalog for main'):
                    catalog.publish_pages(self.client, source, output)
            self.assertFalse(output.exists())
        documentation = Path(catalog.__file__).with_name('ARTIFACT_CATALOG.md').read_text()
        self.assertIn('不会用 `release/v2.5` 替代', documentation)
        self.assertIn('Pages 发布也会失败并保留原入口', documentation)

    def test_history_entry_rejects_missing_render_fields(self):
        pipeline = {'id': '122', 'ref': 'main', 'sha': 'abc123',
                    'created_at': '2026-11-04T00:00:00Z'}
        for field in ('generated_at', 'pipeline_url'):
            broken = self.build()
            broken.pop(field)
            with self.subTest(field=field), self.assertRaises(ValueError):
                catalog.history_entry(pipeline, broken, '38')
        for field in ('board', 'project', 'files', 'logs'):
            broken = self.build()
            broken['rows'][0].pop(field)
            with self.subTest(field=field), self.assertRaises(ValueError):
                catalog.history_entry(pipeline, broken, '38')
        broken = self.build()
        broken['generated_at'] = 'not-a-date'
        with self.assertRaises(ValueError):
            catalog.history_entry(pipeline, broken, '38')

    def test_pages_does_not_publish_an_expired_current_catalog_without_history(self):
        with TemporaryDirectory() as temp:
            source = self.write_pages_source(temp)
            self.fake_pages_api()
            class FrozenDateTime(datetime):
                @classmethod
                def now(cls, tz=None):
                    value = datetime(2026, 11, 20, tzinfo=timezone.utc)
                    return value if tz else value.replace(tzinfo=None)
            output = Path(temp) / 'public'
            with patch.object(catalog, 'datetime', FrozenDateTime):
                with self.assertRaisesRegex(ValueError, 'no unexpired complete catalog for main'):
                    catalog.publish_pages(self.client, source, output)
            self.assertFalse(output.exists())

    def test_pages_never_publishes_files_left_by_unsuccessful_catalog_job(self):
        # Pages runs `always`; a failed catalog job may still have uploaded a complete-looking page.
        for jobs in [[{'id': 1229, 'name': 'artifact_catalog', 'status': 'success', 'retried': True},
                      {'id': 1230, 'name': 'artifact_catalog', 'status': 'failed', 'retried': False}],
                     [{'id': 1230, 'name': 'artifact_catalog', 'status': 'canceled', 'retried': False}],
                     []]:
            with self.subTest(jobs=jobs), TemporaryDirectory() as temp, \
                    patch.dict(os.environ, {'CI_DEFAULT_BRANCH': 'main'}):
                source = self.write_pages_source(temp)
                self.fake_pages_api(catalog_jobs=jobs)
                with self.assertRaisesRegex(ValueError, 'artifact_catalog job is'):
                    catalog.publish_pages(self.client, source, Path(temp) / 'public')
                self.assertFalse((Path(temp) / 'public').exists())
                self.assertEqual(self.requested_paths(), [self.CATALOG_JOB_PATH])

    def test_pages_keeps_published_entry_for_incomplete_or_foreign_catalog(self):
        for kwargs in [{'issues': ['collect_batch_001: collector index missing']}, {'commit': 'old'}]:
            with self.subTest(**kwargs), TemporaryDirectory() as temp, \
                    patch.dict(os.environ, {'CI_DEFAULT_BRANCH': 'main'}):
                source = self.write_pages_source(temp, **kwargs)
                self.fake_pages_api()
                with self.assertRaises(ValueError):
                    catalog.publish_pages(self.client, source, Path(temp) / 'public')
                self.assertFalse((Path(temp) / 'public').exists())
                self.assertEqual(self.requested_paths(), [self.CATALOG_JOB_PATH])

    TRICKY_NAMES = ['sub/core #1.bin', 'a b/中文 名.bin', "x!'()*.bin", 'a#b?c%d&e=f+g.bin', '100%.bin',
                    'emoji 😀.bin']

    def tricky_index(self):
        index = copy.deepcopy(self.index)
        target = index['jobs']['target']
        target['artifacts'] = [{'path': f'target/artifacts/target/{name}', 'size': 42} for name in self.TRICKY_NAMES]
        target['logs'] = [{'path': f'target/ci_build_logs/{name}', 'size': 7} for name in self.TRICKY_NAMES]
        return index

    def test_compact_catalog_drops_only_urls_that_rebuild_exactly(self):
        result = self.build(self.tricky_index())
        row = result['rows'][0]
        self.assertEqual(len(row['files']), len(self.TRICKY_NAMES))
        before = copy.deepcopy(result)
        row['files'][1]['url'] = 'https://elsewhere.example/odd.bin'
        row['logs'][0]['url'] += '&changed=1'
        expected = copy.deepcopy(result)
        compact = catalog.compact_catalog(result)
        self.assertEqual(result, expected)  # the input is never modified
        self.assertEqual(compact['rows'][0]['files'][1], {'name': row['files'][1]['name'],
                                                          'size': 42,
                                                          'url': 'https://elsewhere.example/odd.bin'})
        self.assertEqual(compact['rows'][0]['logs'][0]['url'], expected['rows'][0]['logs'][0]['url'])
        for category in ('files', 'logs'):
            for index, entry in enumerate(compact['rows'][0][category]):
                if (category, index) in {('files', 1), ('logs', 0)}:
                    continue
                with self.subTest(category=category, name=entry['name']):
                    self.assertNotIn('url', entry)
                    full = before['rows'][0][category][index]
                    self.assertEqual(catalog.derived_file_url(row, category, entry['name']), full['url'])
        self.assertEqual(before['rows'][0]['files'][0]['name'], self.TRICKY_NAMES[0])

    def test_pages_output_drops_only_urls_that_rebuild_exactly(self):
        with TemporaryDirectory() as temp, self.frozen_clock():
            source = self.write_pages_source(temp, index=self.tricky_index())
            full = json.loads((source / 'catalog.json').read_text())
            # A URL that differs from the rebuilt one must survive in every Pages copy.
            full['rows'][0]['files'][1]['url'] = 'https://elsewhere.example/odd.bin'
            (source / 'catalog.json').write_text(json.dumps(full), encoding='utf-8')
            self.fake_pages_api()
            output = Path(temp) / 'public'
            self.assertIsNone(catalog.publish_pages(self.client, source, output))
            compact = json.loads((output / 'catalog.json').read_text())
            page = (output / 'index.html').read_text()
            self.assertEqual(json.loads(page.split('id="data">', 1)[1].split('</script>', 1)[0]), compact)
            self.assertEqual(compact, catalog.compact_catalog(full))
            for category in ('files', 'logs'):
                for index, entry in enumerate(compact['rows'][0][category]):
                    full_entry = full['rows'][0][category][index]
                    with self.subTest(category=category, name=entry['name']):
                        if (category, index) == ('files', 1):
                            self.assertEqual(entry['url'], 'https://elsewhere.example/odd.bin')
                        else:
                            self.assertNotIn('url', entry)
                            self.assertEqual(catalog.derived_file_url(compact['rows'][0], category, entry['name']),
                                             full_entry['url'])
            # The job artifact keeps the full form that load_history() validates.
            self.assertEqual(json.loads((source / 'catalog.json').read_text()), full)
            self.assertTrue(all('url' in entry for entry in full['rows'][0]['files'] + full['rows'][0]['logs']))

    def test_pages_history_writes_non_default_builds_only_and_compact(self):
        release = self.build(self.tricky_index())
        release.update(project_id='38', commit='def456', pipeline_id='1250', issues=[])
        with TemporaryDirectory() as temp, self.frozen_clock():
            source = self.write_pages_source(temp)
            self.fake_pages_api(
                catalog_jobs={'123': self.CATALOG_PASSED, '125': [
                    {'id': 1250, 'name': 'artifact_catalog', 'status': 'success', 'retried': False,
                     'started_at': '2026-11-04T03:59:00Z', 'finished_at': '2026-11-04T04:00:00Z'}]},
                history={'release/v2.5': [self.candidate(125, created_at='2026-11-04T03:00:00Z')]},
                extra={'jobs/1250/artifacts/artifact_catalog/catalog.json': release})
            output = Path(temp) / 'public'
            self.assertIsNone(catalog.publish_pages(self.client, source, output))
            self.assertEqual(sorted(p.name for p in (output / 'pipelines').iterdir()), ['125'])
            stored = json.loads((output / 'pipelines/125/catalog.json').read_text())
            self.assertEqual(stored, catalog.compact_catalog(release))
            self.assertNotIn('url', stored['rows'][0]['files'][0])
            page = (output / 'index.html').read_text()
            history_data = json.loads(page.split('id="history-data">', 1)[1].split('</script>', 1)[0])
            self.assertEqual(history_data['default_pipeline_id'], '123')

    @unittest.skipUnless(shutil.which('node'), 'node is not installed')
    def test_js_rebuilds_urls_byte_for_byte_like_python(self):
        result = self.build(self.tricky_index())
        full = copy.deepcopy(result)
        compact = catalog.compact_catalog(result)
        script = self.js_url_helpers() + (
            "\nconst input = JSON.parse(require('fs').readFileSync(0, 'utf8'));"
            "\nrestoreUrls(input);"
            "\nprocess.stdout.write(JSON.stringify(input));")
        completed = subprocess.run(['node', '-e', script], input=json.dumps(compact), capture_output=True,
                                   text=True, encoding='utf-8', check=True)
        restored = json.loads(completed.stdout)
        self.assertEqual(restored['rows'][0]['files'], full['rows'][0]['files'])
        self.assertEqual(restored['rows'][0]['logs'], full['rows'][0]['logs'])

    @staticmethod
    def js_url_helpers():
        page = Path(catalog.__file__).with_name('artifact_catalog.html').read_text(encoding='utf-8')
        return page.split('// BEGIN url helpers', 1)[1].split('\n', 1)[1].split('// END url helpers', 1)[0]

    def test_page_fetches_only_non_default_builds(self):
        template = Path(catalog.__file__).with_name('artifact_catalog.html').read_text(encoding='utf-8')
        # Every eviction goes through forget(), which never drops the embedded default build.
        self.assertEqual(template.count('cache.delete('), 1)
        self.assertIn('const forget = id => { if (id !== embeddedId) cache.delete(id); };', template)
        self.assertIn('forget(id);', template)
        self.assertIn('const embeddedId = historyData ? historyData.default_pipeline_id : null;', template)
        self.assertIn('if (embeddedId) cache.set(embeddedId, data);', template)
        self.assertIn('} else { restoreUrls(data); displayCatalog(data); }', template)

    def test_pages_skips_when_attempt_started_after_catalog_and_deployed(self):
        with TemporaryDirectory() as temp, self.frozen_clock(), \
                patch.object(catalog.time, 'sleep') as sleep:
            source = self.write_pages_source(temp)
            # An attempt starting exactly when the catalog finished counts.
            for started_at in ('2026-11-04T05:00:00Z', '2026-11-04T05:30:00Z'):
                with self.subTest(started_at=started_at):
                    self.fake_pages_api(
                        candidates={'main': [self.candidate(130)]},
                        pages_jobs={'130': [self.pages_job(1301, 'success', started_at)]},
                        statuses={('130', 'pages:deploy'): [('success', '2026-11-04T05:40:00Z')]})
                    output = Path(temp) / 'public'
                    self.assertEqual(catalog.publish_pages(self.client, source, output), '130')
                    self.assertFalse(output.exists())
            sleep.assert_not_called()

    def test_pages_does_not_skip_for_attempt_started_before_catalog_finished(self):
        with TemporaryDirectory() as temp, self.frozen_clock():
            source = self.write_pages_source(temp)
            self.fake_pages_api(
                candidates={'main': [self.candidate(130)]},
                pages_jobs={'130': [self.pages_job(1301, 'success', '2026-11-04T04:50:00Z')]},
                statuses={('130', 'pages:deploy'): [('success', '2026-11-04T05:10:00Z')]})
            output = Path(temp) / 'public'
            self.assertIsNone(catalog.publish_pages(self.client, source, output))
            self.assertTrue((output / 'index.html').is_file())
            self.assertFalse(any('statuses' in path for path in self.requested_paths()))

    def test_pages_does_not_skip_when_attempts_after_catalog_did_not_deploy(self):
        cases = [
            ([self.pages_job(1301, 'success', '2026-11-04T05:30:00Z')], [('failed', '2026-11-04T05:40:00Z')]),
            ([self.pages_job(1301, 'failed', '2026-11-04T05:30:00Z')], []),
        ]
        for jobs, deploys in cases:
            with self.subTest(jobs=jobs, deploys=deploys), TemporaryDirectory() as temp, \
                    self.frozen_clock(), patch.object(catalog.time, 'sleep'):
                source = self.write_pages_source(temp)
                self.fake_pages_api(candidates={'main': [self.candidate(130)]},
                                    pages_jobs={'130': jobs}, statuses={('130', 'pages:deploy'): deploys})
                output = Path(temp) / 'public'
                self.assertIsNone(catalog.publish_pages(self.client, source, output))
                self.assertTrue((output / 'index.html').is_file())

    def test_pages_skip_is_not_used_when_several_attempts_succeeded(self):
        # Two successful attempts produce two pages:deploy statuses, and their order cannot be matched
        # to the attempts. The check must not skip (publishing again is harmless) and must not wait.
        with TemporaryDirectory() as temp, self.frozen_clock(), patch.object(catalog.time, 'sleep') as sleep:
            source = self.write_pages_source(temp)
            self.fake_pages_api(
                candidates={'main': [self.candidate(130)]},
                pages_jobs={'130': [self.pages_job(1300, 'success', '2026-11-04T05:10:00Z'),
                                    self.pages_job(1301, 'success', '2026-11-04T05:30:00Z')]},
                statuses={('130', 'pages:deploy'): [[('success', '2026-11-04T05:20:00Z')],
                                                    [('success', '2026-11-04T05:40:00Z')]]})
            output = Path(temp) / 'public'
            self.assertIsNone(catalog.publish_pages(self.client, source, output))
            self.assertTrue((output / 'index.html').is_file())
            sleep.assert_not_called()

    def test_pages_skip_follows_the_only_successful_attempt_result(self):
        # The earlier attempt failed. The only successful one started after the catalog, but its
        # deployment failed, so the catalog must be published rather than skipped.
        with TemporaryDirectory() as temp, self.frozen_clock(), patch.object(catalog.time, 'sleep') as sleep:
            source = self.write_pages_source(temp)
            self.fake_pages_api(
                candidates={'main': [self.candidate(130)]},
                pages_jobs={'130': [self.pages_job(1300, 'failed', '2026-11-04T05:10:00Z'),
                                    self.pages_job(1301, 'success', '2026-11-04T05:30:00Z')]},
                statuses={('130', 'pages:deploy'): [[('failed', '2026-11-04T05:40:00Z')]]})
            output = Path(temp) / 'public'
            self.assertIsNone(catalog.publish_pages(self.client, source, output))
            self.assertTrue((output / 'index.html').is_file())
            sleep.assert_not_called()

    def test_pages_skip_does_not_query_deployments_without_a_successful_attempt(self):
        with TemporaryDirectory() as temp, self.frozen_clock():
            source = self.write_pages_source(temp)
            self.fake_pages_api(
                candidates={'main': [self.candidate(130)]},
                pages_jobs={'130': [self.pages_job(1301, 'failed', '2026-11-04T05:30:00Z')]},
                statuses={('130', 'pages:deploy'): [('success', '2026-11-04T05:40:00Z')]})
            output = Path(temp) / 'public'
            self.assertIsNone(catalog.publish_pages(self.client, source, output))
            self.assertTrue((output / 'index.html').is_file())
            self.assertFalse(any('statuses' in path for path in self.requested_paths()))

    def test_pages_skip_counts_retried_attempts_and_requests_them(self):
        with TemporaryDirectory() as temp, self.frozen_clock():
            source = self.write_pages_source(temp)
            self.fake_pages_api(
                candidates={'main': [self.candidate(130)]},
                pages_jobs={'130': [self.pages_job(1301, 'failed', '2026-11-04T05:10:00Z', retried=True),
                                    self.pages_job(1302, 'success', '2026-11-04T05:20:00Z')]},
                statuses={('130', 'pages:deploy'): [('success', '2026-11-04T05:30:00Z')]})
            self.assertEqual(catalog.publish_pages(self.client, source, Path(temp) / 'public'), '130')
            self.assertIn('pipelines/130/jobs?include_retried=true&per_page=100&page=1', self.requested_paths())

    def test_pages_skip_waits_for_pending_deployment_then_follows_its_result(self):
        for result, expected in [('success', '130'), ('failed', None)]:
            with self.subTest(result=result), TemporaryDirectory() as temp, self.frozen_clock(), \
                    patch.object(catalog.time, 'sleep') as sleep:
                source = self.write_pages_source(temp)
                self.fake_pages_api(
                    candidates={'main': [self.candidate(130)]},
                    pages_jobs={'130': [self.pages_job(1301, 'success', '2026-11-04T05:30:00Z')]},
                    statuses={('130', 'pages:deploy'): [[], ['running'], [(result, '2026-11-04T05:40:00Z')]]})
                output = Path(temp) / 'public'
                self.assertEqual(catalog.publish_pages(self.client, source, output), expected)
                self.assertEqual(output.exists(), expected is None)
                self.assertEqual(sleep.call_count, 2)

    def test_pages_skip_does_not_wait_forever_for_pending_deployment(self):
        statuses = {('130', 'pages:deploy'): ['running']}
        clock = iter(range(0, catalog.PAGES_DEPLOY_TIMEOUT * 2, catalog.PAGES_DEPLOY_POLL))

        def tick():
            now = next(clock, None)
            if now is None:
                raise AssertionError('waited past the deadline')
            return now
        with TemporaryDirectory() as temp, self.frozen_clock(), \
                patch.object(catalog.time, 'sleep') as sleep, \
                patch.object(catalog.time, 'monotonic', side_effect=tick):
            source = self.write_pages_source(temp)
            self.fake_pages_api(candidates={'main': [self.candidate(130)]},
                                pages_jobs={'130': [self.pages_job(1301, 'success', '2026-11-04T05:30:00Z')]},
                                statuses=statuses)
            with self.assertRaisesRegex(ValueError, 'still unfinished'):
                catalog.publish_pages(self.client, source, Path(temp) / 'public')
            self.assertFalse((Path(temp) / 'public').exists())
            self.assertEqual(sleep.call_count, catalog.PAGES_DEPLOY_TIMEOUT // catalog.PAGES_DEPLOY_POLL - 1)

    def test_pages_skip_ignores_self_and_child_pipelines(self):
        with TemporaryDirectory() as temp, self.frozen_clock():
            source = self.write_pages_source(temp)
            self.fake_pages_api(
                candidates={'main': [self.candidate(123), self.candidate(128, source='parent_pipeline')]},
                pages_jobs={'123': [self.pages_job(1231, 'success', '2026-11-04T05:30:00Z')],
                            '128': [self.pages_job(1281, 'success', '2026-11-04T05:30:00Z')]},
                statuses={('123', 'pages:deploy'): [('success', '2026-11-04T05:40:00Z')],
                          ('128', 'pages:deploy'): [('success', '2026-11-04T05:40:00Z')]})
            output = Path(temp) / 'public'
            self.assertIsNone(catalog.publish_pages(self.client, source, output))
            self.assertTrue((output / 'index.html').is_file())
            paths = self.requested_paths()
            self.assertFalse(any(path.startswith('pipelines/123/jobs?include_retried') for path in paths))
            self.assertFalse(any('pipeline_id=128' in path for path in paths))

    def main_history(self):
        """Arguments for main #130 as the default build: complete, and created before the catalog."""
        main_catalog = self.build()
        main_catalog.update(project_id='38', commit='def456', pipeline_id='1300', issues=[])
        return {
            'catalog_jobs': {'123': self.CATALOG_PASSED, '130': [
                {'id': 1300, 'name': 'artifact_catalog', 'status': 'success', 'retried': False,
                 'started_at': '2026-11-04T03:59:00Z', 'finished_at': '2026-11-04T04:01:00Z'}]},
            'history': {'main': [self.candidate(130)]},
            'extra': {'jobs/1300/artifacts/artifact_catalog/catalog.json': main_catalog},
        }

    def test_release_catalog_is_not_skipped_by_earlier_main_deployment_with_larger_id(self):
        # Release #123 finishes after main #130 deployed. #130 started before the release catalog,
        # so its deployment cannot include it, whatever the ID order.
        with TemporaryDirectory() as temp, self.frozen_clock(), \
                patch.dict(os.environ, {'CI_COMMIT_REF_NAME': 'release/v2.5'}):
            source = self.write_pages_source(temp)
            self.fake_pages_api(
                candidates={'main': [self.candidate(130)]},
                pages_jobs={'130': [self.pages_job(1301, 'success', '2026-11-04T04:00:00Z')]},
                statuses={('130', 'pages:deploy'): [('success', '2026-11-04T04:10:00Z')]},
                **self.main_history())
            output = Path(temp) / 'public'
            self.assertIsNone(catalog.publish_pages(self.client, source, output))
            history_data = json.loads((output / 'index.html').read_text().split(
                'id="history-data">', 1)[1].split('</script>', 1)[0])
            self.assertEqual(history_data['default_pipeline_id'], '130')
            self.assertEqual(sorted(entry['pipeline_id'] for entry in history_data['pipelines']), ['123', '130'])

    def test_pages_skip_only_applies_to_catalogs_that_enter_history(self):
        # A smoke catalog and a pipeline older than the retention window are not in history,
        # so a later deployment does not include them.
        cases = [('smoke', {}), ('expired', {'CI_PIPELINE_CREATED_AT': '2026-10-01T00:00:00Z'})]
        for name, env in cases:
            with self.subTest(name=name), TemporaryDirectory() as temp, self.frozen_clock(), \
                    patch.dict(os.environ, env):
                source = self.write_pages_source(temp)
                if name == 'smoke':
                    current = json.loads((source / 'catalog.json').read_text())
                    current['smoke_test'] = True
                    catalog.write_catalog(current, source)
                self.fake_pages_api(
                    candidates={'main': [self.candidate(130)]},
                    pages_jobs={'130': [self.pages_job(1301, 'success', '2026-11-04T05:30:00Z')]},
                    statuses={('130', 'pages:deploy'): [('success', '2026-11-04T05:40:00Z')]},
                    **self.main_history())
                output = Path(temp) / 'public'
                self.assertIsNone(catalog.publish_pages(self.client, source, output))
                self.assertFalse(any('statuses' in path for path in self.requested_paths()))

    def test_pages_rejects_ref_outside_history_refs_before_any_api_call(self):
        with TemporaryDirectory() as temp, self.frozen_clock(), \
                patch.dict(os.environ, {'CI_COMMIT_REF_NAME': 'feature/pages'}):
            source = self.write_pages_source(temp)
            self.client.json_request = Mock(side_effect=AssertionError('no API call expected'))
            with self.assertRaisesRegex(ValueError, 'not a catalog branch'):
                catalog.publish_pages(self.client, source, Path(temp) / 'public')
            self.client.json_request.assert_not_called()
            self.assertFalse((Path(temp) / 'public').exists())

    def test_smoke_catalog_is_not_published_from_release_branch(self):
        with TemporaryDirectory() as temp, self.frozen_clock(), patch.dict(os.environ, {
                'CI_COMMIT_REF_NAME': 'release/v2.5', 'CI_DEFAULT_BRANCH': 'main'}):
            source = self.write_pages_source(temp)
            current = json.loads((source / 'catalog.json').read_text())
            current['smoke_test'] = True
            catalog.write_catalog(current, source)
            self.fake_pages_api()
            output = Path(temp) / 'public'
            with self.assertRaisesRegex(ValueError, 'only from the default branch'):
                catalog.publish_pages(self.client, source, output)
            self.assertFalse(output.exists())

    def test_pages_skip_rejects_malformed_api_values(self):
        catalog_job = {'id': '1230', 'status': 'success'}
        self.client.json_request = Mock(side_effect=AssertionError('no API call expected'))
        with self.assertRaisesRegex(ValueError, 'finish time is missing'):
            catalog.newer_published_pipeline(self.client, catalog_job)
        with self.assertRaisesRegex(ValueError, 'no timezone'):
            catalog.newer_published_pipeline(self.client, dict(catalog_job, finished_at='2026-11-04T05:00:00'))
        bad_jobs = [
            [{'name': 'pages', 'status': 'success', 'started_at': 'not-a-time'}],
            [{'name': 'pages', 'status': 42, 'started_at': self.CATALOG_FINISHED}],
            ['not-a-job'],
        ]
        for jobs in bad_jobs:
            self.client.json_request = Mock(return_value=jobs)
            with self.subTest(jobs=jobs), self.assertRaises(ValueError):
                catalog.pages_attempts(self.client, '130')
        for statuses, message in [([{'created_at': None, 'status': 'success'}], 'creation time is missing'),
                                  (['not-a-status'], 'invalid commit status')]:
            self.client.json_request = Mock(return_value=statuses)
            with self.subTest(statuses=statuses), self.assertRaisesRegex(ValueError, message):
                catalog.commit_statuses(self.client, 'def456', '130', 'pages:deploy')

    def test_pages_skip_uses_allowed_failure_exit_code(self):
        with patch('sys.argv', ['artifact_catalog.py', 'pages']), patch.object(catalog, 'GitLab'), \
                patch.object(catalog, 'publish_pages', return_value='126'), redirect_stdout(io.StringIO()):
            self.assertEqual(catalog.main(), catalog.PAGES_SKIPPED)
        with patch('sys.argv', ['artifact_catalog.py', 'pages']), patch.object(catalog, 'GitLab'), \
                patch.object(catalog, 'publish_pages', side_effect=ValueError('catalog has 1 issues')), \
                patch('sys.stderr', io.StringIO()):
            self.assertEqual(catalog.main(), 1)

    def test_parent_catalog_file_falls_back_to_read_token_for_permission_errors(self):
        with patch.dict(os.environ, {'CI_PIPELINE_ID': '122'}):
            parent_client = catalog.GitLab()
        pointer = {'kind': 'catalog', 'project_id': '38', 'parent_pipeline_id': '122',
                   'commit': 'abc123', 'job_id': '999', 'pipeline_id': '123'}
        bridges = [{'name': 'trigger_child_pipeline', 'retried': False,
                    'downstream_pipeline': {'id': 123}}]
        jobs = [{'name': 'artifact_catalog', 'id': 999, 'status': 'success'}]
        result = {'pipeline_id': '123', 'project_id': '38', 'commit': 'abc123',
                  'rows': [], 'issues': []}
        for status in (401, 403, 404):
            with self.subTest(status=status):
                parent_client.json_request = Mock(side_effect=[
                    pointer, bridges, jobs,
                    ValueError(f'GitLab request failed (HTTP {status})'), result])
                self.assertEqual(catalog.load_parent_catalog(parent_client), result)
                self.assertEqual(parent_client.json_request.call_args.args,
                                 ('jobs/999/artifacts/artifact_catalog/catalog.json',))
                self.assertEqual(parent_client.json_request.call_args.kwargs,
                                 {'api_token': 'api-read-secret'})

    def test_parent_catalog_rejects_missing_or_ambiguous_current_trigger(self):
        with patch.dict(os.environ, {'CI_PIPELINE_ID': '122'}):
            parent_client = catalog.GitLab()
        pointer = {'kind': 'catalog', 'project_id': '38', 'parent_pipeline_id': '122',
                   'commit': 'abc123', 'job_id': '999', 'pipeline_id': '123'}
        for bridges in ([], [
                {'name': 'trigger_child_pipeline', 'retried': False,
                 'downstream_pipeline': {'id': 123}},
                {'name': 'trigger_child_pipeline', 'retried': False,
                 'downstream_pipeline': {'id': 124}}]):
            parent_client.json_request = Mock(side_effect=[pointer, bridges])
            with self.subTest(bridges=bridges), self.assertRaisesRegex(ValueError, 'missing or ambiguous'):
                catalog.load_parent_catalog(parent_client)
            self.assertEqual(parent_client.json_request.call_count, 2)

    def test_smoke_fixtures_follow_real_collection_and_catalog_paths(self):
        generator = catalog.DynamicPipelineGenerator(smoke_test=True)
        with TemporaryDirectory() as temp:
            root = Path(temp)
            exits = [smoke.write_fixture(name[len('build_'):], root)
                     for name, *_ in generator.get_build_jobs()]
            self.assertEqual(exits, [0, 0, 1])
            original_cwd = Path.cwd()
            try:
                os.chdir(root)
                with redirect_stdout(io.StringIO()):
                    SimpleArtifactsCollector().collect_artifacts()
                    SimpleArtifactsCollector('failed_merged_artifacts', failed_only=True).collect_artifacts()
            finally:
                os.chdir(original_cwd)
            index = json.loads((root / catalog.INDEX_PATH).read_text())
            failed_index = json.loads((root / 'failed_merged_artifacts/file_index.json').read_text())
            self.assertEqual(len(failed_index['jobs']), 1)
            self.client.load_index = Mock(return_value=('789', index))
            self.client.current_collector_jobs = Mock(return_value={
                'collect_batch_001': {'id': '789', 'status': 'success'}})
            result = catalog.build_catalog(generator, self.client, 'https://gitlab.example/sw/sdk')
            self.assertEqual(result['issues'], [])
            self.assertTrue(result['smoke_test'])
            self.assertEqual([row['status'] for row in result['rows']], ['success', 'success', 'failed'])
            for row in result['rows']:
                for item in row['files']:
                    self.assertEqual(item['name'], 'SMOKE_TEST_ONLY.txt')
                    self.assertLess(item['size'], 512)
                    self.assertIn('jobs/789/artifacts/raw/', item['url'])
                self.assertEqual(len(row['logs']), 1)
            catalog.write_catalog(result, root / 'catalog')
            self.assertIn('快速验证数据', (root / 'catalog/README.md').read_text())

    def test_smoke_catalog_fails_when_expected_output_or_logs_are_missing(self):
        generator = catalog.DynamicPipelineGenerator(smoke_test=True)
        index_jobs = {}
        for name, project, _board, _common in generator.get_build_jobs():
            label = name[len('build_'):]
            failed = 'expected-failure' in project
            index_jobs[label] = {
                'status': 'failed' if failed else 'success',
                'logs': [] if label.endswith('smoke_board_b') else [
                    {'path': f'{label}/ci_build_logs/{label}.log', 'size': 20}],
                'artifacts': [] if failed else ([] if label.endswith('smoke_board_b') else [
                    {'path': f'{label}/artifacts/{label}/SMOKE_TEST_ONLY.txt', 'size': 40}]),
            }
        self.client.load_index = Mock(return_value=('789', {
            'pipeline_id': '123', 'project_id': '38', 'merge_strategy': 'all',
            'jobs': index_jobs,
        }))
        self.client.current_collector_jobs = Mock(return_value={
            'collect_batch_001': {'id': '789', 'status': 'success'}})
        result = catalog.build_catalog(generator, self.client, 'https://gitlab.example/sw/sdk')
        self.assertTrue(any('output missing or invalid' in issue for issue in result['issues']))
        self.assertTrue(any('log missing or duplicated' in issue for issue in result['issues']))

    def test_redirect_does_not_send_job_token_to_object_storage(self):
        request = Request('https://gitlab.example/file', headers={'JOB-TOKEN': 'test-secret'})
        redirect = catalog.SafeRedirect().redirect_request(request, None, 302, '', {}, 'https://storage.example/signed')
        self.assertIsNone(redirect.get_header('Job-token'))

    def test_redirect_does_not_send_api_token_to_object_storage(self):
        request = Request('https://gitlab.example/file', headers={'PRIVATE-TOKEN': 'api-read-secret'})
        redirect = catalog.SafeRedirect().redirect_request(request, None, 302, '', {},
                                                           'https://storage.example/signed')
        self.assertIsNone(redirect.get_header('Private-token'))

    def test_parent_entry_requires_read_api_token(self):
        with patch.dict(os.environ, {'CI_PIPELINE_ID': '122'}):
            parent_client = catalog.GitLab()
        with patch.dict(os.environ, {'GITLAB_API_READ_TOKEN': ''}):
            parent_client.json_request = Mock(side_effect=[{}, ValueError('GitLab request failed (HTTP 403)')])
            with self.assertRaisesRegex(ValueError, 'configure GITLAB_API_READ_TOKEN'):
                catalog.load_parent_catalog(parent_client)

    def test_parent_entry_falls_back_to_read_api_token_when_job_token_is_forbidden(self):
        with patch.dict(os.environ, {'CI_PIPELINE_ID': '122'}):
            parent_client = catalog.GitLab()
        pointer = {'kind': 'catalog', 'project_id': '38', 'parent_pipeline_id': '122',
                   'commit': 'abc123', 'job_id': '999', 'pipeline_id': '123'}
        bridges = [{'name': 'trigger_child_pipeline', 'retried': False,
                    'downstream_pipeline': {'id': 123}}]
        result = {'pipeline_id': '123', 'project_id': '38', 'commit': 'abc123',
                  'rows': [], 'issues': []}
        parent_client.json_request = Mock(side_effect=[pointer,
                                                       ValueError('GitLab request failed (HTTP 403)'),
                                                       bridges,
                                                       [{'name': 'artifact_catalog', 'id': 999,
                                                         'status': 'success', 'retried': False}], result])
        catalog.load_parent_catalog(parent_client)
        self.assertEqual(parent_client.json_request.call_args_list[2].kwargs,
                         {'api_token': 'api-read-secret'})

    def test_parent_entry_falls_back_to_read_api_token_on_hidden_permission_404(self):
        with patch.dict(os.environ, {'CI_PIPELINE_ID': '122'}):
            parent_client = catalog.GitLab()
        pointer = {'kind': 'catalog', 'project_id': '38', 'parent_pipeline_id': '122',
                   'commit': 'abc123', 'job_id': '999', 'pipeline_id': '123'}
        bridges = [{'name': 'trigger_child_pipeline', 'retried': False,
                    'downstream_pipeline': {'id': 123}}]
        result = {'pipeline_id': '123', 'project_id': '38', 'commit': 'abc123',
                  'rows': [], 'issues': []}
        parent_client.json_request = Mock(side_effect=[pointer,
                                                       ValueError('GitLab request failed (HTTP 404)'),
                                                       bridges,
                                                       [{'name': 'artifact_catalog', 'id': 999,
                                                         'status': 'success', 'retried': False}], result])
        catalog.load_parent_catalog(parent_client)
        self.assertEqual(parent_client.json_request.call_args_list[2].kwargs,
                         {'api_token': 'api-read-secret'})

    def test_legacy_board_and_core_can_be_selected_by_name(self):
        self.assertEqual(catalog.board_label('example/ble/project/eh-lb555/hcpu', None), 'eh-lb555_hcpu')
        self.assertEqual(catalog.board_label('example/ble/project/eh-lb555/lcpu', None), 'eh-lb555_lcpu')
        self.assertEqual(catalog.board_label('example/ble/project', None), '工程内置配置')

    def test_five_thousand_targets_use_one_small_index_request_per_group(self):
        jobs = [(f'build_target_{i}', f'example/{i}', 'board_1', True) for i in range(5000)]
        self.generator.get_build_jobs.return_value = jobs

        def load(batch):
            offset = (int(batch.rsplit('_', 1)[1]) - 1) * 50
            return str(1000 + offset), {'jobs': {
                f'target_{i}': {'status': 'failed', 'logs': [], 'artifacts': []}
                for i in range(offset, offset + 50)
            }}

        self.client.load_index = Mock(side_effect=load)
        self.client.current_collector_jobs = Mock(return_value={
            f'collect_batch_{i + 1:03d}': {'id': str(1000 + i * 50), 'status': 'success'}
            for i in range(100)})
        result = catalog.build_catalog(self.generator, self.client, 'https://gitlab.example/sw/sdk')
        self.assertEqual(len(result['rows']), 5000)
        self.assertEqual(result['issues'], [])
        self.assertEqual(self.client.load_index.call_count, 100)
        self.assertEqual(len({row['label'] for row in result['rows']}), 5000)


if __name__ == '__main__':
    unittest.main()
