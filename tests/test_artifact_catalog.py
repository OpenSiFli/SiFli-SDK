# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

import copy
from contextlib import redirect_stdout
import io
import json
import os
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import Mock, patch
from urllib.request import Request

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
        })
        self.env.start()
        self.addCleanup(self.env.stop)
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
            'collect_batch_001': {'id': '789', 'status': 'success'}})
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
                         {'id': '720', 'status': 'failed'})
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
            self.assertNotIn('fetch(', page)
            self.assertEqual(json.loads((output / 'catalog.json').read_text()), result)
            self.assertTrue((output / 'README.md').exists())

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
