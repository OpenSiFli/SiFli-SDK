# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

import importlib.util
import os
import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
GENERATOR_PATH = REPO_ROOT / 'tools/ci/dynamic_pipeline.py'
RESOLVER_PATH = REPO_ROOT / 'tools/ci/resolve_build_job.py'
JUNIT_PATH = REPO_ROOT / 'tools/ci/write_build_junit.py'

spec = importlib.util.spec_from_file_location('dynamic_pipeline', GENERATOR_PATH)
dynamic_pipeline = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dynamic_pipeline)
sys_path = str(GENERATOR_PATH.parent)
if sys_path not in sys.path:
    sys.path.insert(0, sys_path)
resolver_spec = importlib.util.spec_from_file_location('resolve_build_job', RESOLVER_PATH)
resolver = importlib.util.module_from_spec(resolver_spec)
resolver_spec.loader.exec_module(resolver)
junit_spec = importlib.util.spec_from_file_location('write_build_junit', JUNIT_PATH)
junit = importlib.util.module_from_spec(junit_spec)
junit_spec.loader.exec_module(junit)


class DynamicPipelineTests(unittest.TestCase):
    def test_generated_config_keeps_parallel_batches_in_one_child_pipeline(self):
        config_path = REPO_ROOT / 'build_config_gcc.yaml'
        generator = dynamic_pipeline.DynamicPipelineGenerator(config_path)
        pipeline = generator.generate_child_pipeline()
        batch_jobs = [name for name in pipeline if name.startswith('build_batch_')]
        build_jobs = generator.get_build_jobs()

        self.assertGreater(len(build_jobs), 0)
        self.assertEqual(
            len(batch_jobs),
            (len(build_jobs) + generator.batch_size - 1) // generator.batch_size,
        )
        self.assertEqual(pipeline['stages'], ['build', 'collect', 'catalog'])
        self.assertEqual(pipeline['artifact_catalog']['dependencies'], [])
        self.assertNotIn('needs', pipeline['artifact_catalog'])
        self.assertEqual(pipeline['include'], [{'local': '.gitlab/ci/build_sdk_batch.yml'}])
        emitted_labels = []
        for index, name in enumerate(batch_jobs):
            suffix = f'{index + 1:03d}'
            matrix = pipeline[name]['parallel']['matrix']
            self.assertEqual(pipeline[name]['stage'], 'build')
            self.assertEqual(pipeline[name]['extends'], '.build_sdk_batch_template')
            self.assertNotIn('trigger', pipeline[name])
            self.assertLessEqual(len(matrix), 50)
            self.assertEqual(len(pipeline[f'collect_batch_{suffix}']['needs']), 1)
            labels = [entry['BUILD_JOB_LABEL'] for entry in matrix]
            emitted_labels.extend(labels)
            self.assertLessEqual(max(len(f'{name}: [{label}]') for label in labels), 128)
            self.assertEqual(
                pipeline[f'collect_batch_{suffix}']['needs'],
                [{'job': name, 'artifacts': True}],
            )
            self.assertEqual(pipeline[f'collect_batch_{suffix}']['stage'], 'collect')
        self.assertTrue(all(
            pipeline[name]['variables']['BUILD_CONFIG'] == str(config_path)
            for name in batch_jobs
        ))
        self.assertEqual(
            emitted_labels,
            [job[0][len('build_'):] for job in build_jobs],
        )
        self.assertLess(len(yaml.safe_dump(pipeline).encode('utf-8')), 1_000_000)

    def test_matrix_labels_resolve_each_target_exactly_once(self):
        generator = dynamic_pipeline.DynamicPipelineGenerator(REPO_ROOT / 'build_config_gcc.yaml')
        pipeline = generator.generate_child_pipeline()
        jobs = generator.get_build_jobs()
        resolved = []
        for name, job in pipeline.items():
            if not name.startswith('collect_batch_'):
                continue
            for need in job['needs']:
                build = pipeline[need['job']]
                for entry in build['parallel']['matrix']:
                    target = resolver.resolve_build_job_by_label(
                        generator,
                        entry['BUILD_JOB_LABEL'],
                    )
                    self.assertIsNotNone(target, entry['BUILD_JOB_LABEL'])
                    resolved.append(target['LOG_NAME'])

        self.assertEqual(resolved, [job[0][len('build_'):] for job in jobs])
        self.assertEqual(len(set(resolved)), len(jobs))

    def test_unused_parallel_slots_are_skipped(self):
        config_path = REPO_ROOT / 'build_config_gcc.yaml'
        generator = dynamic_pipeline.DynamicPipelineGenerator(config_path)
        slot = len(generator.get_build_jobs()) % generator.batch_size
        if slot == 0:
            self.skipTest('configured build count exactly fills the final batch')

        result = resolver.resolve_build_job(
            generator,
            len(generator.get_build_jobs()) // generator.batch_size,
            slot + 1,
        )
        self.assertIsNone(result)

    def test_common_job_keeps_board_directory_and_scons_board(self):
        config_path = REPO_ROOT / 'build_config_gcc.yaml'
        generator = dynamic_pipeline.DynamicPipelineGenerator(config_path)
        jobs = generator.get_build_jobs()
        common_job_index = next(index for index, job in enumerate(jobs) if job[2])
        values = resolver.resolve_build_job(
            generator,
            common_job_index // generator.batch_size,
            common_job_index % generator.batch_size + 1,
        )

        self.assertEqual(values['PROJECT_TYPE'], 'common')
        self.assertTrue(values['BOARD'].endswith('_hcpu'))
        self.assertFalse(values['SCONS_BOARD'].endswith('_hcpu'))
        self.assertEqual(values['BUILD_ARGS'], f"--board={values['SCONS_BOARD']}")

    def test_batch_template_quotes_arguments_and_passes_config(self):
        template = (REPO_ROOT / '.gitlab/ci/build_sdk_batch.yml').read_text(encoding='utf-8')

        self.assertIn('--config "$BUILD_CONFIG"', template)
        self.assertIn('--job-label "$BUILD_JOB_LABEL"', template)
        self.assertIn('after_script:', template)
        self.assertIn('|| exit 1', template)
        self.assertIn('junit: job_status/*.junit.xml', template)
        self.assertNotIn('parallel', yaml.safe_load(template)['.build_sdk_batch_template'])
        self.assertIn('collect_artifacts_simple.py --failed-only', template)

    def test_collector_runs_all_collection_steps_after_any_failure(self):
        template = yaml.safe_load((REPO_ROOT / '.gitlab/ci/build_sdk_batch.yml').read_text())
        command = template['.collect_sdk_batch_template']['script'][0]
        for fail_at in (0, 1, 2, 3):
            with self.subTest(fail_at=fail_at):
                shell = f'''calls=0
python3() {{
  calls=$((calls + 1))
  printf 'CALL:%s:%s\\n' "$calls" "$*"
  [ "$calls" -ne "$FAIL_AT" ]
}}
{command}
'''
                result = subprocess.run(['bash', '-c', shell], env={**os.environ,
                                            'FAIL_AT': str(fail_at)},
                                        capture_output=True, text=True)
                self.assertEqual(sum(line.startswith('CALL:')
                                     for line in result.stdout.splitlines()), 3)
                self.assertEqual(result.returncode == 0, fail_at == 0)

    def test_catalog_scales_beyond_fifty_collectors_without_artifact_dependencies(self):
        from unittest.mock import patch

        generator = dynamic_pipeline.DynamicPipelineGenerator(REPO_ROOT / 'build_config_gcc.yaml')
        jobs = [(f'build_target_{i}', f'example/{i}', 'board', True) for i in range(5000)]
        with patch.object(generator, 'get_build_jobs', return_value=jobs):
            pipeline = generator.generate_child_pipeline()
        self.assertEqual(sum(name.startswith('collect_batch_') for name in pipeline), 100)
        self.assertEqual(pipeline['artifact_catalog']['dependencies'], [])
        self.assertNotIn('needs', pipeline['artifact_catalog'])
        template = yaml.safe_load((REPO_ROOT / '.gitlab/ci/build_sdk_batch.yml').read_text())
        self.assertEqual(template['.sdk_artifact_catalog_template']['when'], 'always')
        self.assertLess(len(yaml.safe_dump(pipeline).encode()), 1_000_000)

    def test_smoke_mode_only_generates_tiny_synthetic_targets(self):
        generator = dynamic_pipeline.DynamicPipelineGenerator(smoke_test=True)
        pipeline = generator.generate_child_pipeline()
        builds = [job for name, job in pipeline.items() if name.startswith('build_batch_')]
        self.assertEqual(len(generator.get_build_jobs()), 3)
        self.assertEqual(len(builds), 1)
        self.assertEqual(builds[0]['extends'], '.build_sdk_catalog_smoke_template')
        self.assertTrue(builds[0]['allow_failure'])
        self.assertEqual(pipeline['variables']['CATALOG_SMOKE_TEST'], '1')
        template = yaml.safe_load((REPO_ROOT / '.gitlab/ci/build_sdk_batch.yml').read_text())
        smoke = template['.build_sdk_catalog_smoke_template']
        self.assertEqual(smoke['after_script'], [])
        self.assertEqual(smoke['variables']['GIT_SUBMODULE_STRATEGY'], 'none')
        self.assertNotIn('build_job.sh', str(smoke['script']))
        self.assertNotIn('export.sh', str(smoke['script']))

    def test_parent_entry_waits_for_child_and_downloads_no_firmware(self):
        parent = yaml.safe_load((REPO_ROOT / '.gitlab/ci/build_sdk.yml').read_text())
        entry = parent['artifact_catalog']
        self.assertEqual(entry['needs'], [{'job': 'trigger_child_pipeline', 'artifacts': False}])
        self.assertEqual(entry['when'], 'always')
        self.assertEqual(entry['script'], ['python3 tools/ci/artifact_catalog.py entry'])
        self.assertEqual(parent['trigger_child_pipeline']['variables']['SDK_PARENT_PIPELINE_ID'],
                         '$CI_PIPELINE_ID')
        self.assertIs(parent['trigger_child_pipeline']['inherit']['variables'], False)
        self.assertEqual(parent['trigger_child_pipeline']['variables']['GITLAB_API_READ_TOKEN'],
                         '$GITLAB_API_READ_TOKEN')
        for name in ['generate_dynamic_pipeline', 'trigger_child_pipeline', 'artifact_catalog']:
            self.assertEqual(parent[name]['rules'][0]['if'],
                             '$CATALOG_SMOKE_TEST == "1" && $CI_PIPELINE_SOURCE =~ /^(web|api)$/')

    def test_pages_publishes_default_branch_catalog_only(self):
        pages = yaml.safe_load((REPO_ROOT / '.gitlab/ci/build_sdk.yml').read_text())['pages']
        self.assertEqual(pages['needs'], [{'job': 'artifact_catalog', 'artifacts': True}])
        self.assertEqual(pages['artifacts']['paths'], ['public/'])
        self.assertEqual(pages['script'], ['python3 tools/ci/artifact_catalog.py pages'])
        self.assertEqual(pages['resource_group'], 'sdk-artifact-catalog-pages')
        self.assertEqual(pages['allow_failure'], {'exit_codes': [3]})
        rules = [(rule['if'], rule['when']) for rule in pages['rules']]
        self.assertEqual(rules[0], ('$CI_COMMIT_BRANCH != $CI_DEFAULT_BRANCH', 'never'))
        # A smoke run must not replace the real catalog unless publishing is requested.
        self.assertIn('$CATALOG_PUBLISH == "1"', rules[1][0])
        self.assertEqual(rules[2], ('$CATALOG_SMOKE_TEST == "1"', 'never'))
        # Build failures fail the upstream trigger; the script itself checks artifact_catalog.
        self.assertEqual([when for _condition, when in rules if when != 'never'], ['always', 'always'])
        self.assertNotIn('merge_request_event', str(rules))

    def test_build_job_label_resolves_common_board_configuration(self):
        generator = dynamic_pipeline.DynamicPipelineGenerator(REPO_ROOT / 'build_config_gcc.yaml')
        common_job = next(job for job in generator.get_build_jobs() if job[2])
        label = common_job[0][len('build_'):]
        values = resolver.resolve_build_job_by_label(generator, label)

        self.assertIsNotNone(values)
        self.assertEqual(values['PROJECT_TYPE'], 'common')
        self.assertEqual(values['BOARD'], common_job[2])
        self.assertIsNone(resolver.resolve_build_job_by_label(generator, 'not_a_real_target'))

    def test_junit_report_records_failed_build_target(self):
        from tempfile import TemporaryDirectory

        with TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / 'nested' / 'build.junit.xml'
            junit.write_report('failed', output, 'example_project', 'example/project', 'board<&')
            report = ET.parse(output).getroot()

        testcase = report.find('.//testcase')
        failure = testcase.find('failure')
        self.assertEqual(testcase.attrib['classname'], 'example/project')
        self.assertEqual(testcase.attrib['name'], 'board<&: example_project')
        self.assertEqual(failure.attrib['message'], 'SDK build failed')


if __name__ == '__main__':
    unittest.main()
