# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

import importlib.util
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
        self.assertEqual(pipeline['stages'], ['build', 'collect'])
        self.assertEqual(pipeline['include'], [{'local': '.gitlab/ci/build_sdk_batch.yml'}])
        for index, name in enumerate(batch_jobs):
            suffix = f'{index + 1:03d}'
            self.assertEqual(pipeline[name]['stage'], 'build')
            self.assertEqual(pipeline[name]['extends'], '.build_sdk_batch_template')
            self.assertNotIn('trigger', pipeline[name])
            self.assertEqual(pipeline[name]['parallel'], generator.batch_size)
            self.assertEqual(
                pipeline[f'collect_batch_{suffix}']['needs'],
                [{'job': name, 'artifacts': True}],
            )
            self.assertEqual(pipeline[f'collect_batch_{suffix}']['stage'], 'collect')
        self.assertTrue(all(
            pipeline[name]['variables']['BUILD_CONFIG'] == str(config_path)
            for name in batch_jobs
        ))
        self.assertLess(len(yaml.safe_dump(pipeline).encode('utf-8')), 32_000)

    def test_emitted_parallel_slots_cover_all_targets_within_needs_limit(self):
        generator = dynamic_pipeline.DynamicPipelineGenerator(REPO_ROOT / 'build_config_gcc.yaml')
        pipeline = generator.generate_child_pipeline()
        jobs = generator.get_build_jobs()
        resolved = []
        for name, job in pipeline.items():
            if not name.startswith('collect_batch_'):
                continue
            expanded_needs = 0
            for need in job['needs']:
                build = pipeline[need['job']]
                expanded_needs += build['parallel']
                batch_index = int(build['variables']['BATCH_INDEX'])
                for node_index in range(1, build['parallel'] + 1):
                    target = resolver.resolve_build_job(generator, batch_index, node_index)
                    if target is not None:
                        resolved.append(target['LOG_NAME'])
            self.assertLessEqual(expanded_needs, 50, name)

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
        self.assertIn('--batch-index "$BATCH_INDEX"', template)
        self.assertIn('--node-index "$CI_NODE_INDEX"', template)
        self.assertIn('after_script:', template)
        self.assertIn('|| exit 1', template)
        self.assertIn('junit: job_status/*.junit.xml', template)
        self.assertNotIn('parallel', yaml.safe_load(template)['.build_sdk_batch_template'])
        self.assertIn('collect_artifacts_simple.py --failed-only', template)
        self.assertNotIn('\\"$BATCH_INDEX\\"', template)

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
