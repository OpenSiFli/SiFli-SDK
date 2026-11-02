# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

import importlib.util
import sys
import unittest
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
GENERATOR_PATH = REPO_ROOT / 'tools/ci/dynamic_pipeline.py'
RESOLVER_PATH = REPO_ROOT / 'tools/ci/resolve_build_job.py'

spec = importlib.util.spec_from_file_location('dynamic_pipeline', GENERATOR_PATH)
dynamic_pipeline = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dynamic_pipeline)
sys_path = str(GENERATOR_PATH.parent)
if sys_path not in sys.path:
    sys.path.insert(0, sys_path)
resolver_spec = importlib.util.spec_from_file_location('resolve_build_job', RESOLVER_PATH)
resolver = importlib.util.module_from_spec(resolver_spec)
resolver_spec.loader.exec_module(resolver)


class DynamicPipelineTests(unittest.TestCase):
    def test_generated_config_uses_bounded_batch_triggers(self):
        config_path = REPO_ROOT / 'build_config_gcc.yaml'
        generator = dynamic_pipeline.DynamicPipelineGenerator(config_path)
        pipeline = generator.generate_child_pipeline()
        bridge_jobs = [name for name in pipeline if name.startswith('build_batch_')]
        build_jobs = generator.get_build_jobs()

        self.assertGreater(len(build_jobs), 0)
        self.assertEqual(
            len(bridge_jobs),
            (len(build_jobs) + generator.batch_size - 1) // generator.batch_size,
        )
        self.assertEqual(len(pipeline['stages']), len(bridge_jobs))
        self.assertEqual(
            [pipeline[name]['stage'] for name in bridge_jobs],
            pipeline['stages'],
        )
        self.assertTrue(all(
            pipeline[name]['trigger']['strategy'] == 'depend'
            for name in bridge_jobs
        ))
        self.assertTrue(all(
            pipeline[name]['variables']['BUILD_CONFIG'] == str(config_path)
            for name in bridge_jobs
        ))
        self.assertLess(len(yaml.safe_dump(pipeline).encode('utf-8')), 32_000)
        self.assertTrue(all(
            pipeline[name]['trigger']['include'][0]['local']
            == '.gitlab/ci/build_sdk_batch.yml'
            for name in bridge_jobs
        ))

    def test_resolver_covers_all_jobs_once(self):
        config_path = REPO_ROOT / 'build_config_gcc.yaml'
        generator = dynamic_pipeline.DynamicPipelineGenerator(config_path)
        jobs = generator.get_build_jobs()
        resolved = [
            resolver.resolve_build_job(
                generator,
                job_index // generator.batch_size,
                job_index % generator.batch_size + 1,
            )
            for job_index in range(len(jobs))
        ]

        self.assertEqual([job['LOG_NAME'] for job in resolved], [job[0][len('build_'):] for job in jobs])
        self.assertEqual(len({job['LOG_NAME'] for job in resolved}), len(jobs))

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
        self.assertIn('|| exit 1', template)
        self.assertNotIn('\\"$BATCH_INDEX\\"', template)


if __name__ == '__main__':
    unittest.main()
