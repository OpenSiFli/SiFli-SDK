#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Build a pipeline-scoped catalog without downloading firmware archives."""

import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timedelta, timezone
import html
import json
import os
import re
from pathlib import Path, PurePosixPath
import shutil
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urlsplit
from urllib.request import HTTPRedirectHandler, Request, build_opener

from dynamic_pipeline import DynamicPipelineGenerator


PACKAGE_NAME = 'sdk-artifact-catalog-pointers'
INDEX_PATH = 'merged_artifacts/file_index.json'
HISTORY_DAYS = 7
HISTORY_REFS = ('main', 'release/v2.5')
HISTORY_WORKERS = 4
# Allowed-failure exit code: a newer pipeline already owns the published entry.
PAGES_SKIPPED = 3
# GitLab deploys a successful pages job asynchronously; wait this long for a newer one.
PAGES_DEPLOY_TIMEOUT = 600
PAGES_DEPLOY_POLL = 10
FINISHED_STATUSES = ('success', 'failed', 'canceled', 'skipped')


class SafeRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        redirected = super().redirect_request(req, fp, code, msg, headers, newurl)
        if redirected and urlsplit(req.full_url).netloc != urlsplit(newurl).netloc:
            redirected.remove_header('Job-token')
            redirected.remove_header('Private-token')
        return redirected


class GitLab:
    def __init__(self):
        self.api = os.environ['CI_API_V4_URL'].rstrip('/')
        self.project = os.environ['CI_PROJECT_ID']
        self.pipeline = os.environ['CI_PIPELINE_ID']
        self.token = os.environ['CI_JOB_TOKEN']

    def json_request(self, path, data=None, api_token=None):
        url = f'{self.api}/projects/{quote(self.project, safe="")}/{path}'
        payload = json.dumps(data).encode('utf-8') if data is not None else None
        headers = {'Content-Type': 'application/json'}
        if api_token:
            headers['PRIVATE-TOKEN'] = api_token
        else:
            headers['JOB-TOKEN'] = self.token
        for attempt in range(3):
            request = Request(url, data=payload, method='PUT' if payload is not None else 'GET',
                              headers=headers)
            try:
                with build_opener(SafeRedirect()).open(request, timeout=30) as response:
                    return json.load(response)
            except HTTPError as exc:
                if exc.code not in (429, 500, 502, 503, 504) or attempt == 2:
                    # Do not log headers, tokens, response bodies or signed redirect URLs.
                    raise ValueError(f'GitLab request failed (HTTP {exc.code})') from None
            except (URLError, TimeoutError):
                if attempt == 2:
                    raise ValueError('GitLab request timed out or could not connect') from None
            time.sleep(attempt + 1)

    def read_json(self, path, resource):
        try:
            return self.json_request(path)
        except ValueError as exc:
            if not any(f'HTTP {code}' in str(exc) for code in (401, 403, 404)):
                raise
            api_token = os.getenv('GITLAB_API_READ_TOKEN')
            if not api_token or api_token == '$GITLAB_API_READ_TOKEN':
                raise ValueError(
                    f'CI_JOB_TOKEN cannot read {resource}; configure GITLAB_API_READ_TOKEN') from None
            return self.json_request(path, api_token=api_token)

    def pointer_path(self, batch, pipeline=None):
        return f'packages/generic/{PACKAGE_NAME}/{pipeline or self.pipeline}/{batch}.json'

    def load_index(self, batch):
        pointer = self.json_request(self.pointer_path(batch))
        if (not isinstance(pointer, dict)
                or str(pointer.get('pipeline_id')) != self.pipeline
                or str(pointer.get('project_id')) != self.project
                or pointer.get('commit') != os.environ['CI_COMMIT_SHA']
                or pointer.get('batch') != batch):
            raise ValueError('collector pointer belongs to another pipeline or commit')
        job_id = str(pointer.get('job_id', ''))
        if not job_id.isdecimal() or int(job_id) < 1:
            raise ValueError('invalid collector job ID')
        # Fetch only the small JSON file, after GitLab has stored the group archive.
        index = self.read_json(f'jobs/{job_id}/artifacts/{INDEX_PATH}', 'job artifacts')
        if (not isinstance(index, dict)
                or str(index.get('pipeline_id')) != self.pipeline
                or str(index.get('project_id')) != self.project
                or index.get('merge_strategy') != 'all'):
            raise ValueError('file index belongs to another pipeline or collection')
        if not isinstance(index.get('jobs'), dict):
            raise ValueError('invalid file index')
        return job_id, index

    def current_collector_jobs(self):
        return self.latest_jobs_with_prefix(f'pipelines/{self.pipeline}/jobs', 'collect_batch_')

    def latest_jobs_with_prefix(self, endpoint, prefix):
        jobs = []
        page = 1
        while True:
            path = f'{endpoint}?per_page=100&page={page}'
            batch = self.read_json(path, 'pipeline jobs')
            if not isinstance(batch, list):
                raise ValueError('invalid pipeline job list')
            jobs.extend(batch)
            if len(batch) < 100:
                break
            page += 1
        current = {}
        for job in jobs:
            if (not isinstance(job, dict) or job.get('retried') is True
                    or not str(job.get('name', '')).startswith(prefix)):
                continue
            name = job['name']
            job_id = str(job.get('id', ''))
            if not job_id.isdecimal() or int(job_id) < 1:
                continue
            if name not in current or int(job_id) > int(current[name]['id']):
                current[name] = {'id': job_id, 'status': job.get('status')}
        return current

    def latest_job(self, pipeline_id, name):
        jobs = self.latest_jobs_with_prefix(f'pipelines/{pipeline_id}/jobs', name)
        return jobs.get(name)


def publish_pointer(client):
    batch = os.environ['CI_JOB_NAME']
    if not batch.startswith('collect_batch_') or not batch[len('collect_batch_'):].isdigit():
        raise ValueError('only a collector may publish a catalog pointer')
    # A pointer is a few hundred bytes; firmware remains in job artifacts.
    client.json_request(client.pointer_path(batch), {
        'project_id': client.project,
        'pipeline_id': client.pipeline,
        'commit': os.environ['CI_COMMIT_SHA'],
        'batch': batch,
        'job_id': os.environ['CI_JOB_ID'],
    })
    print(f'Published catalog pointer for {batch}')


def publish_parent_pointer(client):
    parent = os.getenv('SDK_PARENT_PIPELINE_ID')
    if not parent:
        return
    if not parent.isdecimal() or int(parent) < 1:
        raise ValueError('invalid parent pipeline ID')
    client.json_request(client.pointer_path('artifact_catalog', pipeline=parent), {
        'kind': 'catalog',
        'project_id': client.project,
        'pipeline_id': client.pipeline,
        'parent_pipeline_id': parent,
        'commit': os.environ['CI_COMMIT_SHA'],
        'job_id': os.environ['CI_JOB_ID'],
    })


def load_parent_catalog(client):
    pointer = client.json_request(client.pointer_path('artifact_catalog'))
    bridges_path = f'pipelines/{client.pipeline}/bridges'
    bridges = client.read_json(bridges_path, 'pipeline triggers')
    if not isinstance(bridges, list):
        raise ValueError('invalid parent pipeline trigger list')
    current_children = {
        str(bridge.get('downstream_pipeline', {}).get('id'))
        for bridge in bridges
        if isinstance(bridge, dict)
        and bridge.get('name') == 'trigger_child_pipeline'
        and bridge.get('retried') is not True
        and isinstance(bridge.get('downstream_pipeline'), dict)
        and bridge['downstream_pipeline'].get('id') is not None
    }
    if len(current_children) != 1:
        raise ValueError('current child pipeline for trigger_child_pipeline is missing or ambiguous')
    if (not isinstance(pointer, dict) or pointer.get('kind') != 'catalog'
            or str(pointer.get('project_id')) != client.project
            or str(pointer.get('parent_pipeline_id')) != client.pipeline
            or pointer.get('commit') != os.environ['CI_COMMIT_SHA']):
        raise ValueError('catalog pointer belongs to another parent pipeline or commit')
    job_id = str(pointer.get('job_id', ''))
    pipeline_id = str(pointer.get('pipeline_id', ''))
    if any(not value.isdecimal() or int(value) < 1 for value in (job_id, pipeline_id)):
        raise ValueError('invalid catalog job or child pipeline ID')
    if pipeline_id not in current_children:
        raise ValueError('catalog pointer does not match the current child pipeline')
    current_catalog_job = client.latest_job(pipeline_id, 'artifact_catalog')
    if current_catalog_job is None:
        raise ValueError('current child artifact_catalog job is missing')
    if current_catalog_job['id'] != job_id:
        raise ValueError('catalog pointer does not match the latest child catalog attempt')
    if current_catalog_job['status'] not in ('success', 'failed'):
        raise ValueError(f'latest child catalog attempt is not finished: {current_catalog_job["status"]}')
    catalog = client.read_json(f'jobs/{job_id}/artifacts/artifact_catalog/catalog.json', 'job artifacts')
    if (not isinstance(catalog, dict)
            or str(catalog.get('pipeline_id')) != pipeline_id
            or str(catalog.get('project_id')) != client.project
            or catalog.get('commit') != os.environ['CI_COMMIT_SHA']
            or not isinstance(catalog.get('rows'), list)
            or not isinstance(catalog.get('issues'), list)):
        raise ValueError('catalog belongs to another child pipeline or commit')
    return catalog


def newer_published_pipeline(client):
    """Return a newer default-branch pipeline whose Pages deployment succeeded."""
    ref = quote(os.environ['CI_DEFAULT_BRANCH'], safe='')
    page = 1
    while True:
        pipelines = client.read_json(
            f'pipelines?ref={ref}&order_by=id&sort=desc&per_page=100&page={page}', 'pipelines')
        if not isinstance(pipelines, list):
            raise ValueError('invalid pipeline list')
        for pipeline in pipelines:
            pipeline_id = str(pipeline.get('id', '')) if isinstance(pipeline, dict) else ''
            if not pipeline_id.isdecimal():
                continue
            if int(pipeline_id) <= int(client.pipeline):
                return None
            # Child pipelines hold thousands of build jobs and never run pages.
            if pipeline.get('source') == 'parent_pipeline':
                continue
            sha = pipeline.get('sha')
            if not isinstance(sha, str) or not sha:
                raise ValueError(f'pipeline #{pipeline_id} has no commit SHA')
            if pages_deployed(client, pipeline_id, sha):
                return pipeline_id
        if len(pipelines) < 100:
            return None
        page += 1


def commit_statuses(client, sha, pipeline_id, name):
    statuses = []
    page = 1
    while True:
        # all=true keeps retried attempts; the latest attempt alone may hide a deployment.
        batch = client.read_json(
            f'repository/commits/{quote(sha, safe="")}/statuses?pipeline_id={pipeline_id}'
            f'&name={quote(name, safe="")}&all=true&per_page=100&page={page}', 'commit statuses')
        if not isinstance(batch, list):
            raise ValueError('invalid commit status list')
        statuses.extend(status.get('status') for status in batch if isinstance(status, dict))
        if len(batch) < 100:
            return statuses
        page += 1


def pages_deployed(client, pipeline_id, sha):
    """Return whether GitLab actually deployed a pages attempt of the pipeline.

    A successful pages job only uploads artifacts; GitLab then deploys them
    asynchronously and records the result as a pages:deploy status.
    """
    deadline = time.monotonic() + PAGES_DEPLOY_TIMEOUT
    while True:
        deploys = commit_statuses(client, sha, pipeline_id, 'pages:deploy')
        if 'success' in deploys:
            return True
        jobs = commit_statuses(client, sha, pipeline_id, 'pages')
        # Every successful pages job gets one pages:deploy status once GitLab picks it up.
        pending = (any(status not in FINISHED_STATUSES for status in deploys)
                   or jobs.count('success') > len(deploys))
        if not pending:
            return False
        if time.monotonic() >= deadline:
            raise ValueError(f'Pages deployment of pipeline #{pipeline_id} is still unfinished')
        time.sleep(PAGES_DEPLOY_POLL)



def pipeline_created_at(value):
    if not isinstance(value, str):
        raise ValueError('pipeline has no creation time')
    created = datetime.fromisoformat(value.replace('Z', '+00:00'))
    if created.tzinfo is None:
        raise ValueError('pipeline creation time has no timezone')
    return created


def history_entry(pipeline, catalog, project):
    # The parent job reuses its child's catalog, so catalog.pipeline_id is a child ID.
    if (not isinstance(catalog, dict) or str(catalog.get('project_id')) != project
            or not pipeline.get('sha') or catalog.get('commit') != pipeline['sha']
            or not str(catalog.get('pipeline_id', '')).isdecimal()
            or not isinstance(catalog.get('pipeline_url'), str)
            or not isinstance(catalog.get('generated_at'), str)
            or not isinstance(catalog.get('rows'), list)
            or not isinstance(catalog.get('issues'), list) or catalog['issues']
            or catalog.get('smoke_test')):
        raise ValueError('catalog is incomplete, simulated, or belongs to another project/commit')
    pipeline_created_at(catalog['generated_at'])
    required_strings = ('label', 'project', 'board', 'batch', 'job_url', 'browse_url')
    for row in catalog['rows']:
        if (not isinstance(row, dict)
                or any(not isinstance(row.get(key), str) for key in required_strings)
                or row.get('status') not in ('success', 'failed', 'unknown', 'unavailable')
                or not isinstance(row.get('files'), list) or not isinstance(row.get('logs'), list)):
            raise ValueError('invalid catalog row')
        for file in (*row['files'], *row['logs']):
            if (not isinstance(file, dict) or not isinstance(file.get('name'), str)
                    or not isinstance(file.get('url'), str)
                    or type(file.get('size')) is not int or file['size'] < 0):
                raise ValueError('invalid catalog file entry')
    pipeline_created_at(pipeline.get('created_at'))
    return {
        'pipeline_id': str(pipeline['id']), 'ref': pipeline['ref'], 'commit': pipeline['sha'],
        'created_at': pipeline['created_at'],
        'success_count': sum(row['status'] == 'success' for row in catalog['rows']),
        'failed_count': sum(row['status'] == 'failed' for row in catalog['rows']),
        'catalog': catalog,
    }


def load_history(client):
    """Rebuild recent parent catalogs from artifacts; a broken entry is isolated."""
    cutoff = datetime.now(timezone.utc) - timedelta(days=HISTORY_DAYS)
    updated_after = quote(cutoff.isoformat(), safe='')
    candidates = []
    for branch in HISTORY_REFS:
        page = 1
        while True:
            ref = quote(branch, safe='')
            # Listing failures are fatal: publishing a silently empty history would hide builds.
            pipelines = client.read_json(
                f'pipelines?ref={ref}&updated_after={updated_after}'
                f'&order_by=id&sort=desc&per_page=100&page={page}', 'pipeline history')
            if not isinstance(pipelines, list):
                raise ValueError(f'invalid pipeline history list for {branch}')
            for pipeline in pipelines:
                pipeline_id = str(pipeline.get('id', '')) if isinstance(pipeline, dict) else ''
                if (not pipeline_id.isdecimal() or int(pipeline_id) < 1
                        or int(pipeline_id) == int(client.pipeline)
                        or pipeline.get('source') == 'parent_pipeline'):
                    continue
                try:
                    # updated_after is only a discovery filter; retention uses creation time.
                    if pipeline_created_at(pipeline.get('created_at')) < cutoff:
                        continue
                    pipeline['ref'] = branch
                    candidates.append(pipeline)
                except (ValueError, KeyError, TypeError, OSError) as exc:
                    print(f'Warning: skipping history pipeline #{pipeline_id}: {exc}', file=sys.stderr)
            if len(pipelines) < 100:
                break
            page += 1

    def load_entry(pipeline):
        pipeline_id = str(pipeline['id'])
        try:
            job = client.latest_job(pipeline_id, 'artifact_catalog')
            # Push, sync and other non-build pipelines on the branch have no catalog.
            if job is None:
                return None
            if job.get('status') != 'success':
                raise ValueError('latest artifact_catalog attempt is not successful')
            catalog = client.read_json(
                f'jobs/{job["id"]}/artifacts/artifact_catalog/catalog.json', 'historical catalog')
            return history_entry(pipeline, catalog, client.project)
        except (ValueError, KeyError, TypeError, OSError) as exc:
            print(f'Warning: skipping history pipeline #{pipeline_id}: {exc}', file=sys.stderr)
            return None

    # Bound API concurrency so non-build pipelines do not serialize Pages publication
    # or create an unbounded burst against GitLab.
    with ThreadPoolExecutor(max_workers=HISTORY_WORKERS) as pool:
        return [entry for entry in pool.map(load_entry, candidates) if entry is not None]


def publish_pages(client, source, output):
    # Pages also runs after build failures, so only publish files from a catalog job that passed.
    catalog_job = client.latest_job(client.pipeline, 'artifact_catalog')
    if catalog_job is None or catalog_job['status'] != 'success':
        status = catalog_job['status'] if catalog_job else 'missing'
        raise ValueError(f'artifact_catalog job is {status}; keeping the published entry')
    catalog = json.loads((source / 'catalog.json').read_text(encoding='utf-8'))
    if (not isinstance(catalog, dict) or catalog.get('commit') != os.environ['CI_COMMIT_SHA']
            or not isinstance(catalog.get('issues'), list)
            or not isinstance(catalog.get('rows'), list) or not (source / 'index.html').is_file()):
        raise ValueError('catalog page is missing or belongs to another commit')
    # Build failures are row statuses; issues mean the catalog itself is incomplete.
    if catalog['issues']:
        raise ValueError(f'catalog has {len(catalog["issues"])} issues; keeping the published entry')
    newer = newer_published_pipeline(client)
    if newer:
        return newer
    history = load_history(client)
    created_at = os.environ['CI_PIPELINE_CREATED_AT']
    cutoff = datetime.now(timezone.utc) - timedelta(days=HISTORY_DAYS)
    if not catalog.get('smoke_test') and pipeline_created_at(created_at) >= cutoff:
        history.append(history_entry(
            {'id': client.pipeline, 'ref': os.environ['CI_COMMIT_REF_NAME'],
             'sha': os.environ['CI_COMMIT_SHA'], 'created_at': created_at},
            catalog, client.project))
    history.sort(key=lambda entry: (pipeline_created_at(entry['created_at']), int(entry['pipeline_id'])),
                 reverse=True)
    index = [{key: value for key, value in entry.items() if key != 'catalog'} for entry in history]
    # A Pages smoke run exercises history without promoting simulated data to a real build.
    default_entry = next((entry for entry in history if entry['ref'] == 'main'), None)
    if default_entry is None and (not catalog.get('smoke_test') or history):
        raise ValueError('no unexpired complete catalog for main is available; keeping the published entry')
    latest = default_entry['catalog'] if default_entry else catalog
    shutil.rmtree(output, ignore_errors=True)
    write_catalog(latest, output, history={
        'days': HISTORY_DAYS, 'pipelines': index,
        'default_pipeline_id': default_entry['pipeline_id'] if default_entry else None,
    } if history else None)
    (output / 'history.json').write_text(json.dumps(index, ensure_ascii=False), encoding='utf-8')
    for entry in history:
        directory = output / 'pipelines' / entry['pipeline_id']
        directory.mkdir(parents=True)
        (directory / 'catalog.json').write_text(
            json.dumps(entry['catalog'], ensure_ascii=False, separators=(',', ':')), encoding='utf-8')
    return None


def artifact_path(path):
    if not isinstance(path, str) or not path or '\\' in path:
        raise ValueError('invalid artifact path')
    parsed = PurePosixPath(path)
    if parsed.is_absolute() or '..' in parsed.parts:
        raise ValueError('invalid artifact path')
    return quote(path, safe='/')


def board_label(project, board):
    if board:
        return board
    parts = PurePosixPath(project).parts
    # Legacy projects encode board/core in their directory rather than --board.
    if len(parts) >= 3 and parts[-3] == 'project' and parts[-1] in ('hcpu', 'lcpu', 'acpu'):
        return f'{parts[-2]}_{parts[-1]}'
    return '工程内置配置'


def build_catalog(generator, client, project_url):
    targets = generator.get_build_jobs()
    batches = {}
    for offset in range(0, len(targets), generator.batch_size):
        batch = f'collect_batch_{offset // generator.batch_size + 1:03d}'
        batches[batch] = targets[offset:offset + generator.batch_size]

    def load(batch):
        try:
            job_id, index = client.load_index(batch)
            current_job = current_jobs.get(batch)
            if current_job is None:
                raise ValueError('current collector attempt is missing from pipeline jobs')
            if job_id != current_job['id']:
                raise ValueError('collector pointer does not match the latest attempt')
            if current_job['status'] != 'success':
                raise ValueError(f'latest collector attempt status is {current_job["status"]}')
            return (job_id, index), None
        except (ValueError, KeyError, TypeError) as exc:
            return None, str(exc)

    try:
        current_jobs = client.current_collector_jobs()
    except (ValueError, KeyError, TypeError) as exc:
        current_jobs = {}
        jobs_error = str(exc)
    else:
        jobs_error = None
    with ThreadPoolExecutor(max_workers=8) as pool:
        loaded = dict(zip(batches, pool.map(load, batches)))

    rows, issues = [], []
    for batch, jobs in batches.items():
        result, error = loaded[batch]
        if jobs_error:
            result, error = None, jobs_error
        job_id, index = result if result else (None, {'jobs': {}})
        if jobs_error:
            issues.append(f'{batch}: unable to verify current collector attempt: {jobs_error}')
        elif error:
            issues.append(f'{batch}: {error}')
        for name, project, board, _common in jobs:
            label = name[len('build_'):]
            row = {'label': label, 'project': project, 'board': board_label(project, board),
                   'batch': batch, 'status': 'unavailable', 'files': [], 'logs': []}
            record = index['jobs'].get(label)
            if record is not None:
                try:
                    if not isinstance(record, dict):
                        raise ValueError('invalid target index')
                    status = record['status']
                    if status not in ('success', 'failed', 'unknown'):
                        raise ValueError('invalid build status')
                    indexed_artifact_count = len(record.get('artifacts', []))
                    root = f'{project_url}/-/jobs/{job_id}/artifacts'
                    files, logs = [], []
                    for category, output in [('artifacts', files), ('logs', logs)]:
                        entries = record.get(category, [])
                        if not isinstance(entries, list):
                            raise ValueError('invalid file list')
                        for item in entries:
                            if not isinstance(item, dict):
                                raise ValueError('invalid file entry')
                            path = item['path']
                            encoded = artifact_path(path)
                            prefix = f'{label}/artifacts/{label}/' if category == 'artifacts' else f'{label}/ci_build_logs/'
                            if not path.startswith(prefix):
                                raise ValueError('artifact belongs to another target')
                            size = item['size']
                            if type(size) is not int or size < 0:
                                raise ValueError('invalid file size')
                            url = (f'{root}/raw/merged_artifacts/{encoded}?inline=false'
                                   if category == 'artifacts' else f'{root}/file/merged_artifacts/{encoded}')
                            output.append({'name': path[len(prefix):], 'size': size,
                                           'url': url})
                    # Failed/unknown builds never offer firmware as successful output.
                    row.update(status=status, files=files if status == 'success' else [],
                               logs=logs, job_url=f'{project_url}/-/jobs/{job_id}',
                               browse_url=f'{root}/browse/merged_artifacts/{quote(label, safe="")}/',
                               _indexed_artifact_count=indexed_artifact_count)
                except (KeyError, TypeError, ValueError):
                    issues.append(f'{batch}/{label}: invalid target index')
            else:
                issues.append(f'{batch}/{label}: build record missing')
            rows.append(row)
    catalog = {
        'project_id': client.project,
        'pipeline_id': client.pipeline,
        'smoke_test': generator.smoke_test is True,
        'commit': os.environ['CI_COMMIT_SHA'],
        'pipeline_url': f'{project_url}/-/pipelines/{client.pipeline}',
        'generated_at': datetime.now(timezone.utc).isoformat(),
        'issues': issues,
        'rows': rows,
    }
    if generator.smoke_test is True:
        validate_smoke_catalog(catalog)
    return catalog


def validate_smoke_catalog(catalog):
    expected_failure = [row for row in catalog['rows']
                        if 'expected-failure' in row['project']]
    expected_success = [row for row in catalog['rows']
                        if 'expected-failure' not in row['project']]
    if len(catalog['rows']) != 3 or len(expected_success) != 2 or len(expected_failure) != 1:
        catalog['issues'].append('smoke test target count does not match the three expected fixtures')
        return

    for row in expected_success:
        if row['status'] != 'success':
            catalog['issues'].append(f"smoke test success target has status {row['status']}: {row['label']}")
        if (len(row['files']) != 1 or row['files'][0]['name'] != 'SMOKE_TEST_ONLY.txt'
                or row['files'][0]['size'] >= 512):
            catalog['issues'].append(f"smoke test output missing or invalid: {row['label']}")
        if len(row['logs']) != 1:
            catalog['issues'].append(f"smoke test log missing or duplicated: {row['label']}")

    row = expected_failure[0]
    if row['status'] != 'failed':
        catalog['issues'].append(f"smoke test expected-failure target has status {row['status']}: {row['label']}")
    if row.get('_indexed_artifact_count') != 0:
        catalog['issues'].append(f"smoke test expected-failure target unexpectedly has output: {row['label']}")
    if len(row['logs']) != 1:
        catalog['issues'].append(f"smoke test log missing or duplicated: {row['label']}")
    for row in catalog['rows']:
        row.pop('_indexed_artifact_count', None)


def write_catalog(catalog, output, history=None):
    output.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(catalog, ensure_ascii=False, separators=(',', ':'))
    (output / 'catalog.json').write_text(payload, encoding='utf-8')
    # Embedded data makes the page work offline, without CORS or a user API token.
    template = Path(__file__).with_name('artifact_catalog.html').read_text(encoding='utf-8')
    def embed(value):
        text = json.dumps(value, ensure_ascii=False, separators=(',', ':'))
        return text.replace('<', '\\u003c').replace('>', '\\u003e').replace('&', '\\u0026')
    data = {'__CATALOG_DATA__': embed(catalog), '__HISTORY_DATA__': embed(history)}
    # One pass, so a file name containing a placeholder cannot be substituted twice.
    page = re.sub('__(?:CATALOG|HISTORY)_DATA__', lambda match: data[match.group(0)], template)
    (output / 'index.html').write_text(page, encoding='utf-8')
    # A native GitLab-rendered fallback that also works when HTML preview is disabled.
    lines = [f'# SDK 产物目录 · Pipeline {catalog["pipeline_id"]}', '',
             '下载同目录的 index.html 并在浏览器打开，可按板子、工程、状态筛选。', '',
             f'Commit: `{catalog["commit"]}`', '',
             '产物通常保留一周；目录中的链接始终指向本次构建，过期后不会切换到其他版本。', '']
    if catalog.get('smoke_test'):
        lines.extend(['**快速验证数据：仅用于测试目录、日志和下载，不含可烧录固件。**', ''])
    if catalog['issues']:
        lines.extend([f'**目录不完整：{len(catalog["issues"])} 项异常。** 请查看 catalog.json 的 issues。', ''])
    lines.extend(['| 板子 | 工程 | 状态 | 文件 |', '|---|---|---|---|'])
    for row in catalog['rows']:
        escape = lambda value: html.escape(str(value)).replace('|', '&#124;').replace('\n', ' ')
        link = f'[查看文件]({row["browse_url"]})' if row.get('browse_url') else '无可用清单'
        lines.append(f'| {escape(row["board"])} | {escape(row["project"])} | {row["status"]} | {link} |')
    (output / 'README.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['publish', 'generate', 'entry', 'pages'])
    parser.add_argument('--config', default='build_config_gcc.yaml')
    parser.add_argument('--output', type=Path, default=Path('artifact_catalog'))
    parser.add_argument('--pages-output', type=Path, default=Path('public'))
    args = parser.parse_args()
    try:
        client = GitLab()
        if args.action == 'publish':
            publish_pointer(client)
            return 0
        if args.action == 'pages':
            newer = publish_pages(client, args.output, args.pages_output)
            if newer:
                print(f'Pipeline #{newer} already published a newer catalog; keeping it')
                return PAGES_SKIPPED
            print(f'Publishing catalog from pipeline #{client.pipeline}')
            return 0
        if args.action == 'entry':
            catalog = load_parent_catalog(client)
        else:
            generator = DynamicPipelineGenerator(args.config,
                                                  smoke_test=os.getenv('CATALOG_SMOKE_TEST') == '1')
            catalog = build_catalog(generator, client, os.environ['CI_PROJECT_URL'].rstrip('/'))
        write_catalog(catalog, args.output)
        if args.action == 'generate':
            publish_parent_pointer(client)
        print(f'Artifact catalog: {os.environ["CI_PROJECT_URL"]}/-/jobs/{os.environ["CI_JOB_ID"]}'
              '/artifacts/browse/artifact_catalog/')
        print(f'{len(catalog["rows"])} targets, {len(catalog["issues"])} catalog issues')
        # Still upload the partial catalog for diagnosis; never silently call it complete.
        return 1 if catalog['issues'] else 0
    except (KeyError, ValueError, OSError) as exc:
        print(f'Artifact catalog failed: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
