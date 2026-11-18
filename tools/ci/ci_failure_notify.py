#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Push SDK CI failures to a WeCom (企业微信) group robot.

Runs on an intranet server as a GitLab Pipeline webhook receiver. Only the
standard library is used so the server needs no extra packages (Python 3.8+).
See CI_FAILURE_NOTIFY.md for deployment.
"""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import hashlib
import hmac
import json
import os
from pathlib import Path
import queue
import re
import sqlite3
import sys
import threading
import time
import uuid
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urlsplit
from urllib.request import HTTPRedirectHandler, Request, build_opener

DEFAULT_REFS = 'main,release/v2.5'
# Full SDK builds are started by hand, by schedule or through the API; MR and
# push pipelines are left to their authors.
NOTIFY_SOURCES = ('schedule', 'web', 'api')
# Present on both main and release/v2.5 full builds, absent from sync/mirror pipelines.
FULL_BUILD_BRIDGE = 'trigger_child_pipeline'
MAX_DEPTH = 4
PER_PAGE = 100
MAX_TRACE_JOBS = 5
MAX_TRACE_BYTES = 8 * 1024 * 1024
MAX_ERROR_CHARS = 160
# WeCom rejects markdown content longer than 4096 bytes.
MAX_MESSAGE_BYTES = 4000
MAX_BODY_BYTES = 4 * 1024 * 1024
MAX_PENDING_WEBHOOKS = 256
MAX_QUEUE_RECORDS = 1024
PROCESSING_LEASE_SECONDS = 3600
# GitLab does not redeliver a webhook we answered with 200, so failed events are
# retried here (seconds after each failure), then left for `notify --pipeline`.
RETRY_DELAYS = (60, 300, 900)
# Drop clients that connect but never send, so they cannot hold a handler thread forever.
REQUEST_TIMEOUT = 10
WECOM_SEND_URL = 'https://qyapi.weixin.qq.com/cgi-bin/webhook/send?key={key}'

ANSI_RE = re.compile(r'\x1b\[[0-9;]*[A-Za-z]|\x1b\][^\x07]*\x07|\r')
SECTION_RE = re.compile(r'section_(start|end):\d+:\S*')
# GitLab 18 prefixes every trace line with a timestamp and stream marker, e.g. `...Z 01O `.
TIMESTAMP_RE = re.compile(r'^\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d(?:\.\d+)?Z [0-9a-f]{2}[OE]\+?\s?', re.M)
# Runner checkout prefix, e.g. `/data/gitlab-runner/builds/<id>/<n>/<group>/<project>/`.
BUILD_DIR_RE = re.compile(r'\S*/builds/[^/\s]+/\d+/[^/\s]+/[^/\s]+/')
# Matrix jobs carry the target in brackets; plain jobs are `build_<target>` (see dynamic_pipeline.py).
MATRIX_LABEL_RE = re.compile(r'^build_batch_\d+: \[(.+)\]$')
ERROR_RE = re.compile(r'(\berror\b\s*[:\[]|undefined reference to|scons: \*\*\*|\bfatal\b\s*:)', re.I)
# A gcc error that names its -Werror flag is still an error, so -Werror is not excluded here.
NOT_ERROR_RE = re.compile(r'\b0 errors?\b|^warning:|ERROR: Job failed', re.I)


def log(message):
    print(time.strftime('%Y-%m-%d %H:%M:%S'), message, file=sys.stderr, flush=True)


class GitLabError(ValueError):
    """A failed GitLab request; `code` is the HTTP status, or None when unreachable."""

    def __init__(self, message, code=None):
        super().__init__(message)
        self.code = code


class DeliveryUncertainError(ValueError):
    """The remote service may have accepted a message but did not confirm it."""


class SafeRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        original = urlsplit(req.full_url)
        target = urlsplit(newurl)
        if original.scheme == 'https' and target.scheme != 'https':
            return None
        redirected = super().redirect_request(req, fp, code, msg, headers, newurl)
        if redirected and original.netloc != target.netloc:
            redirected.remove_header('Private-token')
        return redirected


class GitLab:
    """Read-only GitLab API access with a `read_api` token."""

    def __init__(self, url, token):
        self.api = url.rstrip('/') + '/api/v4'
        self.token = token
        self.trigger_jobs_path = None

    def request(self, path, raw=False):
        request = Request(f'{self.api}/{path}', headers={'PRIVATE-TOKEN': self.token})
        for attempt in range(3):
            try:
                with build_opener(SafeRedirect()).open(request, timeout=30) as response:
                    if raw:
                        return response.read(MAX_TRACE_BYTES).decode('utf-8', 'replace')
                    return json.load(response)
            except HTTPError as exc:
                if exc.code not in (429, 500, 502, 503, 504) or attempt == 2:
                    # Never log headers or bodies: they may carry the token.
                    raise GitLabError(f'GitLab request failed (HTTP {exc.code}): {path}', exc.code) from None
            except (URLError, TimeoutError, OSError):
                if attempt == 2:
                    raise GitLabError(f'GitLab request could not connect: {path}') from None
            time.sleep(attempt + 1)

    def project_path(self, project, path):
        return f'projects/{quote(str(project), safe="")}/{path}'

    def get(self, project, path):
        return self.request(self.project_path(project, path))

    def get_all(self, project, path):
        items, page = [], 1
        separator = '&' if '?' in path else '?'
        while True:
            batch = self.get(project, f'{path}{separator}per_page={PER_PAGE}&page={page}')
            if not isinstance(batch, list):
                raise ValueError(f'unexpected GitLab list response: {path}')
            items.extend(batch)
            if len(batch) < PER_PAGE:
                return items
            page += 1

    def trigger_jobs(self, project, pipeline_id):
        """Read pipeline trigger jobs using the endpoint supported by this GitLab."""
        if self.trigger_jobs_path is None:
            version = self.request('version').get('version', '')
            match = re.match(r'^(\d+)\.(\d+)', version)
            if not match:
                raise GitLabError('GitLab API returned an unrecognized version')
            self.trigger_jobs_path = ('trigger_jobs' if tuple(map(int, match.groups())) >= (19, 2)
                                      else 'bridges')
        return self.get_all(project, f'pipelines/{pipeline_id}/{self.trigger_jobs_path}')

    def trace(self, project, job_id):
        return self.request(self.project_path(project, f'jobs/{job_id}/trace'), raw=True)

    def artifact(self, project, job_id, path):
        return self.request(self.project_path(project, f'jobs/{job_id}/artifacts/{quote(path)}'),
                            raw=True)


def build_log_path(job_name):
    """Artifact path of the compiler output that tools/ci/build_job.sh writes, or None.

    The job log itself only has the build wrapper; scons output goes to
    `ci_build_logs/<LOG_NAME>.log`, where LOG_NAME is the job label.
    """
    match = MATRIX_LABEL_RE.match(job_name)
    if match:
        label = match.group(1)
    elif job_name.startswith('build_') and not job_name.startswith('build_batch_'):
        label = job_name[len('build_'):]
    else:
        return None
    return f'ci_build_logs/{label}.log'


def job_error(gitlab, project, job):
    """First error of a job: from its build log artifact, else from its job log."""
    path = build_log_path(job['name'])
    if path:
        try:
            error = first_error(gitlab.artifact(project, job['id'], path))
            if error:
                return error
        except GitLabError as exc:
            # Expired or missing artifacts fall back to the job log.
            if exc.code not in (403, 404):
                raise
    return first_error(gitlab.trace(project, job['id']))


def first_error(trace):
    """Return the first compiler/scons error line of a job log, or None."""
    for line in TIMESTAMP_RE.sub('', SECTION_RE.sub('', ANSI_RE.sub('', trace))).splitlines():
        line = BUILD_DIR_RE.sub('', line.strip())
        if line and ERROR_RE.search(line) and not NOT_ERROR_RE.search(line):
            return line if len(line) <= MAX_ERROR_CHARS else line[:MAX_ERROR_CHARS - 3] + '...'
    return None


def failed_jobs(gitlab, project, pipeline_id, depth=0):
    """Failed jobs of a pipeline and of its downstream pipelines, parents first.

    Both main and release/v2.5 are parent -> one child today (main's batches are
    `parallel:matrix` jobs, not bridges); deeper nesting is handled the same way.
    A failed bridge is expanded into its downstream jobs; it is listed itself when
    it created no pipeline (e.g. invalid child YAML), the child cannot be read, or
    the depth limit is reached. Every job records the project its log lives in.
    """
    jobs = [dict(job, _project=project)
            for job in gitlab.get_all(project, f'pipelines/{pipeline_id}/jobs?scope[]=failed')
            if not job.get('allow_failure')]
    downstream = []
    for bridge in gitlab.trigger_jobs(project, pipeline_id):
        # A retried trigger points at an old child; only the latest attempt counts.
        if bridge.get('retried') is True or bridge.get('allow_failure'):
            continue
        bridge = dict(bridge, _project=project)
        child = bridge.get('downstream_pipeline')
        if isinstance(child, dict) and child.get('id'):
            if child.get('status') == 'failed':
                downstream.append(bridge)
            elif child.get('status') != 'success' and bridge.get('status') == 'failed':
                jobs.append(bridge)
        elif bridge.get('status') == 'failed':
            jobs.append(bridge)
    jobs.sort(key=lambda job: job['id'])
    for bridge in downstream:
        child = bridge['downstream_pipeline']
        if depth >= MAX_DEPTH:
            jobs.append(bridge)
            continue
        try:
            # Multi-project triggers run in another project.
            jobs.extend(failed_jobs(gitlab, child.get('project_id') or project, child['id'], depth + 1))
        except GitLabError as exc:
            # A child we may not read (e.g. another project) is reported as its bridge.
            # Transient errors propagate so the worker retries the whole event later
            # instead of sending a message without the real failures.
            if exc.code not in (403, 404):
                raise
            log(f'warning: cannot read downstream pipeline {child["id"]}: {exc}')
            jobs.append(bridge)
    return jobs


def is_full_build(gitlab, project, pipeline_id):
    trigger_jobs = gitlab.trigger_jobs(project, pipeline_id)
    return any(job.get('name') == FULL_BUILD_BRIDGE for job in trigger_jobs)


def catalog_url(gitlab, project, pipeline_id, project_url):
    """Link to the parent artifact catalog when this pipeline produced one."""
    jobs = gitlab.get_all(project, f'pipelines/{pipeline_id}/jobs?scope[]=success')
    job = next((job for job in jobs if job.get('name') == 'artifact_catalog'), None)
    if job is None:
        return None
    return f'{project_url}/-/jobs/{job["id"]}/artifacts/browse/artifact_catalog/'


def plain(text):
    """Make free text safe inside a WeCom markdown line or inline code."""
    return ' '.join(str(text).replace('`', "'").split())


def truncate_utf8(text, limit):
    encoded = text.encode('utf-8')
    if len(encoded) <= limit:
        return text
    suffix = '...'
    return encoded[:limit - len(suffix)].decode('utf-8', 'ignore') + suffix


def link_text(text):
    """Matrix job names look like `build_batch_001: [label]`; brackets would end the link text."""
    return plain(text).replace('[', '(').replace(']', ')')


def build_message(info, jobs, errors, catalog=None):
    """Render the WeCom markdown message within the byte limit."""
    commit = info['sha'][:10]
    title = plain(info.get('commit_title') or '')
    header = [
        f'## <font color="warning">SDK CI 失败</font> {truncate_utf8(plain(info["ref"]), 128)}',
        f'> 流水线：[#{truncate_utf8(str(info["id"]), 128)}]('
        f'{truncate_utf8(info["web_url"], 768)})　触发：'
        f'{truncate_utf8(plain(info["source"]), 64)}',
        f'> 提交：`{commit}` {truncate_utf8(title, 256)}',
        f'> 失败任务：<font color="warning">{len(jobs)}</font> 个',
        '',
    ]
    footer = []
    if catalog:
        footer.append(f'[查看产物目录（可按"失败"筛选）]({truncate_utf8(catalog, 768)})')

    def size(lines):
        return len('\n'.join(lines).encode('utf-8'))

    body = []
    shown = 0
    for index, job in enumerate(jobs, 1):
        entry = [f'{index}. [{truncate_utf8(link_text(job["name"]), 256)}]('
                 f'{truncate_utf8(job.get("web_url", info["web_url"]), 768)})']
        error = errors.get(job['id'])
        if error:
            entry.append(f'　`{truncate_utf8(plain(error), 256)}`')
        rest = len(jobs) - index
        more = [f'另有 {rest} 个失败任务未列出'] if rest else []
        if size(header + body + entry + more + footer) > MAX_MESSAGE_BYTES:
            entry = entry[:1]
            if size(header + body + entry + more + footer) > MAX_MESSAGE_BYTES:
                break
        body.extend(entry)
        shown = index
    if shown < len(jobs):
        body.append(f'另有 {len(jobs) - shown} 个失败任务未列出')
    if not jobs:
        # Every job passed: the failed status is GitLab's own, e.g. `pages:deploy`, which the
        # jobs API does not list (and a Guest token cannot read through commit statuses).
        body.append('所有任务都已成功，失败来自 GitLab 内部状态（通常是产物目录页面的 pages:deploy 部署失败），'
                    '请打开流水线查看。')
    message = '\n'.join(header + body + ([''] + footer if footer else []))
    if len(message.encode('utf-8')) <= MAX_MESSAGE_BYTES:
        return message
    compact = '\n'.join(header + [f'另有 {len(jobs)} 个失败任务未列出'] +
                         ([''] + footer if footer else []))
    if len(compact.encode('utf-8')) <= MAX_MESSAGE_BYTES:
        return compact
    return (f'## SDK CI 失败\n流水线 #{truncate_utf8(str(info["id"]), 128)}\n'
            f'失败任务：{len(jobs)} 个（详情已省略）')


def collect(gitlab, project, info, project_url):
    jobs = failed_jobs(gitlab, project, info['id'])
    errors = {}
    for job in jobs[:MAX_TRACE_JOBS]:
        if 'downstream_pipeline' in job:  # a bridge has no log of its own
            continue
        try:
            error = job_error(gitlab, job.get('_project', project), job)
        except ValueError as exc:
            log(f'warning: cannot read log of job {job["id"]}: {exc}')
            continue
        if error:
            errors[job['id']] = error
    try:
        catalog = catalog_url(gitlab, project, info['id'], project_url)
    except ValueError as exc:
        log(f'warning: cannot look up artifact catalog: {exc}')
        catalog = None
    return build_message(info, jobs, errors, catalog)


def send_wecom(key, content, opener=None):
    payload = json.dumps({'msgtype': 'markdown', 'markdown': {'content': content}}).encode('utf-8')
    request = Request(WECOM_SEND_URL.format(key=quote(key, safe='')), data=payload,
                      headers={'Content-Type': 'application/json'}, method='POST')
    opener = opener or build_opener()
    # Transport failures are ambiguous; the worker quarantines them instead of resending.
    try:
        with opener.open(request, timeout=15) as response:
            result = json.load(response)
    except (HTTPError, URLError, TimeoutError, OSError) as exc:
        # The key is part of the URL; report only the error type.
        raise DeliveryUncertainError(f'WeCom delivery is uncertain: {type(exc).__name__}') from None
    if result.get('errcode') != 0:
        raise ValueError(f'WeCom rejected the message: errcode={result.get("errcode")} '
                         f'{result.get("errmsg", "")}')


class State:
    """Persist delivery reservations and confirmed pipeline notifications."""

    def __init__(self, path):
        self.path = Path(path)
        try:
            data = json.loads(self.path.read_text(encoding='utf-8'))
        except FileNotFoundError:
            data = []
        except ValueError:
            # A truncated file must not keep the service in a restart loop.
            log(f'warning: state file {self.path} is corrupt; starting empty')
            data = []
        if isinstance(data, list):
            self.sent = {str(item) for item in data}
            self.pending = set()
        elif isinstance(data, dict):
            self.sent = {str(item) for item in data.get('sent', [])}
            self.pending = {str(item) for item in data.get('pending', [])} - self.sent
        else:
            self.sent = set()
            self.pending = set()

    def __contains__(self, pipeline_id):
        return self.status(pipeline_id) is not None

    def status(self, pipeline_id):
        pipeline_id = str(pipeline_id)
        if pipeline_id in self.sent:
            return 'sent'
        if pipeline_id in self.pending:
            return 'pending'
        return None

    def _write(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix('.tmp')
        data = {'sent': sorted(self.sent), 'pending': sorted(self.pending)}
        with open(tmp, 'w', encoding='utf-8') as output:
            output.write(json.dumps(data))
            output.flush()
            os.fsync(output.fileno())
        tmp.replace(self.path)

    def add(self, pipeline_id):
        pipeline_id = str(pipeline_id)
        if pipeline_id in self.sent or pipeline_id in self.pending:
            return
        self.pending.add(pipeline_id)
        try:
            self._write()
        except Exception:
            self.pending.remove(pipeline_id)
            raise

    def mark_sent(self, pipeline_id):
        pipeline_id = str(pipeline_id)
        was_pending = pipeline_id in self.pending
        was_sent = pipeline_id in self.sent
        self.pending.discard(pipeline_id)
        self.sent.add(pipeline_id)
        try:
            self._write()
        except Exception:
            if not was_sent:
                self.sent.remove(pipeline_id)
            if was_pending:
                self.pending.add(pipeline_id)
            raise

    def remove(self, pipeline_id):
        pipeline_id = str(pipeline_id)
        if pipeline_id not in self.pending:
            return
        self.pending.remove(pipeline_id)
        if not self.pending and not self.sent:
            try:
                self.path.unlink(missing_ok=True)
            except Exception:
                self.pending.add(pipeline_id)
                raise
            return
        try:
            self._write()
        except Exception:
            self.pending.add(pipeline_id)
            raise


class DurableQueue:
    """Persist webhook payloads and retry deadlines before acknowledging GitLab."""

    def __init__(self, path, maxsize=MAX_PENDING_WEBHOOKS, max_records=MAX_QUEUE_RECORDS):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.maxsize = maxsize
        self.max_records = max_records
        with self._connect() as db:
            db.execute('PRAGMA journal_mode=WAL')
            db.execute('''CREATE TABLE IF NOT EXISTS events (
                id INTEGER PRIMARY KEY,
                event_key TEXT NOT NULL UNIQUE,
                payload TEXT NOT NULL,
                attempt INTEGER NOT NULL,
                due REAL NOT NULL,
                status TEXT NOT NULL,
                claimed_at REAL
            )''')
            columns = {row[1] for row in db.execute('PRAGMA table_info(events)')}
            if 'claimed_at' not in columns:
                db.execute('ALTER TABLE events ADD COLUMN claimed_at REAL')
            recovered = db.execute(
                "UPDATE events SET status='uncertain', claimed_at=NULL "
                "WHERE status='processing' AND (claimed_at IS NULL OR claimed_at<=?)",
                (time.time() - PROCESSING_LEASE_SECONDS,)).rowcount
        if recovered:
            log(f'warning: {recovered} interrupted event(s) need delivery review; '
                'they were not retried automatically')

    def _connect(self):
        db = sqlite3.connect(self.path, timeout=30)
        db.execute('PRAGMA synchronous=FULL')
        return db

    @staticmethod
    def _event_key(payload):
        attrs = payload.get('object_attributes') or {}
        pipeline_id = attrs.get('id')
        project_id = (payload.get('project') or {}).get('id')
        if pipeline_id is not None and project_id is not None:
            return f'{project_id}:{pipeline_id}'
        raw = json.dumps(payload, sort_keys=True, separators=(',', ':')).encode('utf-8')
        return hashlib.sha256(raw).hexdigest() + ':' + uuid.uuid4().hex

    def put_nowait(self, item):
        payload, attempt = item
        encoded = json.dumps(payload, separators=(',', ':'))
        event_key = self._event_key(payload)
        with self._connect() as db:
            db.execute('BEGIN IMMEDIATE')
            if db.execute('SELECT 1 FROM events WHERE event_key=?', (event_key,)).fetchone():
                return
            pending = db.execute(
                "SELECT COUNT(*) FROM events WHERE status IN ('queued', 'processing')").fetchone()[0]
            total = db.execute('SELECT COUNT(*) FROM events').fetchone()[0]
            if pending >= self.maxsize or total >= self.max_records:
                raise queue.Full
            db.execute('''INSERT INTO events(event_key, payload, attempt, due, status)
                          VALUES (?, ?, ?, ?, 'queued')''',
                       (event_key, encoded, attempt, time.time()))

    def get(self):
        while True:
            with self._connect() as db:
                db.execute('BEGIN IMMEDIATE')
                db.execute(
                    "UPDATE events SET status='uncertain', claimed_at=NULL "
                    "WHERE status='processing' AND claimed_at<=?",
                    (time.time() - PROCESSING_LEASE_SECONDS,))
                row = db.execute(
                    "SELECT id, payload, attempt FROM events "
                    "WHERE status='queued' AND due<=? ORDER BY id LIMIT 1",
                    (time.time(),)).fetchone()
                if row:
                    db.execute("UPDATE events SET status='processing', claimed_at=? WHERE id=?",
                               (time.time(), row[0]))
                    return json.loads(row[1]), row[2], row[0]
            time.sleep(0.25)

    def complete(self, event_id):
        with self._connect() as db:
            db.execute('DELETE FROM events WHERE id=?', (event_id,))

    def retry(self, event_id, attempt, delay):
        with self._connect() as db:
            db.execute("UPDATE events SET attempt=?, due=?, status='queued', claimed_at=NULL WHERE id=?",
                       (attempt, time.time() + delay, event_id))

    def abandon(self, event_id):
        with self._connect() as db:
            db.execute("UPDATE events SET status='uncertain', claimed_at=NULL WHERE id=?",
                       (event_id,))


def should_notify(payload, refs):
    """Cheap filters on the webhook payload; returns a reason when skipping."""
    if payload.get('object_kind') != 'pipeline':
        return 'not a pipeline event'
    attrs = payload.get('object_attributes') or {}
    if attrs.get('status') != 'failed':
        return f'status {attrs.get("status")}'
    if attrs.get('source') not in NOTIFY_SOURCES:
        return f'source {attrs.get("source")}'
    if attrs.get('tag') or attrs.get('ref') not in refs:
        return f'ref {attrs.get("ref")}'
    variables = {item.get('key'): item.get('value') for item in attrs.get('variables') or []
                 if isinstance(item, dict)}
    if variables.get('CATALOG_SMOKE_TEST') == '1':
        return 'catalog smoke test'
    return None


class Notifier:
    def __init__(self, gitlab, wecom_key, refs, state, send=send_wecom):
        self.gitlab = gitlab
        self.wecom_key = wecom_key
        self.refs = refs
        self.state = state
        self.send = send
        self.lock = threading.Lock()

    def handle(self, payload):
        """Process one webhook payload; returns True when a message was sent."""
        reason = should_notify(payload, self.refs)
        attrs = payload.get('object_attributes') or {}
        pipeline_id = attrs.get('id')
        if reason:
            log(f'skip pipeline {pipeline_id}: {reason}')
            return False
        with self.lock:
            state = self.state.status(pipeline_id)
            if state == 'sent':
                log(f'skip pipeline {pipeline_id}: already notified')
                return False
            if state == 'pending':
                raise DeliveryUncertainError(
                    f'pipeline {pipeline_id} has an unresolved delivery; check WeCom before retrying')
            project = payload['project']['id']
            # A retried event can be minutes old; someone may have rerun the jobs since.
            current = self.gitlab.get(project, f'pipelines/{pipeline_id}').get('status')
            if current != 'failed':
                log(f'skip pipeline {pipeline_id}: now {current}')
                return False
            if not is_full_build(self.gitlab, project, pipeline_id):
                log(f'skip pipeline {pipeline_id}: not a full SDK build')
                return False
            info = {
                'id': pipeline_id, 'ref': attrs['ref'], 'sha': attrs['sha'],
                'source': attrs['source'],
                'web_url': attrs.get('url') or f'{payload["project"]["web_url"]}/-/pipelines/{pipeline_id}',
                'commit_title': (payload.get('commit') or {}).get('title'),
            }
            message = collect(self.gitlab, project, info, payload['project']['web_url'])
            self.state.add(pipeline_id)
            try:
                self.send(self.wecom_key, message)
            except DeliveryUncertainError:
                raise
            except Exception:
                self.state.remove(pipeline_id)
                raise
            try:
                self.state.mark_sent(pipeline_id)
            except OSError as exc:
                raise DeliveryUncertainError(
                    f'WeCom accepted pipeline {pipeline_id}, but delivery state could not be saved') from exc
            log(f'notified pipeline {pipeline_id}')
            return True


def make_handler(secret, jobs):
    class Handler(BaseHTTPRequestHandler):
        timeout = REQUEST_TIMEOUT

        def reply(self, code, text):
            body = text.encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', 'text/plain; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            if self.path.rstrip('/') != '/gitlab':
                return self.reply(404, 'not found')
            token = self.headers.get('X-Gitlab-Token', '')
            if not hmac.compare_digest(token.encode('utf-8'), secret.encode('utf-8')):
                return self.reply(401, 'bad token')
            try:
                length = int(self.headers.get('Content-Length', '0'))
            except ValueError:
                return self.reply(400, 'bad length')
            if length <= 0 or length > MAX_BODY_BYTES:
                return self.reply(413, 'bad size')
            try:
                payload = json.loads(self.rfile.read(length))
            except ValueError:
                return self.reply(400, 'bad json')
            if not isinstance(payload, dict):
                return self.reply(400, 'bad json')
            # Persist before acknowledging; GitLab may not redeliver a 200 response.
            try:
                jobs.put_nowait((payload, 0))
            except queue.Full:
                return self.reply(503, 'queue full; retry later')
            except (sqlite3.Error, OSError) as exc:
                log(f'error: cannot persist webhook event: {type(exc).__name__}')
                return self.reply(503, 'queue unavailable; retry later')
            return self.reply(200, 'queued')

        def do_GET(self):
            return self.reply(200, 'ok') if self.path == '/health' else self.reply(404, 'not found')

        def log_message(self, format, *args):
            log(f'{self.address_string()} {format % args}')

    return Handler


def make_server(host, port, secret, jobs):
    """One thread per request, so a slow or idle client cannot stall GitLab's deliveries."""
    server = ThreadingHTTPServer((host, port), make_handler(secret, jobs))
    server.daemon_threads = True
    return server


def worker(notifier, jobs, delays=RETRY_DELAYS, timer=threading.Timer):
    """Handle queued `(payload, attempt)` items forever, retrying failures with backoff."""
    while True:
        item = jobs.get()
        if item is None:
            return
        payload, attempt = item[:2]
        event_id = item[2] if len(item) == 3 else None
        succeeded = False
        try:
            notifier.handle(payload)
            succeeded = True
        except Exception as exc:  # keep serving; one broken event must not stop the service
            pipeline_id = (payload.get('object_attributes') or {}).get('id')
            if isinstance(exc, DeliveryUncertainError):
                log(f'uncertain delivery for pipeline {pipeline_id}: {exc}; '
                    'check WeCom before manual recovery')
                if event_id is not None:
                    jobs.abandon(event_id)
            elif attempt < len(delays):
                log(f'error on pipeline {pipeline_id}: {type(exc).__name__}: {exc}; '
                    f'retry {attempt + 1}/{len(delays)} in {delays[attempt]}s')
                if event_id is not None:
                    jobs.retry(event_id, attempt + 1, delays[attempt])
                else:
                    retry = timer(delays[attempt], jobs.put, args=((payload, attempt + 1),))
                    retry.daemon = True
                    retry.start()
            else:
                log(f'error on pipeline {pipeline_id}: {type(exc).__name__}: {exc}; giving up, '
                    f'resend with: notify --project <id> --pipeline {pipeline_id}')
                if event_id is not None:
                    jobs.abandon(event_id)
        finally:
            if event_id is None:
                jobs.task_done()
        if event_id is not None:
            if succeeded:
                jobs.complete(event_id)


def env(name, required=True, default=None):
    value = os.environ.get(name, default)
    if required and not value:
        sys.exit(f'environment variable {name} is required')
    return value


def pipeline_info(gitlab, project, pipeline_id):
    pipeline = gitlab.get(project, f'pipelines/{pipeline_id}')
    try:
        title = gitlab.get(project, f'repository/commits/{pipeline["sha"]}').get('title')
    except GitLabError as exc:
        # A Guest token can read pipelines and job logs but not the repository.
        if exc.code not in (403, 404):
            raise
        title = None
    return {'id': pipeline['id'], 'ref': pipeline['ref'], 'sha': pipeline['sha'],
            'source': pipeline['source'], 'web_url': pipeline['web_url'], 'commit_title': title}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest='command', required=True)
    serve = sub.add_parser('serve', help='receive GitLab Pipeline webhooks')
    serve.add_argument('--host', default='0.0.0.0')
    serve.add_argument('--port', type=int, default=8099)
    sub.add_parser('test-message', help='send a test message to the WeCom group')
    notify = sub.add_parser('notify', help='render (and send) the message of one pipeline')
    notify.add_argument('--project', required=True, help='project ID or path, e.g. 38')
    notify.add_argument('--pipeline', required=True, type=int)
    notify.add_argument('--dry-run', action='store_true', help='print instead of sending')
    args = parser.parse_args(argv)

    if args.command == 'test-message':
        send_wecom(env('WECOM_BOT_KEY'), '## SDK CI 通知测试\n> 能看到这条消息，说明服务器可以访问企业微信。')
        print('sent')
        return 0

    gitlab = GitLab(env('GITLAB_URL'), env('GITLAB_TOKEN'))
    if args.command == 'notify':
        info = pipeline_info(gitlab, args.project, args.pipeline)
        # An under-privileged token sees an empty bridge list instead of a 403.
        full = is_full_build(gitlab, args.project, args.pipeline)
        print(f'full SDK build (has {FULL_BUILD_BRIDGE}): {"yes" if full else "NO"}', file=sys.stderr)
        project_url = info['web_url'].split('/-/pipelines/')[0]
        message = collect(gitlab, args.project, info, project_url)
        if args.dry_run:
            print(message)
        else:
            state_file = env('NOTIFY_STATE_FILE', False,
                            str(Path.home() / '.ci_failure_notify' / 'notified.json'))
            state = State(state_file)
            state.add(args.pipeline)
            try:
                send_wecom(env('WECOM_BOT_KEY'), message)
            except DeliveryUncertainError:
                raise
            except Exception:
                state.remove(args.pipeline)
                raise
            state.mark_sent(args.pipeline)
            print('sent')
        return 0

    refs = {ref.strip() for ref in env('NOTIFY_REFS', False, DEFAULT_REFS).split(',') if ref.strip()}
    state_file = env('NOTIFY_STATE_FILE', False,
                     str(Path.home() / '.ci_failure_notify' / 'notified.json'))
    state = State(state_file)
    notifier = Notifier(gitlab, env('WECOM_BOT_KEY'), refs, state)
    queue_file = env('NOTIFY_QUEUE_FILE', False,
                     str(Path(state_file).with_name('pending.sqlite3')))
    jobs = DurableQueue(queue_file)
    threading.Thread(target=worker, args=(notifier, jobs), daemon=True).start()
    server = make_server(args.host, args.port, env('WEBHOOK_SECRET'), jobs)
    log(f'listening on {args.host}:{args.port}, refs: {", ".join(sorted(refs))}')
    server.serve_forever()
    return 0


if __name__ == '__main__':
    sys.exit(main())
