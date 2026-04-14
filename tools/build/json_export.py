# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

import datetime
import json
import os

import utils


def _dedupe_keep_order(items):
    result = []
    seen = set()

    for item in items:
        if not item:
            continue
        if item in seen:
            continue
        seen.add(item)
        result.append(item)

    return result


def _normalize_path(path):
    if path is None:
        return None

    value = str(path).strip()
    if not value:
        return None

    if value[0] == '"' and value.endswith('",'):
        value = value[1:-2]
    elif value[0] == '"' and value.endswith('"'):
        value = value[1:-1]

    return os.path.abspath(value).replace('\\', '/')


def _iter_values(value):
    if isinstance(value, list):
        for item in value:
            yield from _iter_values(item)
        return

    if isinstance(value, tuple):
        if len(value) == 2 and not isinstance(value[0], (list, tuple, dict)):
            yield value
            return
        for item in value:
            yield from _iter_values(item)
        return

    yield value


def _normalize_define(item):
    if isinstance(item, tuple):
        key = str(item[0])
        value = item[1]
        if value is None:
            return key
        return '{}={}'.format(key, value)

    return str(item)


def _collect_sources(project):
    sources = []

    for group in project:
        for source in group.get('src', []):
            sources.append(_normalize_path(source))

    return _dedupe_keep_order(sources)


def _collect_headers(env):
    headers = utils.TargetGetList(env, ['h'])
    return _dedupe_keep_order([_normalize_path(header) for header in headers])


def _collect_include_paths(project, env):
    paths = []

    for item in _iter_values(env.get('CPPPATH', [])):
        paths.append(_normalize_path(item))

    for group in project:
        for key in ('CPPPATH', 'LOCAL_CPPPATH'):
            for item in _iter_values(group.get(key, [])):
                paths.append(_normalize_path(item))

    return _dedupe_keep_order(paths)


def _collect_defines(project, env):
    defines = []

    for item in _iter_values(env.get('CPPDEFINES', [])):
        defines.append(_normalize_define(item))

    for group in project:
        for key in ('CPPDEFINES', 'LOCAL_CPPDEFINES'):
            for item in _iter_values(group.get(key, [])):
                defines.append(_normalize_define(item))

    return _dedupe_keep_order(defines)


def _serialize_project(env):
    project = env['project']
    sources = _collect_sources(project)
    headers = _collect_headers(env)
    include_paths = _collect_include_paths(project, env)
    defines = _collect_defines(project, env)

    return {
        'name': env.get('name', ''),
        'full_name': env.get('full_name', ''),
        'parent': env.get('parent', ''),
        'bsp_root': _normalize_path(env.get('BSP_ROOT')),
        'build_dir': _normalize_path(env.get('build_dir')),
        'sources': sources,
        'headers': headers,
        'files': _dedupe_keep_order(sources + headers),
        'include_paths': include_paths,
        'defines': defines,
    }


def TargetJSON(main_env, env_list=None):
    if env_list is None:
        env_list = [main_env]

    projects = []
    all_sources = []
    all_headers = []
    all_files = []
    all_include_paths = []
    all_defines = []

    for env in env_list:
        if 'project' not in env or 'target' not in env:
            continue

        project = _serialize_project(env)
        projects.append(project)
        all_sources.extend(project['sources'])
        all_headers.extend(project['headers'])
        all_files.extend(project['files'])
        all_include_paths.extend(project['include_paths'])
        all_defines.extend(project['defines'])

    output_dir = _normalize_path(main_env['build_dir'])
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, 'codebase_index.json')

    payload = {
        'format_version': 1,
        'system_construction': 'scons',
        'generated_at_utc': datetime.datetime.utcnow().replace(microsecond=0).isoformat() + 'Z',
        'main_project': main_env.get('full_name', ''),
        'projects': projects,
        'all_sources': _dedupe_keep_order(all_sources),
        'all_headers': _dedupe_keep_order(all_headers),
        'all_files': _dedupe_keep_order(all_files),
        'all_include_paths': _dedupe_keep_order(all_include_paths),
        'all_defines': _dedupe_keep_order(all_defines),
    }

    with open(output_path, 'w', encoding='utf-8') as output_file:
        json.dump(payload, output_file, indent=2, ensure_ascii=False)
        output_file.write('\n')

    print('Project file index written to {}'.format(output_path))
