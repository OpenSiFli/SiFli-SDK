#!/usr/bin/env bash

set -e

basedir=$(dirname "$0")
SIFLI_SDK_PATH=$(cd "${basedir}"; pwd -P)
export SIFLI_SDK_PATH

mirror_china_normalized=$(printf '%s' "${SIFLI_SDK_MIRROR_CHINA:-}" | tr '[:upper:]' '[:lower:]')
case "${mirror_china_normalized}" in
    1|true|yes|on)
        export SIFLI_SDK_GITHUB_ASSETS="https://downloads.sifli.com/github_assets"
        export SIFLI_SDK_PYPI_DEFAULT_INDEX="https://mirrors.ustc.edu.cn/pypi/simple"
        export UV_PYTHON_DOWNLOADS_JSON_URL="https://uv.agentsmirror.com/metadata/python-downloads.json"
        export UV_PYPY_INSTALL_MIRROR="https://uv.agentsmirror.com/pypy"
        ;;
esac

if ! command -v uv >/dev/null 2>&1; then
    echo "ERROR: uv was not found in PATH. Please install uv before running install.sh." >&2
    exit 1
fi

uv run --with rich --with tomli_w --python 3.13.11 --no-project "${SIFLI_SDK_PATH}/tools/sdk_env.py" install "$@"
