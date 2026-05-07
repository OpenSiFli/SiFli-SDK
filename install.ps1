#!/usr/bin/env pwsh

$SIFLI_SDK_PATH = $PSScriptRoot
$env:SIFLI_SDK_PATH = $SIFLI_SDK_PATH

$mirrorChina = "$env:SIFLI_SDK_MIRROR_CHINA".Trim().ToLowerInvariant()
if ($mirrorChina -in @("1", "true", "yes", "on")) {
    $env:SIFLI_SDK_GITHUB_ASSETS = "https://downloads.sifli.com/github_assets"
    $env:SIFLI_SDK_PYPI_DEFAULT_INDEX = "https://mirrors.ustc.edu.cn/pypi/simple"
    $env:UV_PYTHON_DOWNLOADS_JSON_URL = "https://uv.agentsmirror.com/metadata/python-downloads.json"
    $env:UV_PYPY_INSTALL_MIRROR = "https://uv.agentsmirror.com/pypy"
}

if (-not (Get-Command uv -ErrorAction SilentlyContinue)) {
    Write-Error "uv was not found in PATH. Please install uv before running install.ps1."
    exit 1
}

& uv run --with rich --with tomli_w --python 3.13.11 --no-project "$SIFLI_SDK_PATH/tools/sdk_env.py" install @args
exit $LASTEXITCODE
