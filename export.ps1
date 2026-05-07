#!/usr/bin/env pwsh

$sifli_sdk_path = "$PSScriptRoot"
$env:SIFLI_SDK_PATH = $sifli_sdk_path

function Show-ExportHelp {
    Write-Output "usage: .\export.ps1 [--profile PROFILE] [-t TOOLCHAIN]"
    Write-Output ""
    Write-Output "Activate the installed SiFli-SDK environment in the current shell."
    Write-Output ""
    Write-Output "options:"
    Write-Output "  --profile PROFILE      profile to export, defaults to `"default`""
    Write-Output "  -t, --toolchain TOOLCHAIN"
    Write-Output "                         toolchain to export: gcc (default) or keil (Windows only)"
    Write-Output "  -h, --help             show this help message and exit"
}

$profile = "default"
for ($i = 0; $i -lt $args.Count; $i++) {
    $arg = [string]$args[$i]
    if ($arg -eq "-h" -or $arg -eq "--help") {
        Show-ExportHelp
        exit 0
    }
    if ($arg -eq "--profile") {
        if ($i + 1 -ge $args.Count) {
            Write-Error "--profile requires a value."
            exit 1
        }
        $profile = [string]$args[$i + 1]
        $i++
        continue
    }
    if ($arg.StartsWith("--profile=")) {
        $profile = $arg.Substring("--profile=".Length)
        continue
    }
    if ($arg -eq "-t" -or $arg -eq "--toolchain") {
        if ($i + 1 -ge $args.Count) {
            Write-Error "$arg requires a value."
            exit 1
        }
        $i++
        continue
    }
    if ($arg.StartsWith("--toolchain=")) {
        continue
    }
}

if ("$env:SIFLI_SDK_MIRROR_CHINA".Trim().ToLowerInvariant() -in @("1", "true", "yes", "on")) {
    $env:SIFLI_SDK_GITHUB_ASSETS = "https://downloads.sifli.com/github_assets"
    $env:SIFLI_SDK_PYPI_DEFAULT_INDEX = "https://mirrors.ustc.edu.cn/pypi/simple"
    $env:UV_PYTHON_DOWNLOADS_JSON_URL = "https://uv.agentsmirror.com/metadata/python-downloads.json"
    $env:UV_PYPY_INSTALL_MIRROR = "https://uv.agentsmirror.com/pypy"
}

if (-not (Get-Command uv -ErrorAction SilentlyContinue)) {
    Write-Error "uv was not found in PATH. Please install uv before running export.ps1."
    exit 1
}

$output = & uv run --with rich --with tomli_w --python 3.13.11 --no-project "$sifli_sdk_path/tools/sdk_env.py" export --shell powershell @args
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

$scriptPath = $output | Select-Object -Last 1
if (-not $scriptPath -or -not (Test-Path $scriptPath)) {
    Write-Error "export helper did not return a valid script path."
    exit 1
}

. $scriptPath
