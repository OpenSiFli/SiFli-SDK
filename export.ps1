#!/usr/bin/env pwsh

# Emergency backup option to use previous export.ps1 (export_legacy.ps1) if the new export approach fails.
# To use it, set environmental variable like: $Env:SIFLI_SDK_LEGACY_EXPORT=1
if ($env:SIFLI_SDK_LEGACY_EXPORT) {
    . ./tools/legacy_exports/export_legacy.ps1
    exit $LASTEXITCODE
}

$sifli_sdk_path = "$PSScriptRoot"

if (-not (Test-Path "$sifli_sdk_path/tools/sifli_sdk_tools.py") -or
    -not (Test-Path "$sifli_sdk_path/tools/activate.py")) {

    Write-Output "Could not detect SIFLI_SDK_PATH. Please navigate to your SiFli-SDK directory and run:"
    Write-Output ".\export.ps1"

    $env:SIFLI_SDK_PATH = ""

    exit 1
}

$sdk_exports = python "$sifli_sdk_path/tools/activate.py" --export
# The dot sourcing is added here in PowerShell since
# Win PSAnalyzer complains about using `Invoke-Expression` command
. $sdk_exports

$env:RTT_EXEC_PATH = $env:PATH