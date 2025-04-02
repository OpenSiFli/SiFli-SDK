#!/usr/bin/env pwsh

param(
    [Switch]$h
)

$SIFLI_SDK_PATH = $PSScriptRoot

if ($h) {
    python "$SIFLI_SDK_PATH/tools/install_util.py" print_help ps1
    Exit
}

Write-Output "INFO: Using SIFLI_SDK_PATH '$SIFLI_SDK_PATH' for installation."

$FEATURES = (python "$SIFLI_SDK_PATH/tools/install_util.py" extract features "$args")

Write-Output "Setting up Python environment"
# PowerShell does not propagate variables to new process (spawned python), so we pass detected SIFLI_SDK_PATH as argument
# to avoid using any previously set SIFLI_SDK_PATH values in the terminal environment.
Start-Process -Wait -PassThru -NoNewWindow -FilePath "python" -Args @("-m", "pip", "install", "requests")
$proces_py_env = Start-Process -Wait -PassThru -NoNewWindow -FilePath "python" -Args "`"$SIFLI_SDK_PATH/tools/sifli_sdk_tools.py`" --sifli-sdk-path ${SIFLI_SDK_PATH} install-python-env --features=${FEATURES}"
$exit_code_py_env = $proces_py_env.ExitCode
if ($exit_code_py_env -ne 0) { exit $exit_code_py_env } # if error

$python_venv_path = (python "$SIFLI_SDK_PATH/tools/sifli_sdk_tools.py" get-install-python-env)
if ($python_venv_path.StartsWith("Error")) {
    Write-Host $python_venv_path
    exit 1
}

$python_venv = "$python_venv_path/Scripts/python"
$TARGETS = & $python_venv "$SIFLI_SDK_PATH/tools/install_util.py" extract targets 

Write-Output "Installing SIFLI-SDK tools"
# PowerShell does not propagate variables to new process (spawned python), so we pass detected SIFLI_SDK_PATH as argument
# to avoid using any previously set SIFLI_SDK_PATH values in the terminal environment.
$proces_tools = Start-Process -Wait -PassThru -NoNewWindow -FilePath "$python_venv" -Args "`"$SIFLI_SDK_PATH/tools/sifli_sdk_tools.py`" --sifli-sdk-path ${SIFLI_SDK_PATH} install --targets=${TARGETS}"
$exit_code_tools = $proces_tools.ExitCode
if ($exit_code_tools -ne 0) { exit $exit_code_tools }  # if error

Write-Output "
All done! You can now run:
    export.ps1
"