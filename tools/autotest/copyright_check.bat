@echo off
setlocal EnableDelayedExpansion

:: =============================================================================
:: Copyright Check Script
::
:: This script performs the following actions:
:: 1. Identifies files changed in the last GIT commit.
:: 2. Copies them to a temporary directory for scanning.
:: 3. Runs Scancode toolkit to analyze licenses.
:: 4. Runs a Python script to validate the licenses against a configuration.
:: =============================================================================

:: -----------------------------------------------------------------------------
:: Configuration
:: -----------------------------------------------------------------------------
set "ROOT=%CD%"
set "SCAN_SOURCE_DIR=scancode_dir"
set "ARTIFACTS_DIR=artifacts"
set "ENV_ROOT=C:\SiFli_Env"
set "LICENSE_CONFIG=tools\scancode\license_config.yml"
set "LICENSE_CHECK_SCRIPT=tools\scancode\license_check.py"
set "SCANCODE_REPORT_JSON=%ARTIFACTS_DIR%\scancode.json"
set "SCANCODE_REPORT_HTML=%ARTIFACTS_DIR%\scancode.html"
set "CHECK_REPORT_TXT=%ARTIFACTS_DIR%\report.txt"


:: -----------------------------------------------------------------------------
:: STAGE 1: Setup Environment and Directories
:: -----------------------------------------------------------------------------
echo [INFO] Setting up environment and directories...

:: Initialize ConEmu environment if available
if exist "%ENV_ROOT%\tools\ConEmu\ConEmu\CmdInit.cmd" (
    call "%ENV_ROOT%\tools\ConEmu\ConEmu\CmdInit.cmd"
)
call set_env.bat
:: Clean and create directories
if exist "%SCAN_SOURCE_DIR%" (
    echo [INFO] Cleaning up previous scan directory...
    rmdir /s /q "%SCAN_SOURCE_DIR%"
)
mkdir "%SCAN_SOURCE_DIR%"

if not exist "%ARTIFACTS_DIR%" (
    mkdir "%ARTIFACTS_DIR%"
)
echo [INFO] Directories are ready.


:: -----------------------------------------------------------------------------
:: STAGE 2: Copy Changed Files
:: -----------------------------------------------------------------------------
echo [INFO] Identifying and copying changed files from last commit...
::chcp 65001 >nul
set "STAGE2_SCRIPT=copy_file_stage.py"
echo #!/usr/bin/env python3> "%STAGE2_SCRIPT%"
echo.>> "%STAGE2_SCRIPT%"
echo from __future__ import annotations>> "%STAGE2_SCRIPT%"
echo import argparse>> "%STAGE2_SCRIPT%"
echo import shutil>> "%STAGE2_SCRIPT%"
echo import subprocess>> "%STAGE2_SCRIPT%"
echo from pathlib import Path>> "%STAGE2_SCRIPT%"
echo.>> "%STAGE2_SCRIPT%"
echo def info(message: str^) -^> None:>> "%STAGE2_SCRIPT%"
echo     print(f"[INFO] {message}"^)>> "%STAGE2_SCRIPT%"
echo.>> "%STAGE2_SCRIPT%"
echo def copy_changed_files(root: Path, scan_source_dir: Path^) -^> None:>> "%STAGE2_SCRIPT%"
echo     git_command = [>> "%STAGE2_SCRIPT%"
echo         "C:\\Program Files\\Git\\bin\\git.exe",>> "%STAGE2_SCRIPT%"
echo         "-c", "core.quotepath=false",>> "%STAGE2_SCRIPT%"
echo         "-c", "i18n.commitencoding=utf-8",>> "%STAGE2_SCRIPT%"
echo         "-c", "i18n.logoutputencoding=utf-8",>> "%STAGE2_SCRIPT%"
echo         "diff", "--name-only", "--diff-filter=AM", "HEAD~1", "HEAD",>> "%STAGE2_SCRIPT%"
echo     ]>> "%STAGE2_SCRIPT%"
echo     result = subprocess.run(git_command, cwd=str(root^), check=True, capture_output=True, text=True, encoding="utf-8", errors="replace"^)>> "%STAGE2_SCRIPT%"
echo     for relative_name in result.stdout.splitlines(^):>> "%STAGE2_SCRIPT%"
echo         if not relative_name.strip(^):>> "%STAGE2_SCRIPT%"
echo             continue>> "%STAGE2_SCRIPT%"
echo         source_path = root / relative_name>> "%STAGE2_SCRIPT%"
echo         if source_path.is_dir(^):>> "%STAGE2_SCRIPT%"
echo             info(f"Skipping submodule \"{relative_name}\""^)>> "%STAGE2_SCRIPT%"
echo             continue>> "%STAGE2_SCRIPT%"
echo         destination_path = scan_source_dir / relative_name>> "%STAGE2_SCRIPT%"
echo         destination_path.parent.mkdir(parents=True, exist_ok=True^)>> "%STAGE2_SCRIPT%"
echo         info(f"Copying \"{relative_name}\""^)>> "%STAGE2_SCRIPT%"
echo         shutil.copy2(source_path, destination_path^)>> "%STAGE2_SCRIPT%"
echo.>> "%STAGE2_SCRIPT%"
echo def main(^) -^> int:>> "%STAGE2_SCRIPT%"
echo     parser = argparse.ArgumentParser(description="Copy changed files from the last commit into a staging directory."^)>> "%STAGE2_SCRIPT%"
echo     parser.add_argument("--root", default=str(Path.cwd(^)^), help="Repository root directory."^)>> "%STAGE2_SCRIPT%"
echo     parser.add_argument("--scan-source-dir", default="scancode_dir", help="Directory used to stage copied files."^)>> "%STAGE2_SCRIPT%"
echo     args = parser.parse_args(^)>> "%STAGE2_SCRIPT%"
echo     root = Path(args.root^).resolve(^)>> "%STAGE2_SCRIPT%"
echo     scan_source_dir = (root / args.scan_source_dir^).resolve(^)>> "%STAGE2_SCRIPT%"
echo     copy_changed_files(root, scan_source_dir^)>> "%STAGE2_SCRIPT%"
echo     info("File copy complete."^)>> "%STAGE2_SCRIPT%"
echo     ^return ^0>> "%STAGE2_SCRIPT%"
echo.>> "%STAGE2_SCRIPT%"
echo if __name__ == "__main__":>> "%STAGE2_SCRIPT%"
echo     raise SystemExit(main(^)^)>> "%STAGE2_SCRIPT%"

python "%STAGE2_SCRIPT%" --root "%ROOT%" --scan-source-dir "%SCAN_SOURCE_DIR%"
if errorlevel 1 (
    echo [ERROR] Failed to copy changed files. Aborting.
    exit /b 1
)
del "%STAGE2_SCRIPT%" >nul 2>nul

:: -----------------------------------------------------------------------------
:: STAGE 3: Run Scancode Analysis
:: -----------------------------------------------------------------------------
echo [INFO] Running Scancode analysis. This may take a while...

:: Setup python environment
call set_env.bat

scancode -clipeu ^
  --license --license-text --license-text-diagnostics ^
  --classify ^
  --summary ^
  --verbose "%SCAN_SOURCE_DIR%" ^
  --processes 2 ^
  --json "%SCANCODE_REPORT_JSON%" ^
  --html "%SCANCODE_REPORT_HTML%"

if not "%errorlevel%"=="0" (
    echo [ERROR] Scancode analysis failed with error code %errorlevel%.
    exit /b 1
)
echo [INFO] Scancode analysis successful.


:: -----------------------------------------------------------------------------
:: STAGE 4: Validate Licenses
:: -----------------------------------------------------------------------------
echo [INFO] Validating licenses...

python %LICENSE_CHECK_SCRIPT% ^
    -c "%LICENSE_CONFIG%" ^
    -s "%SCANCODE_REPORT_JSON%" ^
    -f "%SCAN_SOURCE_DIR%" ^
    -o "%CHECK_REPORT_TXT%"

if not "%errorlevel%"=="0" (
    echo [ERROR] License check failed. See report below.
    echo.
    echo ======================= FAILED FILES =======================
    type "%CHECK_REPORT_TXT%"
    echo ============================================================
    echo.
    exit /b 1
)
echo [INFO] License validation successful.


:: -----------------------------------------------------------------------------
:: SUCCESS
:: -----------------------------------------------------------------------------
echo.
echo [SUCCESS] All checks passed.
exit /b 0
