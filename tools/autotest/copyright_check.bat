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
chcp 65001 >nul
git config --local core.quotepath false
git config --local i18n.commitencoding utf-8
git config --local i18n.logoutputencoding utf-8
for /f "usebackq delims=" %%F in (`
    git diff --name-only --diff-filter=AM HEAD~1 HEAD
`) do (
    if exist "%%F\" (
        echo   - Skipping submodule "%%F"
    ) else (
        echo   - Copying "%%F"
        for %%A in ("%%F") do set "file=%%~fA"
        for %%A in ("%%F") do set "ABS=%%~dpA"
        set "REL=!ABS:%ROOT%\=!"
        if not exist "%SCAN_SOURCE_DIR%\!REL!" (
            mkdir "%SCAN_SOURCE_DIR%\!REL!"
        )
        copy "!file!" "%SCAN_SOURCE_DIR%\!REL!" >nul
        if errorlevel 1 (
            echo [ERROR] Failed to copy "!file!". Aborting.
            exit /b 1
        )
    )
)
echo [INFO] File copy complete.

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
