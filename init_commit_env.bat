@echo off
color 0a
:start
cls
echo,
echo     Initial Commit Env
echo,
echo   1 Gitlab Repo Url
echo,
echo   2 Gerrit Repo Url
echo,
echo   3 Submodule Gerrit Repo Url
echo,
echo   0 Cancel
echo,
echo,
setlocal enabledelayedexpansion
set /p line=<.git
::echo line is %line%
if "%line%"=="" (
    set "dst=.\.git"
) else (
    if "%line%"=="%line:modules= %" (
        set "line=%line:worktrees= %"
    ) 
    for /f  "tokens=2 delims= " %%a in ("!line!") do (
        set "dst=%%a"
    )
    set "dst=!dst:/=\!"
   
)
::echo %dst%
choice /c:1230 /n /m:"please enter an option:"
if %errorlevel%==0 exit
if %errorlevel%==1 goto gitlab
if %errorlevel%==2 goto gerrit
if %errorlevel%==3 goto submodule
exit


:gitlab
copy tools\autotest\commitEnvScripts\pre-commit %dst%\hooks\>nul  /Y
git config commit.template .commit_template
exit

:gerrit
copy tools\autotest\commitEnvScripts\commit-msg %dst%\hooks\>nul  /Y
copy tools\autotest\commitEnvScripts\pre-commit %dst%\hooks\>nul  /Y
git config commit.template .commit_template
exit

:submodule
copy tools\autotest\commitEnvScripts\commit-msg %dst%\hooks\>nul  /Y
copy tools\autotest\commitEnvScripts\pre-commit %dst%\hooks\>nul  /Y
git config commit.template .commit_template
exit


