@echo off
setlocal

echo Updating git submodules...

REM Change directory to where this .bat file lives
cd /d "%~dp0"

REM Safety check: are we in a git repo?
if not exist ".git" (
    echo ERROR: This is not a git repository.
    pause
    exit /b 1
)

git submodule update --init --recursive
echo.
echo Submodules updated successfully.

pause