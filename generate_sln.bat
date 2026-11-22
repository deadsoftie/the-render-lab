@echo off
setlocal
cd /d "%~dp0"

set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
set "GENERATOR="
set "BUILD_DIR="

if exist "%VSWHERE%" (
    for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -version 18.0 -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do (
        set "GENERATOR=Visual Studio 18 2026"
        set "BUILD_DIR=out/build/vs2026"
    )
    if not defined GENERATOR (
        for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -version 17.0 -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do (
            set "GENERATOR=Visual Studio 17 2022"
            set "BUILD_DIR=out/build/vs2022"
        )
    )
)

if not defined GENERATOR (
    echo No Visual Studio 2026 or 2022 install with the C++ workload was found.
    pause
    exit /b 1
)

echo Generating %GENERATOR% solution in %BUILD_DIR%...
cmake -B %BUILD_DIR% -G "%GENERATOR%" -A x64 ^
    -DCMAKE_TOOLCHAIN_FILE=vcpkg/scripts/buildsystems/vcpkg.cmake ^
    -DVCPKG_TARGET_TRIPLET=x64-windows

if errorlevel 1 (
    echo.
    echo Generation FAILED - see errors above.
    pause
    exit /b 1
)

echo.
if exist "%BUILD_DIR%\the-render-lab.slnx" (
    echo Done. Open %BUILD_DIR%\the-render-lab.slnx in Visual Studio.
) else (
    echo Done. Open %BUILD_DIR%\the-render-lab.sln in Visual Studio.
)
pause
