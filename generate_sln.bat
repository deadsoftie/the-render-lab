@echo off
cd /d "%~dp0"

echo Generating Visual Studio solution...
cmake -B out/build/vs2022 -G "Visual Studio 17 2022" -A x64 ^
    -DCMAKE_TOOLCHAIN_FILE=vcpkg/scripts/buildsystems/vcpkg.cmake ^
    -DVCPKG_TARGET_TRIPLET=x64-windows

if errorlevel 1 (
    echo.
    echo Generation FAILED - see errors above.
    pause
    exit /b 1
)

echo.
echo Done. Open out\build\vs2022\the-render-lab.sln in Visual Studio.
pause
