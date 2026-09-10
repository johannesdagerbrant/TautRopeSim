@echo off
rem Build the standalone targets. vcvars64.bat costs ~1.5 s of a ~2 s loop, so its
rem environment is cached on first use and replayed from a file afterwards.
setlocal enabledelayedexpansion
set "ENVFILE=%~dp0msvcenv.txt"
if not exist "%ENVFILE%" (
    for /d %%V in ("C:\Program Files (x86)\Microsoft Visual Studio\*") do (
        if exist "%%V\BuildTools\VC\Auxiliary\Build\vcvars64.bat" set "VCVARS=%%V\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
    )
    if not defined VCVARS (
        echo b.bat: could not find vcvars64.bat under Visual Studio Build Tools 1>&2
        exit /b 1
    )
    echo b.bat: caching MSVC environment ^(first run only^)
    call "!VCVARS!" >nul || exit /b 1
    set > "%ENVFILE%"
)
for /f "usebackq delims=" %%L in ("%ENVFILE%") do set "%%L"
cd /d "%~dp0"
if not exist build\build.ninja cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo || exit /b 1
ninja -C build %*
