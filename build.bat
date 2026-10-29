@echo off
REM ============================================================
REM  Flix STM32 firmware build script
REM  Usage:
REM    build.bat           build (default)
REM    build.bat clean     clean + rebuild
REM    build.bat upload    build + flash via ST-Link
REM    build.bat monitor   open serial monitor (115200)
REM ============================================================

setlocal

set PLATFORMIO_CORE_DIR=%~dp0.pio_core
cd /d %~dp0

set ACTION=%1
if "%ACTION%"=="" set ACTION=build

if "%ACTION%"=="clean" (
    echo.
    echo ===== Clean =====
    python -m platformio run -t clean
)

if "%ACTION%"=="monitor" (
    python -m platformio device monitor -b 115200
    goto :eof
)

echo.
echo ===== Build =====
python -m platformio run
set RC=%ERRORLEVEL%

if %RC% NEQ 0 (
    echo.
    echo ===== Build FAILED, exit code %RC% =====
    pause
    exit /b %RC%
)

echo.
echo ===== Build SUCCESS =====
echo Firmware: .pio\build\stm32f407_vet6\firmware.bin

if "%ACTION%"=="upload" (
    echo.
    echo ===== Upload via ST-Link =====
    python -m platformio run -t upload
    set RC=%ERRORLEVEL%
    if %RC% NEQ 0 (
        echo Upload FAILED, exit code %RC%
        pause
        exit /b %RC%
    )
    echo Upload done
) else (
    echo.
    echo Next steps:
    echo   flash:   build.bat upload
    echo   serial:  build.bat monitor
)

pause
endlocal