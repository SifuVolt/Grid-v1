@echo off
setlocal

if "%~1"=="" (
    echo Usage:
    echo   install_bumblebee_rlutilities.bat "C:\path\to\RLBotPack\RLBotPack\Botimus^&Bumblebee"
    echo.
    echo Pass the Botimus^&Bumblebee folder from your RLBotPack checkout.
    exit /b 2
)

set "SOURCE=%~1\rlutilities"
set "DEST=%~dp0rlutilities"

if not exist "%SOURCE%\rlutilities.cp311-win_amd64.pyd" (
    echo ERROR: CPython 3.11 Windows x64 RLUtilities binary not found:
    echo   "%SOURCE%\rlutilities.cp311-win_amd64.pyd"
    exit /b 3
)

if not exist "%SOURCE%\assets\soccar" (
    echo ERROR: RLUtilities soccar assets not found:
    echo   "%SOURCE%\assets\soccar"
    exit /b 4
)

echo Copying Bumblebee RLUtilities into:
echo   "%DEST%"
if not exist "%DEST%" mkdir "%DEST%"
xcopy "%SOURCE%" "%DEST%" /E /I /Y >nul
if errorlevel 1 (
    echo ERROR: Copy failed.
    exit /b 5
)

if not exist "%DEST%\rlutilities.cp311-win_amd64.pyd" (
    echo ERROR: Native extension copy did not complete.
    exit /b 6
)

echo.
echo Bumblebee RLUtilities installed.
echo Start Volt and look for [BUMBLEBEE FAST AERIAL] in CMD output.
exit /b 0
