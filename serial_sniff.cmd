@echo off
rem Double-click launcher for serial_sniff.ps1
rem Bypasses the machine's script execution policy for this one run only.
setlocal
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0serial_sniff.ps1" %*
echo.
pause
