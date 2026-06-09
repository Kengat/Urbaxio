@echo off
setlocal
set "ROOT=%~dp0.."
"%ROOT%\.venv\Scripts\pythonw.exe" -m qt_shell.main
endlocal
