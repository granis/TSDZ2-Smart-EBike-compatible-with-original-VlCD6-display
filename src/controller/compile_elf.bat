@echo off
PATH = %PATH%;C:\SDCC\usr\local\bin;%~dp0..\..\tools\cygwin\bin

make -f Makefile_elf_windows clean
if errorlevel == 1 goto FAIL

:: pass batch file parameters, e.g. THROTTLE=0
make -f Makefile_elf_windows %*
if errorlevel == 1 goto FAIL

:PASS
goto EXIT
:FAIL
pause
:EXIT