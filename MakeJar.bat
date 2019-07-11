@echo off
::PATH = %PATH%;c:\Program Files\Java\jdk1.8.0_191\bin
PATH = %PATH%;c:\Program Files\Java\jdk1.8.0_112\bin

javac -d . tools\JavaConfigurator\src\OSEC.java
if errorlevel == 1 goto FAIL

jar -cvmf tools\JavaConfigurator\src\OSEC.mf OSEC.jar *.class
if errorlevel == 1 goto FAIL

del *.class
if errorlevel == 1 goto FAIL

del "TSDZ2-Configurator.jar"
if errorlevel == 1 goto FAIL

move OSEC.jar "TSDZ2-Configurator.jar"
if errorlevel == 1 goto FAIL

:PASS
goto EXIT
:FAIL
pause
:EXIT
exit
