@ECHO OFF

ECHO This script will release a new version of the firmware, e.g. 1.0.0
SET /P new_version=Please enter the new version number: %=%

SET home_dir=%~dp0
SET release_folder=releases\%new_version%

IF NOT EXIST %release_folder% (
	MKDIR %release_folder%
    
	:: Non throttle version
	CD src\controller
	CALL compile_ihx.bat THROTTLE=0 || GOTO :error	
	CD %home_dir%
	COPY .\src\controller\main.ihx %release_folder%\TDSZ2_Controller_v%new_version%.hex

    ECHO:
    ECHO Release %new_version% has been created in %home_dir%%release_folder%
    ECHO:

    GOTO :EOF
) ELSE (
    ECHO:
    ECHO Release %new_version% already exists!
    ECHO:
)

:error
ECHO:
ECHO Failed with error code %errorlevel%.
ECHO:
EXIT /b %errorlevel%

@ECHO ON
