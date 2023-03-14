ECHO OFF

:: Version types:
:: --application-version 1
:: --application-version-string "1.0.0"    <-- currently used for this project

:: Modify the path defines as needed

SET NRF_UTIL_LOC=..\..\..\
SET HEX_FILE=..\..\..\plink-device-firmware\build\nrf_plink_app.hex
SET OUTPUT_NAME=plink
SET VERSION=1.0.0

%NRF_UTIL_LOC%\nrfutil.exe pkg generate --hw-version 52 --application-version-string "%VERSION%" --application %HEX_FILE% --sd-req 0x103 --key-file private.key %OUTPUT_NAME%.zip

::ECHO "%VERSION%"

:: Create version file without trailing white spaces or carriage return
ECHO|SET /P ="%VERSION%" > version

::PAUSE