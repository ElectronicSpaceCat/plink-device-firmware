ECHO OFF

:: Version types:
:: --application-version 1
:: --application-version-string "1.0.0"

:: Modify the path defines as needed

SET NRF_UTIL_LOC=..\..\..\
SET HEX_FILE=..\..\..\plink-device-firmware\_Prod_makefile\nrf_plink_app.hex
SET OUTPUT_NAME=plink

%NRF_UTIL_LOC%\nrfutil.exe pkg generate --hw-version 52 --application-version-string "1.0.0" --application %HEX_FILE% --sd-req 0x103 --key-file private.key %OUTPUT_NAME%.zip

::PAUSE