ECHO OFF

:: Version types:
:: --application-version 1
:: --application-version-string "1.0.0"

:: Modify the path defines as needed

SET NRF_UTIL=.\
SET HEX_PATH=C:\Users\Green\Documents\eclipse-workspace\nrf_plink_s112\dfu_w_bonds\nrf_plink_app\_Prod
SET HEX_NAME=nRF_plink_app
SET OUTPUT_NAME=plink

%NRF_UTIL%\nrfutil.exe pkg generate --hw-version 52 --application-version-string "1.0.0" --application %HEX_PATH%\%HEX_NAME%.hex --sd-req 0x103 --key-file private.key %OUTPUT_NAME%.zip

Created %OUTPUT_NAME%.zip

::PAUSE