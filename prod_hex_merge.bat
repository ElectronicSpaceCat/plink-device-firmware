:: Place this file at the top project directory

ECHO OFF

:: ============================================================
:: This script creates bootloader settings and then
:: merges the bootloader + bootloader settings + soft device + application
:: into one hex file for production programming.
:: ============================================================
:: NOTES:
::
:: Family setting  | nRF devices
:: ---------------------------------------------------------
:: NRF51           | nRF51xxx
:: NRF52           | nRF52832,           nRF52833
:: NRF52QFAB       | nRF52832-QFAB,      nRF52820
:: NRF52810        | nRF52810,           nRF52811, nRF52805
:: NRF52840        | nRF52840
::
:: SDK version     BL settings version
:: <=12.0          1
:: >=15.3.0        2
::
:: bootloader-version is user defined
:: ============================================================
::
:: Version types:
:: --application-version 1
:: --application-version-string "1.0.0"  (currently used in device)


:: Change path below to point to the nrfutil.exe
SET NRF_UTIL=..\nrfutil.exe

:: Change path below to point to the nRF softdevice (SD) hex file
SET NRF_SD=..\nRF5_SDK_17.1.0_ddde560\components\softdevice\s112\hex\s112_nrf52_7.2.0_softdevice.hex

:: Firmware location
SET FW=..\plink-device-firmware\_Prod_makefile\nrf_plink_app.hex

:: Bootloader location
SET BL=..\plink-device-bootloader\_Prod_makefile\nrf_plink_bootloader.hex

:: Output file name
SET OUTPUT_NAME=prod_plink_app

%NRF_UTIL% settings generate --family NRF52 --application %FW% --application-version-string "1.0.0" --bootloader-version 0 --bl-settings-version 2 bl_setting.hex

mergehex --merge bl_setting.hex %BL% %NRF_SD% --output temp_merge.hex

mergehex --merge temp_merge.hex %FW% --output %OUTPUT_NAME%.hex

DEL bl_setting.hex
DEL temp_merge.hex

::PAUSE
