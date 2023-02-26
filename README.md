# Firmware application

The firmware prod_plink_app_s112.hex is a merge of the app + softdevice + bootloader + bootloader settings. The prod_hex_merge.bat will do this but the nrfutil.exe is required to run it. The path to the utility in the script should point to its location on your system.

The nrfutil can be found here:

https://www.nordicsemi.com/Products/Development-tools/nrf-util

The nRF5 SDK is required to build the project. Modify the makefile SDK paths to point to its location on your system.

The SDK can be downloaded here: 

https://nsscprodmedia.blob.core.windows.net/prod/software-and-other-downloads/sdks/nrf5/binaries/nrf5_sdk_17.1.0_ddde560.zip

Segger RTT is required to flash/debug the device.

## TODO
* instructions on flashing firmware via segger
