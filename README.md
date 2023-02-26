# Plink

<img src="https://github.com/ElectronicSpaceCat/plink-device-app/blob/main/extra/images/device.jpg" width="250"> <img src="https://github.com/ElectronicSpaceCat/plink-device-app/blob/main/extra/images/device_on_phone.jpg" width="250"> <img src="https://github.com/ElectronicSpaceCat/plink-device-app/blob/main/extra/images/testrig_back.JPG" width="200">

What is Plink? It is an over-engineered coin shooter whith the intention of being as small as possible yet delivering a decent launch range. It uses torsion springs instead of rubberbands (which you'll see in most designs). It can reach about 15ft with the current springs, but it can easily hit 20-25ft with larger ones. The big difference with Plink is that it has two custom circuit boards working togetehr to read the coin carriage positon and device height using time-of-flight (ToF) sensors. The sensor data is relayed over bluetooth low energy (BLE) to an Android mobile device. The mobile's gyroscope is required in conjuntion with the sensor data for target alignment and ballistic calculations. It runs on a lithium-ion battery which can be charged over USB-C and a single button is used to turn on/off the device. To top it off, the device has over-the-air (OTA) device-firmeware-update (DFU) where it can recieve updates over Bluetooth via the Android mobile application.

The firmware prod_plink_app_s112.hex is a merge of the app + softdevice + bootloader + bootloader settings. The prod_hex_merge.bat will do this but the nrfutil.exe is required to run it. The path to the utility in the script should point to its location on your system.

The nrfutil can be found here:

https://www.nordicsemi.com/Products/Development-tools/nrf-util

The nRF5 SDK is required to build the project. Modify the makefile SDK paths to point to its location on your system.

The SDK can be downloaded here: 

https://nsscprodmedia.blob.core.windows.net/prod/software-and-other-downloads/sdks/nrf5/binaries/nrf5_sdk_17.1.0_ddde560.zip

Segger RTT is required to flash/debug the device.

## TODO
* instructions on flashing firmware via segger
* images of device, pcbs, app screenshots, etc

## Android App (currently not avaiable for download)

<img src="https://github.com/ElectronicSpaceCat/plink-android-app/blob/master/extra/images/Screenshot_20230217-104249.png" width="150"> <img src="https://github.com/ElectronicSpaceCat/plink-android-app/blob/master/extra/images/Screenshot_20230217-104148.png" width="150"> <img src="https://github.com/ElectronicSpaceCat/plink-android-app/blob/master/extra/images/Screenshot_20230217-104220.png" width="150"> <img src="https://github.com/ElectronicSpaceCat/plink-android-app/blob/master/extra/images/Screenshot_20230217-104228.png" width="150"> <img src="https://github.com/ElectronicSpaceCat/plink-android-app/blob/master/extra/images/Screenshot_20230217-104358.png" width="150"> <img src="https://github.com/ElectronicSpaceCat/plink-android-app/blob/master/extra/images/Screenshot_20230217-104308.png" width="150">
