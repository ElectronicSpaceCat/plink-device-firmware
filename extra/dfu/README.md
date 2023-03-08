# Device Firmware Update (DFU)

The files here are used to generate a .zip package for doing over-the-air (OTA) device firmware update (DFU) updates to the device.

## Key and Package Generation

* STEP 1
Generate Private-Public Key Pair:
Navigate to a folder of your choice for storing the private-public key pair files.
With the nrfutil.exe run the commands:
nrfutil keys generate private.key
nrfutil keys display --key pk --format code private.key --out_file public_key.c

* STEP 2
Place public_key.c in the src folder of the bootloader project.

* STEP 3
Run plink_dfu_pkg.bat to a generate the .zip package for doing over-the-air (OTA) device firmware update (DFU) updates to the device.

### Breakdown of the plink_dfu_pkg.bat commands:

* pkg - Display or generate a DFU package (zip file)
* generate - Generate a zip file for performing DFU
* hw-version 52 - The hardware version
* application-version-string "1.0.0" - The assigned application version
* application nrf_plink_app.hex: The application hex file to be included in the DFU package
* sd-req 0x103 - The SoftDevice firmware ID(s) required for the update to be processed, of which one must be present on the target device
* key-file private.key - The private (signing) key in PEM format
* plink.zip - Name of the output ZIP file (the DFU package)

#### NOTE: The sd-req # can be found when connecting the NRF Connect Programmer over SWD or JTAG to the bluetooth device, should show up in the output window. If a new softdevice is being loaded over the old then the sd-id should reflect the new one.
