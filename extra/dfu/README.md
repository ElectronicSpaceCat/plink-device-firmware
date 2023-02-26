# Device Firmware Update

The files here are used to generate a .zip package for doing over-the-air (OTA) device firmware update (DFU) updates to the device.

## Key and Package Generation

-- STEP 1 --
Generate Private-Public Key Pair:
Navigate to a folder of your choice for storing the private-public key pair files.
Run the commands:
nrfutil keys generate private.key
nrfutil keys display --key pk --format code private.key --out_file public_key.c

-- STEP 2 --
Place public_key.c in the src folder of the bootloader project.

-- STEP 3 --
Command to run at nrfutil.exe:
nrfutil.exe pkg generate --hw-version 52 --application-version 1 --application nRF_ble_dfu.hex --sd-req 0x101 --key-file private.key app_dfu_package.zip
Zip created at app_dfu_package.zip

Let’s look at each of the arguments passed into the command:

pkg: Display or generate a DFU package (zip file).
generate: Generate a zip file for performing DFU.

--hw-version 52: The hardware version.

--application-version 1: The assigned application version.

--application nRF_ble_dfu.hex.hex: The application hex file to be included in the DFU package.

--sd-req 0xB6: The SoftDevice firmware ID(s) required for the update to be processed, of which one must be present on the target device. Below is the list of SoftDevice firmware IDs supported by version 5.2.0 of the nrfutil command-line tool. We are using SoftDevice s140 version 6.1.1 which matches ID value: 0xB6.

--sd-id 0xB6: The new SoftDevice ID to be used as –sd-req for the Application update in case the ZIP contains a SoftDevice and an Application.

--key-file private.key: The private (signing) key in PEM format.
app_dfu_package.zip: Name of the output ZIP file (the DFU package).


sd-req # can be found when connecting the NRF Connect Programmer over SWD or JTAG to the bluetooth device, should show up in the output window.
If a new softdevice is being loaded over the old then the sd-id should reflect the new one.
