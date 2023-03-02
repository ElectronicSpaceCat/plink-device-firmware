# Driver for the ToF Sensors

The VL53L4CX (long range) and VL53L4CD (short range) ToF sensors are used in this project.

Each sensor has their own driver; however, the one for the VL53L4CX is more generic and covers all VL53LX sensors and it has more advanced sensor tuning. The Ultra lite driver (ULD) for the VL53L4CD is smaller in code size with basic functionallity. The former was chosen for both.

This driver has been modified to interface over I2C with the bluetooth module. 

### Files modified:
* vl53lx_api.c
* vl53lx_api.h
* vl53lx_platform.c
* vl53lx_platform.h

### Original drivers:

VL53L4CX - https://www.st.com/en/embedded-software/stsw-img029.html

VL53L4CD - https://www.st.com/en/embedded-software/stsw-img026.html
