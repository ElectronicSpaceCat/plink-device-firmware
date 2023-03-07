/*******************************************************************************
    BSD 3-Clause License

    Copyright (c)  2021-present  Andrew Green

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its
       contributors may be used to endorse or promote products derived from
       this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *******************************************************************************/

#include "tof_state_machine.h"

#include "boards.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "tof_twi.h"
#include "timer_delay.h"

#include "tof_fds.h"


#define SENSOR_SHORT VL53L4CD
#define SENSOR_LONG  VL53L4CX

#define SENSOR_SHORT_ADDR (I2C_ADDR_DEFAULT + 1)
#define SENSOR_LONG_ADDR (I2C_ADDR_DEFAULT + 2)

#include "tof_VL53LX_states.h"

static device_t device;

void tof_sm_init(void) {
    /* Initialize the i2c interface */
    tof_twi_init();

    // Initialize the sensor data
    device.id_selected = TOF_SNSR_SHORT_RANGE;
    device.is_debug_enabled = false;
    device.is_ranging_enabled = false;
    device.config_data.cmd = CONFIG_CMD_NA;

    // Initialize the short range sensor
    vl53lx_init(&device.sensors[TOF_SNSR_SHORT_RANGE], SENSOR_SHORT, TOF_SNSR_SHORT_RANGE, SENSOR_SHORT_ADDR, PIN_TOF_SHORT_XSHUT);

    // Initialize the long range sensor
    vl53lx_init(&device.sensors[TOF_SNSR_LONG_RANGE], SENSOR_LONG, TOF_SNSR_LONG_RANGE, SENSOR_LONG_ADDR, PIN_TOF_LONG_XSHUT);

    // Set the default sensor on startup
    device.sensor = &device.sensors[TOF_SNSR_SHORT_RANGE];
}

void tof_sm_uninit(void) {
    tof_twi_uninit();
}

void tof_sm_run(void) {
    // Switch sensors only when the current sensor status is in Standby or Error
    // Note: Switching should be done here and not by the sensors in the event of sensor error
    if(device.sensor->id != device.id_selected &&
        (TOF_STATUS_STANDBY == device.sensor->status  || TOF_STATUS_ERROR == device.sensor->status)){
        device.sensor = &device.sensors[device.id_selected];
    }

    // Run current state
    device.sensor->state(&device);
}

void tof_sensor_select(sensor_id_t id) {
    if (device.sensor->id == id) {
        NRF_LOG_INFO("%s already selected", device.sensor->name);
    }
    else if (id < NUM_TOF_SNSR) {
        device.id_selected = id;
        NRF_LOG_INFO("%s requested", device.sensors[id].name);
    }
    else {
        NRF_LOG_INFO("invalid sensor id");
    }
}

const device_t* tof_device_get(void) {
    return &device;
}

void tof_config_cmd(uint8_t cmd, uint8_t id, int32_t value){
    device.config_data.cmd = cmd;
    device.config_data.id = id;
    device.config_data.value = value;
    // Set pending flag
    device.config_pending = true;
}

int32_t tof_sensor_cached_config_get(uint8_t config_id){
    if(config_id < device.sensor->num_configs){
        return device.sensor->config[config_id].value;
    }
    else{
        return INVALID_CONFIG_VALUE;
    }
}

void tof_sensor_debug_set(uint8_t value) {
    device.is_debug_enabled = value;
}

void tof_sensor_debug_set_ref(uint16_t distance_mm_ref) {
    device.distance_mm_ref = distance_mm_ref;
}

uint16_t tof_sensor_debug_get_ref(void) {
    return device.distance_mm_ref;
}

void tof_sensor_ranging_enable_set(uint8_t value) {
    if(device.is_ranging_enabled != value){
        if(value){
            device.is_ranging_enabled = 1;
            NRF_LOG_INFO("%s ranging enabled", device.sensor->name);
        }
        else{
            device.is_ranging_enabled = 0;
            NRF_LOG_INFO("%s ranging disabled", device.sensor->name);
        }
    }
    tof_data_callback(&device, TOF_DATA_SAMPLING_ENABLED);
}

void tof_sensor_reset(uint8_t reset_type){
    device.reset_cmd = reset_type;
}
