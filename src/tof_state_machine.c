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

#include "timer_delay.h"
#include "tof_fds.h"

#include "tof_VL53LX_states.h"

static device_t* device;

void tof_sm_init(void) {
    // Initialize device
    tof_device_init();

    device = tof_device_get();

    // Initialize device data
    device->id_selected = TOF_SNSR_SHORT_RANGE;
    device->is_debug_enabled = false;
    device->is_ranging_enabled = false;
    device->config_cmd.trgt = CONFIG_TRGT_NA;

    // Initialize the short range sensor
    vl53lx_init(SNSR_TYPE_VL53L4CD, TOF_SNSR_SHORT_RANGE, (I2C_ADDR_DEFAULT + 1), PIN_TOF_SHORT_XSHUT);

    // Initialize the long range sensor
    vl53lx_init(SNSR_TYPE_VL53L4CX, TOF_SNSR_LONG_RANGE, (I2C_ADDR_DEFAULT + 2), PIN_TOF_LONG_XSHUT);

    // Set the default sensor on startup
    device->sensor = &device->sensors[TOF_SNSR_SHORT_RANGE];
}

void tof_sm_uninit(void) {
    tof_device_uninit();
}

void tof_sm_run(void) {
    // Switch sensors only when the current sensor status is in Standby or Error
    // Note: Switching should be done here and not by the sensors in the event of sensor error
    if(device->sensor->id != device->id_selected &&
        (TOF_STATUS_STANDBY == device->sensor->status  || TOF_STATUS_ERROR == device->sensor->status)){
        device->sensor = &device->sensors[device->id_selected];
    }

    // Run current state
    device->sensor->state();
}
