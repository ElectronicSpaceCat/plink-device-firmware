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

#ifndef TOF_STATE_MACHINE_H
#define TOF_STATE_MACHINE_H

#include "tof_sensor.h"
#include <stdint.h>

void tof_sm_init(void);
void tof_sm_uninit(void);
void tof_sm_run(void);

/**
 * Select sensor
 * @param snsr_id
 */
void tof_sensor_select(sensor_id_t snsr_id);

/**
 * Get the device data
 * @return
 */
const device_t* tof_device_get(void);

/**
 * Sends a configuration request
 * @param config_id  - Id of the configuration
 * @param value      - Value to send
 *                     A configuration request is sent if (value == 0x7FFFFFFF).
 *                     Otherwise any (value != 0x7FFFFFFF) will be processed.
 */
void tof_config_cmd(uint8_t cmd, uint8_t id, int32_t value);

/**
 * Debug: Get a configuration value
 * @param config_id
 * @return
 */
int32_t tof_sensor_cached_config_get(uint8_t config_id);

/**
 * Enable/Disable range sampling
 * @param value
 */
void tof_sensor_ranging_enable_set(uint8_t value);

/**
 * Debug: Enable/Disable debug mode
 * @param value
 */
void tof_sensor_debug_set(uint8_t value);

/**
 * Debug: Set the reference distance
 * @param distance_mm_ref
 */
void tof_sensor_debug_set_ref(uint16_t distance_mm_ref);

/**
 * Debug: Get the reference distance
 * @return distance_mm_ref
 */
uint16_t tof_sensor_debug_get_ref(void);

/**
 * Reset current sensor
 */
void tof_sensor_reset(uint8_t reset_type);

#endif /* TOF_STATE_MACHINE_H */
