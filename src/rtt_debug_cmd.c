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

#include "rtt_debug_cmd.h"

#include "stdlib.h"

#include "boards.h"
#include "SEGGER_RTT.h"
#include "string.h"
#include "stdbool.h"
#include "tof_state_machine.h"
#include "tof_pwr_monitor.h"
#include "ble_tof_service.h"
#include "ble_pwr_service.h"
#include "peer_manager.h"

#define CHAR_BUFF_SIZE 20

typedef enum {
    DATA_COINFIG_SET,
    DATA_COINFIG_GET,
    DATA_DIST_REF,
    DATA_NA,
    NUM_DATA_ENTRIES
} data_entry_t;

static char _buff[CHAR_BUFF_SIZE];

void rtt_debug_cmd_init(void) {
    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
}

void rtt_debug_cmd_check(void) {
    // Exit immediately if no data to process
    if (!SEGGER_RTT_HasKey()) {
        return;
    }

    uint8_t idx = 0;
    int cmd_ready = 0;
    int r = 0;

    // Read characters as long as they are available
    while (SEGGER_RTT_HasKey()) {
        // Read characters into the buffer until newline or carriage return is seen
        r = SEGGER_RTT_GetKey();
        if (!cmd_ready) {
            if (idx < (CHAR_BUFF_SIZE - 1)) {
                if (r == '\r' || r == '\n') {
                    _buff[idx++] = '\0';
                    cmd_ready = 1;
                }
                else {
                    _buff[idx++] = (char) r;
                }
            }
        }
    }

    // Process the cmd when ready
    if (cmd_ready) {
        char *token = strtok(_buff, ":");

        // Get first token and process it
        if (!strcmp(token, "help")) {
            SEGGER_RTT_WriteString(0, "Commands:\r");
            SEGGER_RTT_WriteString(0, "batt:print        batt - toggle batt sampling, batt:print - toggle debug\r");
            SEGGER_RTT_WriteString(0, "rng:print         rng - toggle ranging, rng:print - toggle debug\r");
            SEGGER_RTT_WriteString(0, "ble:tof|pwr:print debug ble:tof|pwr service\r");
            SEGGER_RTT_WriteString(0, "snsr:id           snsr - get snsr id, snsr:id select snsr\r");
            SEGGER_RTT_WriteString(0, "ref:value         ref - get ref dist, ref:value - set ref dist\r");
            SEGGER_RTT_WriteString(0, "cfg:trgt:cmd:id:value  send config cmd, -cmds for cmd list, -trgts for trgt list\r");
            SEGGER_RTT_WriteString(0, "cfg:all           get all configs\r");
            SEGGER_RTT_WriteString(0, "rst:s|sf|d        reset: s - sensor, sf - sensor factory, d - device\r");
            SEGGER_RTT_WriteString(0, "shutdown          shutdown device\r");
            return;
        }
        else if (!strcmp(token, "-cmds")) {
            SEGGER_RTT_WriteString(0, "config Commands:\r");
            SEGGER_RTT_WriteString(0, "0  get\r");
            SEGGER_RTT_WriteString(0, "1  set\r");
            SEGGER_RTT_WriteString(0, "2  reset\r");
            SEGGER_RTT_WriteString(0, "3  store\r");
            return;
        }
        else if (!strcmp(token, "-trgts")) {
            SEGGER_RTT_WriteString(0, "config targets:\r");
            SEGGER_RTT_WriteString(0, "0  active sensor\r");
            SEGGER_RTT_WriteString(0, "1  storage\r");
            return;
        }
        else if (!strcmp(token, "batt")) {
            token = strtok(NULL, ":");
            if (NULL == token) {
                tof_pwr_batt_sample_voltage();
            }
            else if (!strcmp(token, "print")) {
                static uint8_t _toggle = 0;
                _toggle = _toggle ? 0 : 1;
                tof_pwr_batt_print_enable(_toggle);
                if (_toggle) {
                    SEGGER_RTT_WriteString(0, "batt debug enable\r");
                }
                else {
                    SEGGER_RTT_WriteString(0, "batt debug disable\r");
                }
            }
            else{
                // Do nothing..
            }
            return;
        }
        else if (!strcmp(_buff, "rng")) {
            token = strtok(NULL, ":");
            if (NULL == token) {
                const device_t *device = tof_device_get();
                tof_sensor_ranging_enable_set(device->is_ranging_enabled? 0 : 1);
                if (device->is_ranging_enabled) {
                    SEGGER_RTT_WriteString(0, "range enabled\r");
                }
                else {
                    SEGGER_RTT_WriteString(0, "range disabled\r");
                }
            }
            else if (!strcmp(token, "print")) {
                const device_t *device = tof_device_get();
                tof_sensor_debug_set(device->is_debug_enabled? 0 : 1);
                if (device->is_debug_enabled) {
                    SEGGER_RTT_WriteString(0, "range debug enable\r");
                }
                else {
                    SEGGER_RTT_WriteString(0, "range debug disable\r");
                }
            }
            else{
                // Do nothing..
            }
            return;
        }
        else if (!strcmp(token, "ble")) {
            token = strtok(NULL, ":");
            if (!strcmp(token, "tof")) {
                token = strtok(NULL, ":");
                if (!strcmp(token, "print")) {
                    static uint8_t _toggle = 0;
                    _toggle = _toggle ? 0 : 1;
                    tof_gatts_hvx_debug_set(_toggle);
                    if (_toggle) {
                        SEGGER_RTT_WriteString(0, "ble:tof debug enable\r");
                    }
                    else {
                        SEGGER_RTT_WriteString(0, "ble:tof debug disable\r");
                    }
                }
            }
            else if (!strcmp(token, "pwr")) {
                token = strtok(NULL, ":");
                if (!strcmp(token, "print")) {
                    static uint8_t _toggle = 0;
                    _toggle = _toggle ? 0 : 1;
                    pwr_gatts_hvx_debug_set(_toggle);
                    if (_toggle) {
                        SEGGER_RTT_WriteString(0, "ble:pwr debug enable\r");
                    }
                    else {
                        SEGGER_RTT_WriteString(0, "ble:pwr debug disable\r");
                    }
                }
            }
            else{
                // Do nothing..
            }
            return;
        }
        else if (!strcmp(_buff, "snsr")) {
            token = strtok(NULL, ":");
            if (NULL == token) {
                const device_t *device = tof_device_get();
                SEGGER_RTT_printf(0, "sensor selected: %s, addr: 0x%X\r\n", device->sensor->name, device->sensor->address);
            }
            else {
                uint8_t id = (uint8_t) atol(token);
                tof_sensor_select(id);
            }
            return;
        }
        else if (!strcmp(_buff, "ref")) {
            token = strtok(NULL, ":");
            if (NULL == token) {
                uint16_t value = tof_sensor_debug_get_ref();
                SEGGER_RTT_printf(0, "ref set to: %d\r\n", value);
            }
            else {
                uint16_t dist_ref = (uint16_t) atol(token);
                tof_sensor_debug_set_ref(dist_ref);
                SEGGER_RTT_printf(0, "ref set to: %d\r\n", dist_ref);
            }
            return;
        }
        else if (!strcmp(_buff, "cfg")) {
            token = strtok(NULL, ":");
            if (!strcmp(token, "all")) {
                const device_t *device = tof_device_get();
                for (uint8_t i = 0; i < device->sensor->num_configs; ++i) {
                    int32_t value = tof_sensor_cached_config_get(i);
                    SEGGER_RTT_printf(0, "config: %d : %d\r\n", i, value);
                }
            }
            else {
                uint8_t trgt = (uint8_t) atol(token);
                token = strtok(NULL, ":");
                if (NULL == token) {
                    return;
                }

                uint8_t cmd = (uint8_t) atol(token);

                token = strtok(NULL, ":");
                if (NULL == token) {
                    return;
                }

                uint8_t id = (uint8_t) atol(token);

                if(cmd != CONFIG_CMD_SET){
                    tof_config_cmd_set(trgt, cmd, id, 0);
                    return;
                }

                token = strtok(NULL, ":");
                if (NULL == token) {
                    return;
                }

                int32_t value = (int32_t) atol(token);
                tof_config_cmd_set(trgt, cmd, id, value);
            }
            return;
        }
        else if (!strcmp(_buff, "rst")) {
            token = strtok(NULL, ":");
            if (!strcmp(token, "s")) {
                tof_sensor_reset(TOF_RESET_SENSOR);
                return;
            }
            else if (!strcmp(token, "sf")) {
                tof_sensor_reset(TOF_RESET_SENSOR_FACTORY);
                return;
            }
            else if (!strcmp(token, "d")) {
                tof_pwr_reset();
                return;
            }
            else{
                // Do nothing..
            }
        }
        else if (!strcmp(_buff, "shutdown")) {
            tof_pwr_shutdown();
            return;
        }
        else{
            // Nothing to process..
        }

        SEGGER_RTT_WriteString(0, "unknown command\r");
    }
}
