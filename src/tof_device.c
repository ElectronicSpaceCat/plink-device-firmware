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

#include "tof_device.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "tof_twi.h"
#include "tof_fds.h"

#define FILE_ID(cfg_cmd_id) (cfg_cmd_id & 0xF0)

#define FILE_ID_EXT_STORAGE 0x1001
#define RKEY_EXT_STORAGE 0x1115

static device_t device;

static void config_cmd_ext_handler(void);

const char* get_sensor_name_str(uint8_t snsr_type){
    switch(snsr_type){
        case SNSR_TYPE_VL53L4CD:
            return "VL53L4CD";
        case SNSR_TYPE_VL53L4CX:
            return "VL53L4CX";
        default:
            return "unknown type";
    }
}

const char* get_trgt_str(uint8_t trgt){
    switch (trgt) {
        case CONFIG_TRGT_SNSR:
            return "sensor";
        case CONFIG_TRGT_EXT:
            return "ext";
        default:
            return "unknown";
    }
}

const char* get_cmd_str(uint8_t cmd){
    switch (cmd) {
        case CONFIG_CMD_GET:
            return "get";
        case CONFIG_CMD_SET:
            return "set";
        case CONFIG_CMD_RESET:
            return "reset";
        case CONFIG_CMD_STORE:
            return "store";
        default:
            return "unknown";
    }
}

const char* get_status_str(uint8_t status){
    switch (status) {
        case CONFIG_STAT_OK:
            return "ok";
        case CONFIG_STAT_UPDATED:
            return "updated";
        case CONFIG_STAT_MISMATCH:
            return "mismatch";
        case CONFIG_STAT_ERROR:
            return "error";
        case CONFIG_STAT_NA:
            return "not_available";
        default:
            return "unknown";
    }
}

void tof_device_init(void) {
    /* Initialize the i2c interface */
    tof_twi_init();
    /* Initialize the fds interface */
    tof_fds_init();

    // Read in the external stored data
    tof_fds_read(FILE_ID_EXT_STORAGE, RKEY_EXT_STORAGE, (uint8_t*)&device.ext_data[0], sizeof(device.ext_data));
}

void tof_device_uninit(void) {
    tof_twi_uninit();
}

void config_cmd_message(config_cmd_data_t* config_cmd){
    NRF_LOG_INFO("<< trgt: %s, cmd: %s, cfg: %d, value: %d",
            get_trgt_str(config_cmd->trgt),
            get_cmd_str(config_cmd->cmd),
            config_cmd->id,
            config_cmd->value);
}

void config_resp_message(config_cmd_data_t* config_cmd){
    NRF_LOG_INFO(">> trgt: %s, cmd: %s, cfg: %d, value: %d, status: %s",
            get_trgt_str(config_cmd->trgt),
            get_cmd_str(config_cmd->cmd),
            config_cmd->id,
            config_cmd->value,
            get_status_str(config_cmd->status));
}

void tof_config_cmd_set(uint8_t trgt, uint8_t cmd, uint8_t id, int32_t value){
    device.config_cmd.trgt = trgt;
    device.config_cmd.cmd = cmd;
    device.config_cmd.id = id;
    device.config_cmd.value = value;
    // Set the pending flag
    device.config_pending = true;
}

void tof_process_config_cmd(void) {
    // Return if no configuration pending
    if (!device.config_pending)
        return;

    switch (device.config_cmd.trgt) {
        case CONFIG_TRGT_SNSR:
            return; // Handled by sensor state machine
        case CONFIG_TRGT_EXT:
            tof_handle_config_cmd(&config_cmd_ext_handler);
            return;
        default:
            device.config_cmd.cmd = CONFIG_CMD_NA;
            tof_handle_config_cmd(NULL);
            return;
    }
}

void tof_handle_config_cmd(config_cmd_handler_t cmd_handler){
    // Print the command message
    config_cmd_message(&device.config_cmd);

    if(cmd_handler){
        cmd_handler();
    }

    // Print the response message
    config_resp_message(&device.config_cmd);
    // Notify the user
    tof_data_callback(&device, TOF_DATA_CONFIG);
    // Reset the cmd target
    device.config_cmd.trgt = CONFIG_TRGT_NA;
    // Reset the pending flag
    device.config_pending = false;
}

static void config_cmd_ext_handler(void) {
    uint8_t id = device.config_cmd.id;
    uint8_t cmd = device.config_cmd.cmd;
    int32_t value = device.config_cmd.value;

    device.config_cmd.status = CONFIG_STAT_OK;

    // If configuration id is not in list then exit function
    if (id >= MAX_STORAGE_DATA_BUFF_SIZE && cmd != CONFIG_CMD_STORE) {
        device.config_cmd.status = CONFIG_STAT_NA;
        return;
    }

    // Process the command
    switch (cmd) {
        case CONFIG_CMD_GET:
            device.config_cmd.value = device.ext_data[id];
            return;
        case CONFIG_CMD_SET:
            device.ext_data[id] = value;
            return;
        case CONFIG_CMD_RESET:
            // Maybe cache a default value in the future
            return;
        case CONFIG_CMD_STORE:
            tof_fds_write(FILE_ID_EXT_STORAGE, RKEY_EXT_STORAGE, (uint8_t*)&device.ext_data[0], sizeof(device.ext_data));
            return;
        default:
            device.config_cmd.status = CONFIG_STAT_INVALID;
            return;
    }
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

device_t* tof_device_get(void) {
    return &device;
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
