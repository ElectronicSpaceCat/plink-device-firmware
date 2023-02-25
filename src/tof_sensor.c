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

#include "tof_sensor.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

const char* get_type_name_str(uint8_t type){
    switch(type){
        case VL53L4CD:
            return "VL53L4CD";
        case VL53L4CX:
            return "VL53L4CX";
        default:
            return "unknown type";
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

void config_cmd_message(const char* name, config_cmd_data_t* config_data){
    NRF_LOG_INFO("%s << cmd: %s, cfg: %d, value: %d",
            name,
            get_cmd_str(config_data->cmd),
            config_data->id,
            config_data->value);
}

void config_resp_message(const char* name, config_cmd_data_t* config_data){
    NRF_LOG_INFO("%s >> cmd: %s, cfg: %d, value: %d, status: %s",
            name,
            get_cmd_str(config_data->cmd),
            config_data->id,
            config_data->value,
            get_status_str(config_data->status));
}
