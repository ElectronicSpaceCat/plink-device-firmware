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

#ifndef TOF_DEVICE_H
#define TOF_DEVICE_H

#include "boards.h"
#include <stdint.h>

#define MAX_CONFIG_BUFF_SIZE 20
#define MAX_STORAGE_DATA_BUFF_SIZE 20
#define INVALID_CONFIG_ID 0xFF
#define INVALID_CONFIG_VALUE 0x7FFFFFFF
#define I2C_ADDR_DEFAULT 0x29

#define ERR_TIMEOUT_MS APP_TIMER_TICKS(2500)

/*
#define BUILD_STRUCT(name,s0,s1)                   \
             typedef enum {                        \
                TOF_SNSR_##s0,                     \
                TOF_SNSR_##s1,                     \
                NUM_TOF_SNSR,                      \
                TOF_SNSR_NA                        \
             }name##_type_t;                       \
             const char *name##_str[] = {          \
                     #s0, #s1                      \
             };                                    \
             const char *name##_to_str(int idx) {  \
                 if(idx < NUM_TOF_SNSR){           \
                        return name##_str[idx];    \
                 }                                 \
                 else{                             \
                     return "unknown";             \
                 }                                 \
              };

BUILD_STRUCT(tof_snsr, VL53L4CD, VL53L4CX)
*/

typedef enum {
    TOF_SNSR_SHORT_RANGE = 0,
    TOF_SNSR_LONG_RANGE,
    NUM_TOF_SNSR,
    TOF_SNSR_UNKNOWN
} sensor_id_t;

typedef enum {
    SNSR_TYPE_VL53L4CD = 0,
    SNSR_TYPE_VL53L4CX,
    NUM_SNSR_TYPES,
    SNSR_TYPE_NA
}sensor_type_t;

/** Sensor error mask to merge
 * with the status */
typedef enum {
    TOF_ERROR_NONE = 0,
    TOF_ERROR_NO_COMMS,
    TOF_ERROR_BOOT_CONFIG,
    NUM_TOF_ERRORS,
    TOF_ERROR_NA
} error_t;

/** High level statuses of the sensors
 * sent to the user */
typedef enum {
    TOF_STATUS_BOOTING = 0,
    TOF_STATUS_READY,
    TOF_STATUS_STANDBY,
    TOF_STATUS_ERROR,
    NUM_TOF_STATUS,
    TOF_STATUS_NA
} status_t;

/** Data types from the sensors wrapper
 * sent to the user */
typedef enum {
    TOF_DATA_SELECTED = 0,
    TOF_DATA_STATUS,
    TOF_DATA_SAMPLING_ENABLED,
    TOF_DATA_DISTANCE,
    TOF_DATA_CONFIG,
    TOF_DATA_RESET,
    NUM_TOF_DATA_TYPE,
    TOF_DATA_NA
} snsr_data_type_t;

typedef enum {
    CONFIG_STAT_OK = 0,
    CONFIG_STAT_UPDATED,
    CONFIG_STAT_MISMATCH,
    CONFIG_STAT_ERROR,
    CONFIG_STAT_INVALID,
    NUM_CONFIG_STATS,
    CONFIG_STAT_NA
} config_status_t;

typedef enum {
    CONFIG_CMD_GET = 0,
    CONFIG_CMD_SET,
    CONFIG_CMD_RESET,
    CONFIG_CMD_STORE,
    NUM_CONFIG_CMDS,
    CONFIG_CMD_NA
} config_cmd_t;

typedef enum {
    CONFIG_TYPE_PARAM = 0,
    CONFIG_TYPE_CAL,
    NUM_CONFIG_TYPES,
    CONFIG_TYPE_NA
} sensor_config_type_t;

typedef enum {
    CONFIG_TRGT_SNSR = 0,
    CONFIG_TRGT_EXT,
    NUM_CONFIG_TRGTS,
    CONFIG_TRGT_NA
} config_trgt_t;

typedef enum {
    TOF_RESET_DEVICE = 0,
    TOF_RESET_SENSOR,
    TOF_RESET_SENSOR_FACTORY,
    NUM_RESET_OPTIONS,
    TOF_RESET_NA
} reset_cmd_t;

typedef struct {
    uint8_t trgt; // target of interest
    uint8_t cmd; // configuration command
    uint8_t id; // id of the configuration
    int32_t value; // value of the configuration
    uint8_t status; // response status
}config_cmd_data_t;

typedef struct {
    uint8_t type;
    int32_t value;
    int32_t value_default;
}sensor_cfg_data_t;

// Forward declaration of the device_t type (allows circular reference)
typedef struct device_s device_t;

typedef void (*stateHandler_t)(void);

typedef struct {
    char name[20]; // sensor name
    uint8_t id; // sensor id
    uint8_t type; // sensor type
    uint8_t address; // address to override the default
    uint32_t pin_xshut; // pin id for xshut
    stateHandler_t state; // sensor state
    uint8_t status; // sensor status
    uint8_t error; // active error if any
    uint8_t num_configs; // number of available configurations within MAX_CONFIG_BUFF_SIZE
    sensor_cfg_data_t config[MAX_CONFIG_BUFF_SIZE]; // configuration value
    void* context; // used for unique sensor data
} snsr_data_t;

struct device_s{
    uint8_t id_selected; // selected sensor
    uint8_t is_ranging_enabled; // flag for enabling ranging
    uint8_t is_debug_enabled; // flag for enabling debug output
    uint16_t distance_mm; // ranging distance
    uint16_t distance_mm_ref; // ranging distance reference (for debugging)
    uint16_t sample_count; // sample count, used in debug
    config_cmd_data_t config_cmd; // configuration command data
    reset_cmd_t reset_cmd; // reset command
    snsr_data_t sensors[NUM_TOF_SNSR]; // sensors
    snsr_data_t* sensor; // active sensor
    int32_t ext_data[MAX_STORAGE_DATA_BUFF_SIZE];
};

/**
 * Callback used to pass sensor data
 */
void tof_data_callback(device_t *tof_data, snsr_data_type_t type);

/**
 * Initialize the device
 */
void tof_device_init(void);

/**
 * Un-Initialize the device
 */
void tof_device_uninit(void);

/**
 * Select sensor
 * @param snsr_id
 */
void tof_sensor_select(sensor_id_t snsr_id);

/**
 * Get the device data
 * @return
 */
device_t* tof_device_get(void);

/**
 * Sends a configuration request
 * @param trgt       - Target of interest
 * @param cmd        - Configuration command
 * @param id         - Id of the configuration
 * @param value      - Value to send
 */
void tof_config_cmd(uint8_t trgt, uint8_t cmd, uint8_t id, int32_t value);

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

/**
 * Get the senosr name
 * @return string
 */
const char* get_sensor_name_str(uint8_t snsr_type);

/**
 * Get the command name
 * @return string
 */
const char* get_cmd_str(uint8_t cmd);

/**
 * Process configuration command for external data storage
 * @param device
 */
void process_config_cmd_ext(void);

/**
 * Get the command status
 * @return string
 */
const char* get_status_str(uint8_t status);

/**
 * Log output the configuration command
 */
void config_cmd_message(config_cmd_data_t* config_data);

/**
 * Log output the configuration command response
 */
void config_resp_message(config_cmd_data_t* config_data);

#endif /* TOF_DEVICE_H */
