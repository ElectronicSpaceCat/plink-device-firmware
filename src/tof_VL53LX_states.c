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

#include "tof_VL53LX_states.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_gpio.h"

#include "app_timer.h"

#include "timer_delay.h"

#include "tof_fds.h"

#include "vl53lx_api.h"

// State functions
static void state_boot(device_t *device);
static void state_prepare(device_t *device);
static void state_init(device_t *device);
static void state_idle(device_t *device);
static void state_start(device_t *device);
static void state_stop(device_t *device);
static void state_clear_int_and_start(device_t *device);
static void state_standby(device_t *device);
static void state_int_status(device_t *device);
static void state_get_result(device_t *device);
static void state_err(device_t *device);
static void state_err_timeout(device_t *device);
static void state_config(device_t *device);

// Structure holding stored information for the sensors
// that resides in FDS. It is 4-byte (word) aligned to
// match FDS flash storage alignment type.
typedef struct {
    struct{
        uint8_t power_level;
        uint8_t phasecal_pwr_lvl;
        uint8_t offset_mode;
        uint8_t distance_mode;
    };
    struct{
        uint8_t en_smudge_corr;
        uint8_t en_xtalk_comp;
        uint16_t unused;
    };
    int32_t time_budget;
    uint32_t roi;
    VL53LX_CalibrationData_t cal;
}store_data_t;

// All sensor data
typedef struct {
    VL53LX_Dev_t sensor;
    store_data_t store;
}sensor_data_t;

// Helper functions
static void process_config_cmd(device_t* device);
static uint8_t set_config(snsr_data_t* sensor, uint8_t id, int32_t value);
static uint8_t load_config(snsr_data_t* sensor, uint8_t id);
static void get_store_data(snsr_data_t* sensor, store_data_t* data);
static const char* get_config_str(uint8_t config);

APP_TIMER_DEF(err_timeout_timer_id);

// Helper macros to access sensor data
#define VL53LX(snsr)            ((VL53LX_DEV)&(((sensor_data_t*)snsr->context)->sensor))
#define VL53LX_DATA(snsr)       ((sensor_data_t*)snsr->context)
#define VL53LX_DATA_STORE(snsr) (((sensor_data_t*)snsr->context)->store)


typedef enum {
    // Configurations
    CONFIG_POWER_LEVEL = 0,
    CONFIG_PHASECAL_PATCH_PWR, // Note: Setting to 2 increases the time of the first sample which is used as a reference
    CONFIG_TIME_BUDGET,
    CONFIG_OFFSET_MODE,
    CONFIG_DISTANCE_MODE,
    CONFIG_SMUDGE_CORR_EN,
    CONFIG_XTALK_COMP_EN,
    CONFIG_RECT_OF_INTEREST, // Note: If using value 101255430: TopLeft_XY(6,9) BottomRight_XY(9,6) is a 4x4 SPAD array in a 15x15 box where BottomLeft is (0,0) and TopRight is (15,15)
    // Calibrations
    CONFIG_CAL_REFSPAD,
    CONFIG_CAL_OFFSET_SIMPLE,
    CONFIG_CAL_OFFSET_ZERO,
    CONFIG_CAL_OFFSET_VCSEL,
    CONFIG_CAL_XTALK,
    NUM_CONFIGS,
} config_t;

#if (NUM_CONFIGS > MAX_CONFIG_BUFF_SIZE)
#error Increase MAX_CONFIG_BUFF_SIZE to handle NUM_CONFIGS for this device
#endif

void vl53lx_init(snsr_data_t *sensor, uint8_t type, uint8_t id, uint8_t address, uint8_t xshut_pin) {
    ASSERT(sensor)

    snprintf(sensor->name, sizeof(sensor->name), "%s_%d", get_type_name_str(type), id);

    sensor->id = id;
    sensor->type = type;
    sensor->address = address;
    sensor->pin_xshut = xshut_pin;
    sensor->num_configs = NUM_CONFIGS;
    sensor->state = state_boot;
    sensor->status = TOF_STATUS_BOOTING;

    // Pointer to new instance of VL53LX_DEV
    // Note: Required if using multiple instances of same sensor type
    sensor->context = (void*)malloc(sizeof(sensor_data_t));

    ASSERT(sensor->context)

    /* Configure XSHUT pin (gpio0 on the sensor) as output */
    nrf_gpio_cfg_output(sensor->pin_xshut);
    /* Set XSHUT pin low to trigger a "fresh out of reset" condition */
    nrf_gpio_pin_clear(sensor->pin_xshut);

    /* Init the flash-data-storage if not already */
    tof_fds_init();
}

static void err_timeout_timer_handler(void *p_context) {
    device_t *device = (device_t*) p_context;
    device->sensor->state = state_boot;
    device->sensor->status = TOF_STATUS_ERROR;
    tof_data_callback(device, TOF_DATA_STATUS);
}

static void load_default_configs(device_t *device) {
    // Load and copy default configurations
    for (uint8_t i = 0; i < NUM_CONFIGS; ++i) {
        load_config(device->sensor, i);
        // Store copy of the factory default values
        device->sensor->config[i].value_default = device->sensor->config[i].value;
    }
}

static void state_boot(device_t *device) {
    // Restart the device by toggling the xshut pin
    nrf_gpio_pin_clear(device->sensor->pin_xshut);
    PollingDelayMS(5);
    nrf_gpio_pin_set(device->sensor->pin_xshut);
    PollingDelayMS(5);

    // Notify sensor selected
    tof_data_callback(device, TOF_DATA_SELECTED);
    // Notify sensor status
    device->sensor->status = TOF_STATUS_BOOTING;
    tof_data_callback(device, TOF_DATA_STATUS);

    // Set the factory default address
    VL53LX(device->sensor)->i2c_slave_address = I2C_ADDR_DEFAULT;

    // Wait for boot-up
    if (!VL53LX_WaitDeviceBooted(VL53LX(device->sensor))) {
        NRF_LOG_INFO("%s booted", device->sensor->name);
    }
    else {
        NRF_LOG_INFO("%s err: boot", device->sensor->name);
        device->sensor->state = state_err;
        return;
    }

    /* Set new address */
    if (!VL53LX_SetDeviceAddress(VL53LX(device->sensor), ((uint8_t)device->sensor->address << 1))) {
        VL53LX(device->sensor)->i2c_slave_address = device->sensor->address;
        NRF_LOG_INFO("%s address set to %d", device->sensor->name, device->sensor->address);
    }
    else {
        NRF_LOG_INFO("%s err: setting address", device->sensor->name);
        device->sensor->state = state_err;
        return;
    }

//    // Get device information (only use if comparing sensor meta data)
//    VL53LX_DeviceInfo_t dInfo;
//    if (!VL53LX_GetDeviceInfo(VL53LX(device->sensor), &dInfo)) {
//        NRF_LOG_INFO("%s get info", device->sensor->name);
//    }
//    else {
//        NRF_LOG_INFO("%s err: get info", device->sensor->name);
//        device->sensor->state = state_err;
//        return;
//    }

    device->sensor->state = state_prepare;
}

static void state_prepare(device_t *device) {
    // Data init
    if (!VL53LX_DataInit(VL53LX(device->sensor))) {
        NRF_LOG_INFO("%s data init", device->sensor->name);
    }
    else {
        NRF_LOG_INFO("%s err: data init", device->sensor->name);
        device->sensor->state = state_err;
        return;
    }

    // Load and copy default configurations
    load_default_configs(device);

    // Is a factory reset requested?
    if (TOF_RESET_SENSOR_FACTORY == device->reset_cmd) {
        // Yes - Delete the stored sensor data
        tof_fds_delete(device->sensor->id);
    }

    // Read stored data from fds if it exists
    if(!tof_fds_read(device->sensor->id, (uint8_t*)&VL53LX_DATA_STORE(device->sensor), sizeof(VL53LX_DATA_STORE(device->sensor)))){
        // Set calibration data
        if (!VL53LX_SetCalibrationData(VL53LX(device->sensor), &VL53LX_DATA_STORE(device->sensor).cal)) {
            NRF_LOG_INFO("%s set cal data", device->sensor->name);
        }
        else {
            NRF_LOG_INFO("%s err: set cal data", device->sensor->name);
        }

        // Set distance mode Note: this should be called after setting the calibration data
        if(!set_config(device->sensor, CONFIG_DISTANCE_MODE, VL53LX_DATA_STORE(device->sensor).distance_mode)){
            load_config(device->sensor, CONFIG_DISTANCE_MODE);
            NRF_LOG_INFO("%s set %s", device->sensor->name, get_config_str(CONFIG_DISTANCE_MODE));
        }
        else{
            NRF_LOG_INFO("%s set %s: error", device->sensor->name, get_config_str(CONFIG_DISTANCE_MODE));
        }

        // Set the user configurations
        int32_t ldata = 0;
        for(int i = 0; i < NUM_CONFIGS; ++i){

            // Skip distance mode since it's handled outside of this loop
            if(i == CONFIG_DISTANCE_MODE){
                continue;
            }

            // Break when last configuration is hit..ie don't process calibrations
            if(i > CONFIG_RECT_OF_INTEREST){
                break;
            }

            switch(i){
                case CONFIG_POWER_LEVEL:
                    ldata = (int32_t) VL53LX_DATA_STORE(device->sensor).power_level;
                    break;
                case CONFIG_PHASECAL_PATCH_PWR:
                    ldata = (int32_t) VL53LX_DATA_STORE(device->sensor).phasecal_pwr_lvl;
                    break;
                case CONFIG_TIME_BUDGET:
                    ldata = (int32_t) VL53LX_DATA_STORE(device->sensor).time_budget;
                    break;
                case CONFIG_OFFSET_MODE:
                    ldata = (int32_t) VL53LX_DATA_STORE(device->sensor).offset_mode;
                    break;
                case CONFIG_DISTANCE_MODE: // Should not hit this
                    ldata = (int32_t) VL53LX_DATA_STORE(device->sensor).distance_mode;
                    break;
                case CONFIG_SMUDGE_CORR_EN:
                    ldata = (int32_t) VL53LX_DATA_STORE(device->sensor).en_smudge_corr;
                    break;
                case CONFIG_XTALK_COMP_EN:
                    ldata = (int32_t) VL53LX_DATA_STORE(device->sensor).en_xtalk_comp;
                    break;
                case CONFIG_RECT_OF_INTEREST:
                    ldata = (int32_t) VL53LX_DATA_STORE(device->sensor).roi;
                    break;
                default:
                    break;
            }

            // Set the configuration. If no error then cache the value
            if(CONFIG_STAT_ERROR != set_config(device->sensor, i, ldata)){
                load_config(device->sensor, i);
                NRF_LOG_INFO("%s set %s", device->sensor->name, get_config_str(i));
            }
            else{
                NRF_LOG_INFO("%s set %s: error", device->sensor->name, get_config_str(i));
                device->sensor->state = state_err;
            }
        }
    }
    else{
        // Perform ref spad
        if (!set_config(device->sensor, CONFIG_CAL_REFSPAD, 0)) {
            NRF_LOG_INFO("%s ref spad", device->sensor->name);
        }
        else {
            NRF_LOG_INFO("%s err: ref spad", device->sensor->name);
            device->sensor->state = state_err;
            return;
        }

        // Perform xtalk
        if (!set_config(device->sensor, CONFIG_CAL_XTALK, 0)) {
            NRF_LOG_INFO("%s xtalk", device->sensor->name);
        }
        else {
            NRF_LOG_INFO("%s err: xtalk", device->sensor->name);
            device->sensor->state = state_err;
            return;
        }
    }

    // Go to state_init
    device->sensor->state = state_init;
}

static void state_init(device_t *device) {
    device->sample_count = 0;
    device->distance_mm = 0;

    device->sensor->status = TOF_STATUS_READY;
    NRF_LOG_INFO("%s ready", device->sensor->name);

    tof_data_callback(device, TOF_DATA_SELECTED);
    tof_data_callback(device, TOF_DATA_STATUS);
    tof_data_callback(device, TOF_DATA_DISTANCE);
    tof_data_callback(device, TOF_DATA_SAMPLING_ENABLED);

    if (TOF_RESET_SENSOR == device->reset_cmd ||
        TOF_RESET_SENSOR_FACTORY == device->reset_cmd) {
        tof_data_callback(device, TOF_DATA_RESET);
    }

    device->reset_cmd = TOF_RESET_NA;

    // Go to state_idle
    device->sensor->state = state_idle;
}

static void state_idle(device_t *device) {
    // If this sensor is not the selected then standby
    if (device->sensor->id != device->id_selected) {
        device->sensor->state = state_standby;
    }
    // Process configuration commands
    else if (true == device->config_pending) {
        device->sensor->state = state_config;
    }
    // Reset the sensor if the reset command was sent
    else if (TOF_RESET_SENSOR == device->reset_cmd ||
             TOF_RESET_SENSOR_FACTORY == device->reset_cmd) {
        device->sensor->state = state_boot;
    }
    // Run the sensor ranging if enabled
    else if (device->is_ranging_enabled) {
        device->sensor->state = state_start;
    }
    else {
        // Do nothing..
    }
}

static void state_start(device_t *device) {
    if (!VL53LX_StartMeasurement(VL53LX(device->sensor))) {
        device->sensor->state = state_clear_int_and_start;
    }
    else {
        NRF_LOG_INFO("%s err: start ranging", device->sensor->name);
        device->sensor->state = state_err;
    }
}

static void state_stop(device_t *device) {
    // Make sure a to stop any err timers
    (void) app_timer_stop(err_timeout_timer_id);

    if (!VL53LX_StopMeasurement(VL53LX(device->sensor))) {
        device->sample_count = 0;
        device->distance_mm = 0;
        tof_data_callback(device, TOF_DATA_DISTANCE);
        NRF_LOG_INFO("%s stopped", device->sensor->name);
        device->sensor->state = state_idle;
    }
    else {
        NRF_LOG_INFO("%s err: stopping", device->sensor->name);
        device->sensor->state = state_err;
    }
}

void state_clear_int_and_start(device_t *device) {
    // If this sensor is not the selected then standby
    if (device->sensor->id != device->id_selected) {
        device->sensor->state = state_stop;
    }
    // Process configuration commands
    else if (true == device->config_pending) {
        device->sensor->state = state_config;
    }
    // Reset the sensor if the command was sent
    else if (TOF_RESET_SENSOR == device->reset_cmd ||
             TOF_RESET_SENSOR_FACTORY == device->reset_cmd) {
        device->sensor->state = state_boot;
    }
    // Stop the sensor ranging if disabled
    else if (!device->is_ranging_enabled) {
        device->sensor->state = state_stop;
    }
    else if (!VL53LX_ClearInterruptAndStartMeasurement(VL53LX(device->sensor))) {
        device->sensor->state = state_int_status;
    }
    else {
        NRF_LOG_INFO("%s err: clear int and start", device->sensor->name);
        device->sensor->state = state_err;
    }
}

static void state_standby(device_t *device) {
    // Wait here until the state machine switches to another sensor
    if (device->sensor->id != device->id_selected) {
        device->sensor->status = TOF_STATUS_STANDBY;
    }
    else {
        device->sensor->state = state_init;
    }
}

static void state_int_status(device_t *device) {
    uint8_t pMeasurementDataReady;
    if (!VL53LX_GetMeasurementDataReady(VL53LX(device->sensor), &pMeasurementDataReady)) {
        if (pMeasurementDataReady) {
            device->sensor->state = state_get_result;
        }
    }
    else {
        NRF_LOG_INFO("%s err: get status", device->sensor->name);
        device->sensor->state = state_err;
    }
}

static void state_get_result(device_t *device) {
    VL53LX_MultiRangingData_t rangeData;
    if (!VL53LX_GetMultiRangingData(VL53LX(device->sensor), &rangeData)) {
        device->sample_count++;
        int16_t lrange = 0;
        for (int j = 0; j < rangeData.NumberOfObjectsFound; ++j) {
            // Keep the largest range value
            if (lrange < rangeData.RangeData[j].RangeMilliMeter) {
                lrange = rangeData.RangeData[j].RangeMilliMeter;
            }
            if (device->is_debug_enabled) {
                NRF_LOG_INFO("%s obj: %d, status=%d, D=%5dmm, Signal=%2.2f Mcps, Ambient=%2.2f Mcps",
                        device->sensor->name,
                        j,
                        rangeData.RangeData[j].RangeStatus,
                        rangeData.RangeData[j].RangeMilliMeter,
                        rangeData.RangeData[j].SignalRateRtnMegaCps / 65536.0,
                        rangeData.RangeData[j].AmbientRateRtnMegaCps / 65536.0);
            }
        }
        device->distance_mm = lrange;
        tof_data_callback(device, TOF_DATA_DISTANCE);
        device->sensor->state = state_clear_int_and_start;
    }
    else {
        NRF_LOG_INFO("%s err: get result", device->sensor->name);
        device->sensor->state = state_err;
    }
}

static void state_err(device_t *device) {
    uint32_t err_code = app_timer_create(&err_timeout_timer_id, APP_TIMER_MODE_SINGLE_SHOT, err_timeout_timer_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(err_timeout_timer_id, ERR_TIMEOUT_MS, device);
    APP_ERROR_CHECK(err_code);

    device->sensor->state = state_err_timeout;
}

static void state_err_timeout(device_t *device) {
    // Sit here until err timeout forces a reboot
}

static void state_config(device_t *device) {
    // Print the command message
    config_cmd_message(device->sensor->name, &device->config_data);
    // Set the configuration
    process_config_cmd(device);
    // Clear the flag
    device->config_pending = false;
    // Print the response message
    config_resp_message(device->sensor->name, &device->config_data);
    // Notify the user
    tof_data_callback(device, TOF_DATA_CONFIG);
    // Reset state to idle
    device->sensor->state = state_idle;
}

static void process_config_cmd(device_t *device) {
    uint8_t id = device->config_data.id;
    uint8_t cmd = device->config_data.cmd;
    int32_t value = device->config_data.value;

    device->config_data.status = CONFIG_STAT_OK;

    // Special command to store all configuration data
    if(CONFIG_CMD_STORE == cmd && INVALID_CONFIG_ID == id){
        get_store_data(device->sensor, &VL53LX_DATA_STORE(device->sensor));
        tof_fds_write(device->sensor->id, (uint8_t*)&VL53LX_DATA_STORE(device->sensor), sizeof(VL53LX_DATA_STORE(device->sensor)));
        return;
    }

    // If configuration id not in list then exit function
    if (id >= NUM_CONFIGS) {
        device->config_data.status = CONFIG_STAT_NA;
        return;
    }

    // Process the command
    switch (cmd) {
        case CONFIG_CMD_GET:
            // Override the value to send back
            device->config_data.value = device->sensor->config[id].value;
            return;
        case CONFIG_CMD_SET:
            // Return if value is the same
            if (value == device->sensor->config[id].value) {
                return;
            }
            break;
        case CONFIG_CMD_RESET:
            // Return if value is the same
            if (device->sensor->config[id].value == device->sensor->config[id].value_default) {
                device->config_data.value = device->sensor->config[id].value_default;
                return;
            }
            else {
                // Set the value to factory default
                value = device->sensor->config[id].value_default;
            }
            break;
        case CONFIG_CMD_STORE:
            return;
        default:
            device->config_data.status = CONFIG_STAT_INVALID;
            return;
    }

    // Try to set the requested value
    uint8_t status = set_config(device->sensor, id, value);

    // Load configuration
    load_config(device->sensor, id);

    // Override the value to send back
    device->config_data.value = device->sensor->config[id].value;

    // Check status
    if (status) {
        device->config_data.status = CONFIG_STAT_ERROR;
        return;
    }
    else if (CONFIG_CMD_SET == cmd
            && value != device->sensor->config[id].value
            && device->sensor->config[id].value != INVALID_CONFIG_VALUE) {
        device->config_data.status = CONFIG_STAT_MISMATCH;
        return;
    }
    else {
        return;
    }
}

static uint8_t set_config(snsr_data_t *sensor, uint8_t id, int32_t value) {
    uint8_t status = CONFIG_STAT_OK;

    switch (id) {
        /** Configurations */
        case CONFIG_POWER_LEVEL:
            status = VL53LX_SetVCSELPowerAdjust(VL53LX(sensor), value);
            break;
        case CONFIG_PHASECAL_PATCH_PWR:
            status = VL53LX_SetTuningParameter(VL53LX(sensor), VL53LX_TUNINGPARM_PHASECAL_PATCH_POWER, value);
            break;
        case CONFIG_TIME_BUDGET:
            status = VL53LX_SetMeasurementTimingBudgetMicroSeconds(VL53LX(sensor), (uint32_t) value);
            break;
        case CONFIG_OFFSET_MODE:
            status = VL53LX_SetOffsetCorrectionMode(VL53LX(sensor), (uint8_t) value);
            break;
        case CONFIG_DISTANCE_MODE:
            status = VL53LX_SetDistanceMode(VL53LX(sensor), (uint8_t) value);
            break;
        case CONFIG_SMUDGE_CORR_EN:
            status = VL53LX_SmudgeCorrectionEnable(VL53LX(sensor), (uint8_t) value);
            if(!status){
                sensor->config[id].value = value;
            }
            break;
        case CONFIG_XTALK_COMP_EN:
            status = VL53LX_SetXTalkCompensationEnable(VL53LX(sensor), (uint8_t) value);
            break;
        case CONFIG_RECT_OF_INTEREST:{
            status = VL53LX_SetUserROI(VL53LX(sensor), (VL53LX_UserRoi_t*)&value);
            break;
        }
        /** Calibrations */
        case CONFIG_CAL_REFSPAD:
            status = VL53LX_PerformRefSpadManagement(VL53LX(sensor));
            break;
        case CONFIG_CAL_OFFSET_SIMPLE:
            status = VL53LX_PerformOffsetSimpleCalibration(VL53LX(sensor), value);
            break;
        case CONFIG_CAL_OFFSET_ZERO:
            status = VL53LX_PerformOffsetZeroDistanceCalibration(VL53LX(sensor));
            break;
        case CONFIG_CAL_OFFSET_VCSEL:
            status = VL53LX_PerformOffsetPerVcselCalibration(VL53LX(sensor), value);
            break;
        case CONFIG_CAL_XTALK:
            status = VL53LX_PerformXTalkCalibration(VL53LX(sensor));
            if(!status){
                // TimeBudget is modified by xtalk calibration so send notification
                device_t dev;
                dev.config_data.id = CONFIG_TIME_BUDGET;
                status = load_config(sensor, dev.config_data.id);
                // Notify of change only when in Ready status
                if (sensor->status == TOF_STATUS_READY) {
                    dev.config_data.cmd = CONFIG_CMD_GET;
                    dev.config_data.status = CONFIG_STAT_UPDATED;
                    dev.config_data.value = sensor->config[dev.config_data.id].value;
                    tof_data_callback(&dev, TOF_DATA_CONFIG);
                }
            }
            break;
        default:
            status = CONFIG_STAT_NA;
    }

    return status;
}

static uint8_t load_config(snsr_data_t *sensor, uint8_t id) {
    uint8_t status = CONFIG_STAT_OK;

    switch (id) {
        /** Configurations */
        case CONFIG_POWER_LEVEL: {
            uint8_t power_level;
            status = VL53LX_GetVCSELPowerAdjust(VL53LX(sensor), &power_level);
            if (!status) {
                sensor->config[id].value = (int32_t)power_level;
            }
            break;
        }
        case CONFIG_PHASECAL_PATCH_PWR:{
            int32_t pTuningParameterValue;
            status = VL53LX_GetTuningParameter(VL53LX(sensor), VL53LX_TUNINGPARM_PHASECAL_PATCH_POWER, &pTuningParameterValue);
            if (!status) {
                sensor->config[id].value = pTuningParameterValue;
            }
            break;
        }
        case CONFIG_TIME_BUDGET: {
            uint32_t time_budget;
            status = VL53LX_GetMeasurementTimingBudgetMicroSeconds(VL53LX(sensor), &time_budget);
            // Get the timing data
            if (!status) {
                sensor->config[id].value = (int32_t)time_budget;
            }
            break;
        }
        case CONFIG_OFFSET_MODE:
            sensor->config[id].value = (int32_t)VL53LX(sensor)->Data.LLData.offset_correction_mode;
            break;
        case CONFIG_DISTANCE_MODE:
            sensor->config[id].value = (int32_t)VL53LX(sensor)->Data.CurrentParameters.DistanceMode;
            break;
        case CONFIG_SMUDGE_CORR_EN:
            break;
        case CONFIG_XTALK_COMP_EN:{
            uint8_t xtalk_comp_en;
            status = VL53LX_GetXTalkCompensationEnable(VL53LX(sensor), &xtalk_comp_en);
            if (!status) {
                sensor->config[id].value = (int32_t)xtalk_comp_en;
            }
            break;
        }
        case CONFIG_RECT_OF_INTEREST:{
            VL53LX_UserRoi_t roi;
            status = VL53LX_GetUserROI(VL53LX(sensor), &roi);
            if (!status) {
                memcpy(&sensor->config[id].value, &roi, sizeof(VL53LX_UserRoi_t));
            }
            break;
        }
        /** Calibrations */
        case CONFIG_CAL_REFSPAD:
        case CONFIG_CAL_OFFSET_SIMPLE:
        case CONFIG_CAL_OFFSET_ZERO:
        case CONFIG_CAL_OFFSET_VCSEL:
        case CONFIG_CAL_XTALK:
            sensor->config[id].value = INVALID_CONFIG_VALUE;
            break;
        default:
            break;
    }

    return status;
}

static void get_store_data(snsr_data_t* sensor, store_data_t* data){
    data->power_level = sensor->config[CONFIG_POWER_LEVEL].value;
    data->phasecal_pwr_lvl = (uint8_t)sensor->config[CONFIG_PHASECAL_PATCH_PWR].value;
    data->en_smudge_corr = (uint8_t)sensor->config[CONFIG_SMUDGE_CORR_EN].value;
    data->en_xtalk_comp = (uint8_t)sensor->config[CONFIG_XTALK_COMP_EN].value;
    data->time_budget = sensor->config[CONFIG_TIME_BUDGET].value;
    data->offset_mode = sensor->config[CONFIG_OFFSET_MODE].value;
    data->distance_mode = sensor->config[CONFIG_DISTANCE_MODE].value;
    data->roi = (uint32_t)sensor->config[CONFIG_RECT_OF_INTEREST].value;

    VL53LX_GetCalibrationData(VL53LX(sensor), &data->cal);
}

static const char* get_config_str(uint8_t config) {
    switch (config) {
        case CONFIG_POWER_LEVEL:
            return "pwr lvl";
        case CONFIG_PHASECAL_PATCH_PWR:
            return "patch pwr";
        case CONFIG_TIME_BUDGET:
            return "time budget";
        case CONFIG_OFFSET_MODE:
            return "offset mode";
        case CONFIG_DISTANCE_MODE:
            return "dist mode";
        case CONFIG_SMUDGE_CORR_EN:
            return "smudge corr en";
        case CONFIG_XTALK_COMP_EN:
            return "xtalk comp en";
        case CONFIG_RECT_OF_INTEREST:
            return "roi";
        case CONFIG_CAL_REFSPAD:
            return "ref spad";
        case CONFIG_CAL_OFFSET_SIMPLE:
            return "offset simple";
        case CONFIG_CAL_OFFSET_ZERO:
            return "offset zero";
        case CONFIG_CAL_OFFSET_VCSEL:
            return "offst vcsel";
        case CONFIG_CAL_XTALK:
            return "xtalk";
        default:
            return "unknown";
    }
}
