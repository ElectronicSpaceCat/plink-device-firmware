// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#ifdef _MSC_VER
#define snprintf _snprintf
#endif

#include "vl53lx_platform.h"
#include <vl53lx_platform_log.h>

#include "app_timer.h"
#include "timer_delay.h"

#define VL53LX_get_register_name(VL53LX_p_007,VL53LX_p_032) VL53LX_COPYSTRING(VL53LX_p_032, "");

#define trace_print(level, ...) \
	_LOG_TRACE_PRINT(VL53LX_TRACE_MODULE_PLATFORM, \
	level, VL53LX_TRACE_FUNCTION_NONE, ##__VA_ARGS__)

#define trace_i2c(...) \
	_LOG_TRACE_PRINT(VL53LX_TRACE_MODULE_NONE, \
	VL53LX_TRACE_LEVEL_NONE, VL53LX_TRACE_FUNCTION_I2C, ##__VA_ARGS__)

VL53LX_Error VL53LX_WriteMulti(VL53LX_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count){
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    int32_t status = 0;
    uint8_t buffer[count + 2];

    buffer[0] = index >> 8;
    buffer[1] = index & 0x00FF;

    // Copy contents of pdata to buffer starting after the index
    memcpy(&buffer[2], pdata, count);

    status = I2CWrite(Dev->i2c_slave_address, (uint8_t*) buffer, count + 2);

    if (status) {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
    }

    return Status;
}

VL53LX_Error VL53LX_ReadMulti(VL53LX_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count){
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    int32_t status;
    uint8_t buffer[2];

    buffer[0] = index >> 8;
    buffer[1] = index & 0x00FF;

    status = I2CWrite(Dev->i2c_slave_address, buffer, (uint8_t) 2);
    if (!status) {
        status = I2CRead(Dev->i2c_slave_address, pdata, count);
    }

    if (status) {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
    }

    return Status;
}

VL53LX_Error VL53LX_WrByte(VL53LX_DEV Dev, uint16_t index, uint8_t data){
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    int32_t status;
    uint8_t buffer[3];

    buffer[0] = index >> 8;
    buffer[1] = index & 0x00FF;
    buffer[2] = data;

    status = I2CWrite(Dev->i2c_slave_address, buffer, (uint8_t) 3);

    if (status) {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
    }

    return Status;
}

VL53LX_Error VL53LX_WrWord(VL53LX_DEV Dev, uint16_t index, uint16_t data){
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    int32_t status;
    uint8_t buffer[4];

    buffer[0] = index >> 8;
    buffer[1] = index & 0x00FF;
    buffer[2] = data >> 8;
    buffer[3] = data & 0x00FF;

    status = I2CWrite(Dev->i2c_slave_address, buffer, (uint8_t) 4);

    if (status) {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
    }

    return Status;
}

VL53LX_Error VL53LX_WrDWord(VL53LX_DEV Dev, uint16_t index, uint32_t data){
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    int32_t status;
    uint8_t buffer[6];

    buffer[0] = index >> 8;
    buffer[1] = index & 0x00FF;
    buffer[2] = (data >> 24) & 0xFF;
    buffer[3] = (data >> 16) & 0xFF;
    buffer[4] = (data >> 8) & 0xFF;
    buffer[5] = (data >> 0) & 0xFF;

    status = I2CWrite(Dev->i2c_slave_address, buffer, (uint8_t) 6);

    if (status) {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
    }

    return Status;
}

VL53LX_Error VL53LX_UpdateByte(VL53LX_DEV Dev, uint16_t index, uint8_t AndData, uint8_t OrData){
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    int32_t status;
    uint8_t data = 0;

    status = VL53LX_RdByte(Dev, index, &data);
    if (status) {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
    }

    if (Status == VL53LX_ERROR_NONE) {
        data = (data & AndData) | OrData;
        status = VL53LX_WrByte(Dev, index, data);
        if (status) {
            Status = VL53LX_ERROR_CONTROL_INTERFACE;
        }
    }

    return Status;
}

VL53LX_Error VL53LX_RdByte(VL53LX_DEV Dev, uint16_t index, uint8_t *data){
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    int32_t status;
    uint8_t buffer[2];

    buffer[0] = index >> 8;
    buffer[1] = index & 0x00FF;

    status = I2CWrite(Dev->i2c_slave_address, buffer, (uint8_t) 2);
    if (!status) {
        status = I2CRead(Dev->i2c_slave_address, buffer, (uint8_t) 1);
        if (!status) {
            *data = buffer[0];
        }
    }

    if (status) {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
    }

    return Status;
}

VL53LX_Error VL53LX_RdWord(VL53LX_DEV Dev, uint16_t index, uint16_t *data){
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    int32_t status;
    uint8_t buffer[2];

    buffer[0] = index >> 8;
    buffer[1] = index & 0x00FF;

    status = I2CWrite(Dev->i2c_slave_address, buffer, (uint8_t) 2);
    if (!status) {
        status = I2CRead(Dev->i2c_slave_address, buffer, (uint8_t) 2);
        if (!status) {
            /* Registers are Big endian if cpu is be direct read direct into *data is possible */
            *data = ((uint16_t) buffer[0] << 8) | (uint16_t) buffer[1];
        }
    }

    return Status;
}

VL53LX_Error VL53LX_RdDWord(VL53LX_DEV Dev, uint16_t index, uint32_t *data){
    VL53LX_Error Status = VL53LX_ERROR_NONE;
    int32_t status;
    uint8_t buffer[4];

    buffer[0] = index >> 8;
    buffer[1] = index & 0x00FF;

    status = I2CWrite(Dev->i2c_slave_address, (uint8_t*) buffer, (uint8_t) 2);
    if (!status) {
        status = I2CRead(Dev->i2c_slave_address, buffer, 4);
        if (!status) {
            /* Registers are Big endian if cpu is be direct read direct into data is possible */
            *data = ((uint32_t) buffer[0] << 24) | ((uint32_t) buffer[1] << 16) | ((uint32_t) buffer[2] << 8) | ((uint32_t) buffer[3]);
        }
    }

    if (status) {
        Status = VL53LX_ERROR_CONTROL_INTERFACE;
    }

    return Status;
}

VL53LX_Error VL53LX_PollingDelay(VL53LX_DEV Dev){
    PollingDelayMS(2);
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitUs(VL53LX_DEV Dev, int32_t wait_us){
    PollingDelayUS(1); // 152us actual
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitMs(VL53LX_DEV Dev, int32_t wait_ms){
    PollingDelayMS(wait_ms);
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GetTickCount(VL53LX_Dev_t *pdev, uint32_t *ptick_count_ms){
    VL53LX_Error status = VL53LX_ERROR_NONE;
    (void) pdev;
    // NOTE: AG - When using a 32.768 lfclck we have to divide the ticks by ~32
    *ptick_count_ms = ((app_timer_cnt_get() * 1000) / 32768);
    return status;
}

VL53LX_Error VL53LX_WaitValueMaskEx(VL53LX_Dev_t *pdev, uint32_t timeout_ms, uint16_t index, uint8_t value, uint8_t mask, uint32_t poll_delay_ms){
    VL53LX_Error status = VL53LX_ERROR_NONE;
    uint32_t start_time_ms = 0;
    uint32_t current_time_ms = 0;
    uint8_t byte_value = 0;
    uint8_t found = 0;
#ifdef VL53LX_LOG_ENABLE
    uint32_t     trace_functions = 0;
#endif

    _LOG_STRING_BUFFER(register_name);

    SUPPRESS_UNUSED_WARNING(poll_delay_ms);

#ifdef VL53LX_LOG_ENABLE

    VL53LX_get_register_name(
            index,
            register_name);


    trace_i2c("WaitValueMaskEx(%5d, %s, 0x%02X, 0x%02X, %5d);\n",
        timeout_ms, register_name, value, mask, poll_delay_ms);
#endif

    VL53LX_GetTickCount(pdev, &start_time_ms);
    pdev->new_data_ready_poll_duration_ms = 0;

#ifdef VL53LX_LOG_ENABLE
    trace_functions = _LOG_GET_TRACE_FUNCTIONS();
#endif
    _LOG_SET_TRACE_FUNCTIONS(VL53LX_TRACE_FUNCTION_NONE);

    while ((status == VL53LX_ERROR_NONE) &&
            (pdev->new_data_ready_poll_duration_ms < timeout_ms) &&
            (found == 0))
    {
        status = VL53LX_RdByte(pdev, index, &byte_value);

        if ((byte_value & mask) == value){
            found = 1;
        }

        VL53LX_GetTickCount(pdev, &current_time_ms);
        pdev->new_data_ready_poll_duration_ms = current_time_ms - start_time_ms;
    }

    _LOG_SET_TRACE_FUNCTIONS(trace_functions);

    if (found == 0 && status == VL53LX_ERROR_NONE){
        status = VL53LX_ERROR_TIME_OUT;
    }

    return status;
}
