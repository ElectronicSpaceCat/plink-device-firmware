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

#ifndef BLE_TOF_SERVICE_H__
#define BLE_TOF_SERVICE_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

// TODO: AG - Make sure to generate unique 128-bit UUID here
// Defining 128-bit base UUIDs
#define BLE_UUID_BASE_TOF                           \
{{                                                  \
    0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, \
    0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00  \
}}

#ifndef BLE_TOF_BLE_OBSERVER_PRIO
#define BLE_TOF_BLE_OBSERVER_PRIO 2
#endif

#define BLE_TOF_DEF(_name)                          \
static ble_tof_t _name;                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                 \
                     BLE_TOF_BLE_OBSERVER_PRIO,     \
                     ble_tof_on_ble_evt,            \
                     &_name)

#define BLE_UUID_TOF_SERVICE             0xF00D

// Defining 16-bit characteristic UUID
#define BLE_UUID_TOF_RANGE               0xBEAA // TODO: AG - Make sure the 16-bit char UUID does not conflict with registered ones
#define BLE_UUID_TOF_SELECT              0xBEAB // TODO: AG - Make sure the 16-bit char UUID does not conflict with registered ones
#define BLE_UUID_TOF_CONFIG              0xBEAC // TODO: AG - Make sure the 16-bit char UUID does not conflict with registered ones
#define BLE_UUID_TOF_STATUS              0xBEAD // TODO: AG - Make sure the 16-bit char UUID does not conflict with registered ones
#define BLE_UUID_TOF_RANGING_ENABLE      0xBEAE // TODO: AG - Make sure the 16-bit char UUID does not conflict with registered ones
#define BLE_UUID_TOF_RESET               0xBEAF // TODO: AG - Make sure the 16-bit char UUID does not conflict with registered ones

// Forward declaration of the ble_tof_t type.
typedef struct ble_tof_s ble_tof_t;

typedef void (*ble_tof_evt_handler_t) (ble_tof_t* p_tof, ble_evt_t const *p_ble_evt);

typedef void (*ble_tof_write_handler_t) (uint16_t conn_handle, uint8_t data);
typedef void (*ble_tof_write_config_handler_t) (uint16_t conn_handle, uint8_t cmd, uint8_t id, int32_t value);

typedef struct
{
    ble_tof_evt_handler_t            evt_handler;
    bool                             support_notification;           /**< TRUE if notification of Battery Level measurement is supported. */
    ble_tof_write_handler_t          tof_select_write_handler;
    ble_tof_write_config_handler_t   tof_config_write_handler;
    ble_tof_write_handler_t          tof_ranging_enable_write_handler;
    ble_tof_write_handler_t          tof_reset_write_handler;
} ble_tof_init_t;

// This structure contains various status information for our service.
// The name is based on the naming convention used in Nordics SDKs.
// 'ble’ indicates that it is a Bluetooth Low Energy relevant structure.
struct ble_tof_s
{
    ble_tof_evt_handler_t          evt_handler;
    uint16_t                       conn_handle;    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
    uint16_t                       service_handle; /**< Handle of Our Service (as provided by the BLE stack). */
    bool                           is_notification_supported;
    // ToF range handles
	ble_gatts_char_handles_t       tof_range_char_handles;
    // ToF sensor select handles
    ble_gatts_char_handles_t       tof_select_char_handles;
    ble_tof_write_handler_t        tof_select_write_handler;
    // ToF sensor config handles
    ble_gatts_char_handles_t       tof_config_char_handles;
    ble_tof_write_config_handler_t tof_config_write_handler;
    // ToF sensor status handles
    ble_gatts_char_handles_t       tof_status_char_handles;
    // ToF ranging_enable handles
    ble_gatts_char_handles_t       tof_ranging_enable_char_handles;
    ble_tof_write_handler_t        tof_ranging_enable_write_handler;
    // ToF sensor reset
    ble_gatts_char_handles_t       tof_reset_char_handles;
    ble_tof_write_handler_t        tof_reset_write_handler;
};

/**@brief Function for handling BLE Stack events related to our service and characteristic.
 *
 * @details Handles all events from the BLE stack of interest to Our Service.
 *
 * @param[in]   p_our_service       Our Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_tof_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

/**@brief Function for initializing our new service.
 *
 * @param[in]   p_tof       Pointer to Our Service structure.
 */
ret_code_t ble_tof_init(ble_tof_t *p_tof, const ble_tof_init_t * p_tof_init);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_tof            Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
void tof_select_characteristic_update(ble_tof_t *p_tof, uint8_t sensor, uint8_t sensor_type);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_tof            Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
void tof_config_characteristic_update(ble_tof_t *p_tof, uint8_t cmd, uint8_t id, int32_t value, uint8_t status);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_tof            Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
void tof_status_characteristic_update(ble_tof_t *p_tof, uint8_t *tof_status_value);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_tof            Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
void tof_range_characteristic_update(ble_tof_t *p_tof, uint16_t *tof_range_value);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_tof            Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
void tof_ranging_enable_characteristic_update(ble_tof_t *p_tof, uint8_t * tof_sampling_enabled);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_tof            Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
void tof_reset_characteristic_update(ble_tof_t *p_tof, uint8_t * value);

/**@brief Function for updating and sending hvx Indications
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_tof            Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
void tof_hvx_gatts_queue_process(void);

void tof_gatts_hvx_debug_set(uint8_t value);

#endif  /* _ BLE_TOF_SERVICE_H__ */
