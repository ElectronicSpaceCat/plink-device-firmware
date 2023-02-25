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

#ifndef BLE_PWR_SERVICE_H__
#define BLE_PWR_SERVICE_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

// TODO: AG - Make sure to generate unique 128-bit UUID here
// Defining 128-bit base UUIDs
#define BLE_UUID_BASE_PWR                           \
{{                                                  \
    0x23, 0xD2, 0x15, 0xEF, 0x5F, 0x78, 0x23, 0x15, \
    0xDA, 0xFA, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00  \
}}

#ifndef BLE_PWR_BLE_OBSERVER_PRIO
#define BLE_PWR_BLE_OBSERVER_PRIO 2
#endif

#define BLE_PWR_DEF(_name)                          \
static ble_pwr_t _name;                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                 \
                     BLE_PWR_BLE_OBSERVER_PRIO,     \
                     ble_pwr_on_ble_evt,            \
                     &_name)

#define BLE_UUID_PWR_SERVICE        0xF0DD

// Defining 16-bit characteristic UUID
#define BLE_UUID_PWR_SOURCE         0xBEBA // TODO: AG - Make sure the 16-bit char UUID does not conflict with registered ones
#define BLE_UUID_PWR_BATT_STATUS    0xBEBB // TODO: AG - Make sure the 16-bit char UUID does not conflict with registered ones
#define BLE_UUID_PWR_BATT_LEVEL     0xBEBC // TODO: AG - Make sure the 16-bit char UUID does not conflict with registered ones

// Forward declaration of the ble_pwr_t type.
typedef struct ble_pwr_s ble_pwr_t;

typedef void (*ble_pwr_evt_handler_t) (ble_pwr_t* p_pwr, ble_evt_t const *p_ble_evt);

typedef struct
{
    ble_pwr_evt_handler_t       evt_handler;
    bool                        support_notification;  /**< TRUE if notification of Battery Level measurement is supported. */
} ble_pwr_init_t;

// This structure contains various status information for our service.
// The name is based on the naming convention used in Nordics SDKs.
// 'ble’ indicates that it is a Bluetooth Low Energy relevant structure.
struct ble_pwr_s
{
    ble_pwr_evt_handler_t       evt_handler;
    uint16_t                    conn_handle;    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
    bool                        is_notification_supported;
    uint16_t                    service_handle; /**< Handle of Our Service (as provided by the BLE stack). */
	ble_gatts_char_handles_t    pwr_source_char_handles;
    ble_gatts_char_handles_t    pwr_batt_status_char_handles;
    ble_gatts_char_handles_t    pwr_batt_level_char_handles;
};

/**@brief Function for handling BLE Stack events related to our service and characteristic.
 *
 * @details Handles all events from the BLE stack of interest to Our Service.
 *
 * @param[in]   p_pwr                    Our Service structure.
 * @param[in]   p_ble_evt                Event received from the BLE stack.
 */
void ble_pwr_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

/**@brief Function for initializing our new service.
 *
 * @param[in]   p_pwr                    Pointer to Our Service structure.
 */
ret_code_t ble_pwr_init(ble_pwr_t *p_pwr, const ble_pwr_init_t * p_pwr_init);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_pwr                    Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
void pwr_source_characteristic_update(ble_pwr_t *p_pwr, uint8_t *pwr_source_value);


/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_pwr                    Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
void pwr_batt_status_characteristic_update(ble_pwr_t *p_pwr, uint8_t *batt_status_value);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_pwr                    Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
void pwr_batt_level_characteristic_update(ble_pwr_t *p_pwr, uint32_t *batt_volts);


void pwr_gatts_hvx_debug_set(uint8_t value);

#endif  /* _ BLE_PWR_SERVICE_H__ */
