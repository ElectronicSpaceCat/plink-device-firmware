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

#include <stdint.h>
#include <string.h>
#include "ble_tof_service.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "tof_state_machine.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "nrf_queue.h"

#include "timer_delay.h"

static uint8_t is_debug_enabled = false;

static uint8_t is_tof_range_enabled = false;
static uint8_t is_tof_select_enabled = false;
static uint8_t is_tof_status_enabled = false;
static uint8_t is_tof_rng_en_enabled = false;
static uint8_t is_tof_config_enabled = false;
static uint8_t is_tof_reset_enabled = false;

static volatile uint8_t m_hvc_active = false;

#define HVX_BUFF_SIZE 15
#define HVX_DATA_BUFF_SIZE 10

typedef struct {
    uint8_t idx;
    uint8_t data[HVX_DATA_BUFF_SIZE];
    uint16_t* conn_handle;
    uint16_t data_len;
    ble_gatts_hvx_params_t hvx_params;
}hvx_queue_t;

NRF_QUEUE_DEF(hvx_queue_t, m_hvx, HVX_BUFF_SIZE, NRF_QUEUE_MODE_NO_OVERFLOW);

static void hvx_gatts_queue_push(uint16_t* conn_handle, ble_gatts_hvx_params_t *p_hvx_params, size_t data_len);
static void characteristic_update(ble_tof_t *p_service, ble_gatts_char_handles_t *char_handle, uint8_t type, uint8_t *data, size_t data_len);
static void characteristic_set(ble_tof_t *p_service, ble_gatts_char_handles_t *char_handle, uint8_t *data, size_t data_len, uint16_t offset);

static void check_cccds_enabled(ble_gatts_evt_write_t const *p_evt_write, ble_tof_t *p_tof, ble_evt_t const *p_ble_evt) {
    if(p_evt_write->len != 2) return;

    if (p_evt_write->handle == p_tof->tof_range_char_handles.cccd_handle) {
        is_tof_range_enabled = ble_srv_is_notification_enabled(p_evt_write->data);
    }
    if (p_evt_write->handle == p_tof->tof_select_char_handles.cccd_handle) {
        is_tof_select_enabled = ble_srv_is_indication_enabled(p_evt_write->data);
    }
    if (p_evt_write->handle == p_tof->tof_config_char_handles.cccd_handle) {
        is_tof_config_enabled = ble_srv_is_indication_enabled(p_evt_write->data);
    }
    if (p_evt_write->handle == p_tof->tof_status_char_handles.cccd_handle) {
        is_tof_status_enabled = ble_srv_is_indication_enabled(p_evt_write->data);
    }
    if (p_evt_write->handle == p_tof->tof_ranging_enable_char_handles.cccd_handle) {
        is_tof_rng_en_enabled = ble_srv_is_indication_enabled(p_evt_write->data);
    }
    if (p_evt_write->handle == p_tof->tof_reset_char_handles.cccd_handle) {
        is_tof_reset_enabled = ble_srv_is_indication_enabled(p_evt_write->data);
    }
}

static void on_write(ble_tof_t *p_tof, ble_evt_t const *p_ble_evt) {
    ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    // Check the cccds indication enable status
    check_cccds_enabled(p_evt_write, p_tof, p_ble_evt);

    // ToF sensor select write handle
    if (p_evt_write->handle == p_tof->tof_select_char_handles.value_handle) {
        if ((p_evt_write->len == 1) && (p_tof->tof_select_write_handler != NULL)) {
            p_tof->tof_select_write_handler(p_ble_evt->evt.gap_evt.conn_handle, p_evt_write->data[0]);
        }
    }
    // ToF sensor config write handle
    else if (p_evt_write->handle == p_tof->tof_config_char_handles.value_handle) {
        if ((p_evt_write->len == 6) && (p_tof->tof_config_write_handler != NULL)) {
            int32_t value;
            memcpy(&value, &p_evt_write->data[2], sizeof(int32_t));
            p_tof->tof_config_write_handler(p_ble_evt->evt.gap_evt.conn_handle, p_evt_write->data[0], p_evt_write->data[1], value);
        }
    }
    // ToF sensor ranging enable write handle
    else if (p_evt_write->handle == p_tof->tof_ranging_enable_char_handles.value_handle) {
        if ((p_evt_write->len == 1) && (p_tof->tof_ranging_enable_write_handler != NULL)) {
            p_tof->tof_ranging_enable_write_handler(p_ble_evt->evt.gap_evt.conn_handle, p_evt_write->data[0]);
        }
    }
    // ToF sensor reset write handle
    else if (p_evt_write->handle == p_tof->tof_reset_char_handles.value_handle) {
        if ((p_evt_write->len == 1) && (p_tof->tof_reset_write_handler != NULL)) {
            p_tof->tof_reset_write_handler(p_ble_evt->evt.gap_evt.conn_handle, p_evt_write->data[0]);
        }
    }
}

// Declaration of a function that will take care of some housekeeping of ble connections related to our service and characteristic
void ble_tof_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context) {
    ble_tof_t *p_tof = (ble_tof_t*) p_context;

    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
            p_tof->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            is_tof_range_enabled = false;
            is_tof_select_enabled = false;
            is_tof_config_enabled = false;
            is_tof_status_enabled = false;
            is_tof_rng_en_enabled = false;
            is_tof_reset_enabled = false;

            p_tof->conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        case BLE_GATTS_EVT_WRITE:
            on_write(p_tof, p_ble_evt);
            break;
        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            break;
        case BLE_GATTS_EVT_TIMEOUT:
            // Set flag false and let the queue manager re-send the last indication
            m_hvc_active = false;
            break;
        case BLE_GATTS_EVT_HVC:{
            m_hvc_active = false;
            // Only pop data off the queue if indication was received
            hvx_queue_t hvx;
            if(NRF_SUCCESS == nrf_queue_generic_pop(&m_hvx, &hvx, false)){
                if (is_debug_enabled) {
                    NRF_LOG_INFO(
                            "tof hvx handle: 0x%X de-queued: %d/%d, status: %d",
                            p_ble_evt->evt.gatts_evt.params.hvc.handle,
                            hvx.idx,
                            m_hvx.size,
                            p_ble_evt->header.evt_id);
                }
            }
            break;
        }
        default:
            // No implementation needed
            break;
    }

    // Call application event handler.
    if (p_tof->evt_handler != NULL) {
        p_tof->evt_handler(p_tof, p_ble_evt);
    }
}

/**@brief Function for adding our new characteristic to "Our service" that we initiated in the previous tutorial.
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
static uint32_t our_char_add(ble_tof_t *p_tof, uint16_t uuid, bool read, bool write, uint8_t type, uint16_t value_size, uint32_t value_init, ble_gatts_char_handles_t *char_handle) {
    // Add a custom characteristic UUID
    uint32_t err_code;
    ble_uuid_t char_uuid;

    ble_uuid128_t base_uuid = BLE_UUID_BASE_TOF;
    char_uuid.uuid = uuid;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code);

    // Add read/write properties to our characteristic
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = read;
    char_md.char_props.write = write;

    // Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    char_md.p_cccd_md = &cccd_md;
    if (type == BLE_GATT_HVX_NOTIFICATION) {
        char_md.char_props.notify = true;
    }
    if (type == BLE_GATT_HVX_INDICATION) {
        char_md.char_props.indicate = true;
    }

    // Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc = BLE_GATTS_VLOC_STACK;

    // Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    // Configure the characteristic value attribute
    ble_gatts_attr_t attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;

    // Set characteristic length in number of bytes
    attr_char_value.max_len = value_size;
    attr_char_value.init_len = value_size;
    uint8_t buff[value_size];
    int32_t value = value_init;
    if (value_size > sizeof(value_init)) {
        memset(&buff[0], value_init, value_size);
    }
    else {
        memcpy(&buff[0], &value, value_size);
    }
    attr_char_value.p_value = &buff[0];

    // Add our new characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_tof->service_handle, &char_md, &attr_char_value, char_handle);

    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}

/**@brief Function for initiating our new service.
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
ret_code_t ble_tof_init(ble_tof_t *p_tof, const ble_tof_init_t *p_tof_init) {
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions

    // Declare 16-bit service and 128-bit base UUIDs and add them to the BLE stack
    ble_uuid_t service_uuid;
    ble_uuid128_t base_uuid = BLE_UUID_BASE_TOF;
    service_uuid.uuid = BLE_UUID_TOF_SERVICE;

    p_tof->evt_handler = p_tof_init->evt_handler;
    p_tof->is_notification_supported = p_tof_init->support_notification;
    p_tof->tof_select_write_handler = p_tof_init->tof_select_write_handler;
    p_tof->tof_config_write_handler = p_tof_init->tof_config_write_handler;
    p_tof->tof_ranging_enable_write_handler = p_tof_init->tof_ranging_enable_write_handler;
    p_tof->tof_reset_write_handler = p_tof_init->tof_reset_write_handler;

    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
    APP_ERROR_CHECK(err_code);

    // Set our service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
    p_tof->conn_handle = BLE_CONN_HANDLE_INVALID;

    // Add our service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &(p_tof->service_handle));

    APP_ERROR_CHECK(err_code);

    // Call the function our_char_add() to add our new characteristic to the service.
    our_char_add(p_tof, BLE_UUID_TOF_RANGE, true, false, BLE_GATT_HVX_NOTIFICATION, sizeof(uint16_t), 0xFFFF, &p_tof->tof_range_char_handles);
    our_char_add(p_tof, BLE_UUID_TOF_SELECT, true, true, BLE_GATT_HVX_INDICATION, sizeof(uint16_t), 0xFFFF, &p_tof->tof_select_char_handles);
    our_char_add(p_tof, BLE_UUID_TOF_STATUS, true, false, BLE_GATT_HVX_INDICATION, sizeof(uint8_t), 0xFF, &p_tof->tof_status_char_handles);
    our_char_add(p_tof, BLE_UUID_TOF_RANGING_ENABLE, true, true, BLE_GATT_HVX_INDICATION, sizeof(uint8_t), 0xFF, &p_tof->tof_ranging_enable_char_handles);
    our_char_add(p_tof, BLE_UUID_TOF_RESET, true, true, BLE_GATT_HVX_INDICATION, sizeof(uint8_t), 0xFF, &p_tof->tof_reset_char_handles);
    our_char_add(p_tof, BLE_UUID_TOF_CONFIG, true, true, BLE_GATT_HVX_INDICATION, 7, 0xFF, &p_tof->tof_config_char_handles);

    return err_code;
}

// Update tof range data, notify if characteristic is enabled
void tof_range_characteristic_update(ble_tof_t *p_tof, uint16_t *tof_range_value) {
    if(is_tof_range_enabled){
        characteristic_update(p_tof, &p_tof->tof_range_char_handles, BLE_GATT_HVX_NOTIFICATION, (uint8_t*) tof_range_value, sizeof(uint16_t));
    }
    else{
        characteristic_set(p_tof, &p_tof->tof_range_char_handles, (uint8_t*) tof_range_value, sizeof(uint16_t), 0);
    }
}

// Update tof select data, notify if characteristic is enabled
void tof_select_characteristic_update(ble_tof_t *p_tof, uint8_t sensor, uint8_t sensor_type) {
    uint16_t data = 0;
    data = (0x00FF & sensor);
    data |= (0xFF00 & (sensor_type << 8));

    if(is_tof_select_enabled){
        characteristic_update(p_tof, &p_tof->tof_select_char_handles, BLE_GATT_HVX_INDICATION, (uint8_t*) &data, sizeof(uint16_t));
    }
    else{
        characteristic_set(p_tof, &p_tof->tof_select_char_handles, (uint8_t*) &data, sizeof(uint16_t), 0);
    }
}

// Update tof config data, notify if characteristic is enabled
void tof_config_characteristic_update(ble_tof_t *p_tof, uint8_t cmd, uint8_t id, int32_t value, uint8_t status) {
    static uint16_t buff_size = 7;
    uint8_t data[buff_size];
    data[0] = cmd;
    data[1] = id;
    memcpy(&data[2], &value, 4);
    data[6] = status;

    if(is_tof_config_enabled){
        characteristic_update(p_tof, &p_tof->tof_config_char_handles, BLE_GATT_HVX_INDICATION, data, buff_size);
    }
    else{
        characteristic_set(p_tof, &p_tof->tof_config_char_handles, data, buff_size, 0);
    }
}

// Update tof status data, notify if characteristic is enabled
void tof_status_characteristic_update(ble_tof_t *p_tof, uint8_t *tof_status_value) {
    if(is_tof_status_enabled){
        characteristic_update(p_tof, &p_tof->tof_status_char_handles, BLE_GATT_HVX_INDICATION, (uint8_t*) tof_status_value, sizeof(uint8_t));
    }
    else{
        characteristic_set(p_tof, &p_tof->tof_status_char_handles, (uint8_t*) tof_status_value, sizeof(uint8_t), 0);
    }
}

// Update tof range enable data, notify if characteristic is enabled
void tof_ranging_enable_characteristic_update(ble_tof_t *p_tof, uint8_t *tof_sampling_enabled) {
    if(is_tof_rng_en_enabled){
        characteristic_update(p_tof, &p_tof->tof_ranging_enable_char_handles, BLE_GATT_HVX_INDICATION, (uint8_t*) tof_sampling_enabled, sizeof(uint8_t));
    }
    else{
        characteristic_set(p_tof, &p_tof->tof_ranging_enable_char_handles, (uint8_t*) tof_sampling_enabled, sizeof(uint8_t), 0);
    }
}

// Update tof reset data, notify if characteristic is enabled
void tof_reset_characteristic_update(ble_tof_t *p_tof, uint8_t *reset) {
    if(is_tof_reset_enabled){
        characteristic_update(p_tof, &p_tof->tof_reset_char_handles, BLE_GATT_HVX_INDICATION, (uint8_t*) reset, sizeof(uint8_t));
    }
    else{
        characteristic_set(p_tof, &p_tof->tof_reset_char_handles, (uint8_t*) reset, sizeof(uint8_t), 0);
    }
}

static void characteristic_update(ble_tof_t *p_service, ble_gatts_char_handles_t *char_handle, uint8_t type, uint8_t *data, size_t data_len) {
    if (p_service->conn_handle != BLE_CONN_HANDLE_INVALID) {
        uint16_t len = data_len;
        ble_gatts_hvx_params_t hvx_params;

        hvx_params.handle = char_handle->value_handle;
        hvx_params.type = type;
        hvx_params.offset = 0;
        hvx_params.p_len = &len;
        hvx_params.p_data = data;

        switch(type) {
            // Notifications are allowed to send regardless of hvx status
            case BLE_GATT_HVX_NOTIFICATION:{
                uint32_t status = sd_ble_gatts_hvx(p_service->conn_handle, &hvx_params);
                if (is_debug_enabled) {
                    NRF_LOG_INFO("gatts_hvs: tof handle: 0x%X, status: %d", char_handle->value_handle, status);
                }
                break;
            }
            // Indications require queuing if too many are sent to the ble manager
            // and queuing helps maintain complete transfers. Indication will be ignored if queue is full.
            case BLE_GATT_HVX_INDICATION:
                hvx_gatts_queue_push(&p_service->conn_handle, &hvx_params, data_len);
                break;
            default:
                break;
        }
    }
}

static void hvx_gatts_queue_push(uint16_t* conn_handle, ble_gatts_hvx_params_t *p_hvx_params, size_t data_len) {
    if(nrf_queue_is_full(&m_hvx) || data_len > HVX_DATA_BUFF_SIZE){
        return;
    }

    hvx_queue_t hvx;

    // Determine the queue idx by peaking the front of the queue
    // and taking the difference of the control-block's back index
    // from the front index queue and adding 1 to start it at 1/BuffSize
    // for debug info.
    if(NRF_SUCCESS == nrf_queue_generic_pop(&m_hvx, &hvx, true)){
        hvx.idx = (abs(m_hvx.p_cb->back - m_hvx.p_cb->front) + 1);
    }
    // If queue is empty then set the index to 1
    else{
        hvx.idx = 1;
    }

    // Load hvx data
    hvx.conn_handle = conn_handle;
    hvx.data_len = data_len;
    memcpy(hvx.data, p_hvx_params->p_data, hvx.data_len);
    hvx.hvx_params.handle = p_hvx_params->handle;
    hvx.hvx_params.type = p_hvx_params->type;
    hvx.hvx_params.offset = p_hvx_params->offset;

    // Cannot assigned the hvx data below:
    //
    // hvx.hvx_params.p_data = hvx.data;
    // hvx.hvx_params.p_len  = &hvx.data_len
    //
    // Reason:
    // The p_data and p_len are pointers to data addresses.
    // If hvx data is put in queue slot 1 and then a consecutive
    // call moves slot 1 data to slot 2 data, then the pointer values would still be
    // pointing to data within slot 1.
    //
    // Why?:
    // The queue manager does a memcpy of the data from slot to slot
    // (i.e. a shallow copy), therefore not transferring pointer references.

    // Push hvx data on queue
    if(NRF_SUCCESS == nrf_queue_push(&m_hvx, &hvx)){
        if (is_debug_enabled) {
            NRF_LOG_INFO("tof hvx handle: 0x%X queued at: %d/%d", hvx.hvx_params.handle, hvx.idx, m_hvx.size);
        }
    }
}

static void characteristic_set(ble_tof_t *p_service, ble_gatts_char_handles_t *char_handle, uint8_t *data, size_t data_len, uint16_t offset){
    ble_gatts_value_t p_value = {
            .len = data_len,
            .offset = offset,
            .p_value = data
    };
    sd_ble_gatts_value_set(p_service->conn_handle, char_handle->value_handle, &p_value);
}

void tof_hvx_gatts_queue_process(void){
    // Return if an hvx indication is in process or no data in the queue
    if(m_hvc_active || nrf_queue_is_empty(&m_hvx)) return;

    // Peak the data in the queue
    hvx_queue_t hvx;
    (void)nrf_queue_generic_pop(&m_hvx, &hvx, true);
    hvx.hvx_params.p_data = hvx.data; // Data pointer has to be set here
    hvx.hvx_params.p_len = &hvx.data_len; // Data length pointer has to be set here

    // Send the hvx indication data
    uint32_t status = sd_ble_gatts_hvx(*hvx.conn_handle, &hvx.hvx_params);

    if (is_debug_enabled) {
        NRF_LOG_INFO("tof hvx handle: 0x%X tx-queued: %d/%d, status: %d", hvx.hvx_params.handle, hvx.idx, m_hvx.size, status);
    }
    // Is status == NRF_SUCCESS
    if(NRF_SUCCESS == status){
        if (is_debug_enabled) {
            NRF_LOG_FLUSH();
        }
        // Yes - Set flag and wait for BLE_GATTS_EVT_TIMEOUT or BLE_GATTS_EVT_HVC response
        m_hvc_active = true;
        wfe(&m_hvc_active);
    }
}

void tof_gatts_hvx_debug_set(uint8_t value) {
    is_debug_enabled = value;
}
