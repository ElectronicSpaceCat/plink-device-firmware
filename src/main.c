/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
 
/** @name Plink Application
 *
 * @brief Main file for the Plink application. This app interfaces over i2c with
 * ST Electronics Time of Flight (ToF) sensors and sends the data over bluetooth.
 * It also incorporate Nordic's Over the Air (OTA) Device Firmware Update (DFU)
 * capability for updating the application code.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "boards.h"

#include "nordic_common.h"

#include "nrf.h"
#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdm.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "nrf_power.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_bootloader_info.h"
#include "nrf_gpio.h"

#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_state.h"
#include "ble_dis.h"
//#include "ble_bas.h"
//#include "ble_tps.h"
#include "ble_dfu.h"

#include "peer_manager_handler.h"

#include "app_error.h"
#include "app_timer.h"

/// Our stuff
#include "tof_state_machine.h"
#include "tof_pwr_monitor.h"
#include "ble_tof_service.h"
#include "ble_pwr_service.h"

#include "rtt_debug_cmd.h"

#define APP_NOTIFY_DELETE_BONDS_BIT_MASK (0x02)
#define APP_NOTIFY_DELETE_BONDS_MASK (BOOTLOADER_DFU_GPREGRET2_MASK | APP_NOTIFY_DELETE_BONDS_BIT_MASK)
#define APP_NOTIFY_DELETE_BONDS (BOOTLOADER_DFU_GPREGRET2 | APP_NOTIFY_DELETE_BONDS_BIT_MASK)

/** Device advertising name */
#define DEVICE_NAME                     "Plink"                                 /**< Name of device. Will be included in the advertising data. */

/** @defgroup ble_dis_config Configuration for Device Information Service. @{ */
#define BLE_DIS_MANUFACTURER_NAME       "GreenTech"                             /**< Manufacturer Name String. */
#define BLE_DIS_MODEL_NUMBER            "V24"                                   /**< Model Number String. */ // TODO AG - Should read this from flash value
#define BLE_DIS_SERIAL_NUMBER           "12345"                                 /**< Serial Number String. */
#define BLE_DIS_HW_REVISION             "V5.0"                                  /**< Hardware Revision String. */
#define BLE_DIS_FW_REVISION             "1.0.0"                                 /**< Firmware Revision String. */
#define BLE_DIS_SW_REVISION             "1.0.0"                                 /**< Software Revision String. */
#define BLE_DIS_MANUFACTURER_ID         0x0000000000                            /**< Manufacturer ID for System ID. */
#define BLE_DIS_OU_ID                   0x000000                                /**< Organizationally unique ID for System ID. */
#define BLE_DIS_CERT_LIST               {0x00, 0x00, 0x00, 0x00}                /**< IEEE 11073-20601 Regulatory Certification Data List. */
#define BLE_DIS_VENDOR_ID               0x0000                                  /**< Vendor ID for PnP ID. */
#define BLE_DIS_PRODUCT_ID              0x0000                                  /**< Product ID for PnP ID. */
#define BLE_DIS_PRODUCT_VERSION         0x0000                                  /**< Product Version for PnP ID. */
/** @} */

/** @defgroup General setup parameters @{ */

#define APP_ADV_TX_PWR                  -16                                     /**< TX power level when advertising. Note: Power level at -40 does not work for the BC832*/
#define APP_CONN_TX_PWR                 -20                                     /**< TX power level when connected. */

#define APP_ADV_FAST_INTERVAL           300                                     /**< Advertising interval in units of 0.625 ms. (This value corresponds to 187.5 ms)*/
#define APP_ADV_FAST_DURATION           18000                                   /**< Advertising duration in units of 10 milliseconds. (This value corresponds to 3 min or 180 seconds)*/

//#define APP_ADV_SLOW_INTERVAL           3200                                    /**< Advertising interval in units of 0.625 ms. (This value corresponds to 2000 ms)*/
//#define APP_ADV_SLOW_DURATION           18000                                   /**< Advertising duration in units of 10 milliseconds. (This value corresponds to 3 min or 180 seconds)*/

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

// NOTE AG - Setting the Max intervals lower seems to help
//           with the data through put for this application
//           If streaming then larger would be required
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)        /**< Minimum acceptable connection interval. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(50, UNIT_1_25_MS)         /**< Maximum acceptable connection interval.  */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Key press notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */
/** @} */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

NRF_BLE_GATT_DEF(m_gatt); /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr); /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising); /**< Advertising module instance. */
//BLE_BAS_DEF(m_bas); /**< Battery service instance. */
BLE_TOF_DEF(m_tof); /**< ToF service instance. */
BLE_PWR_DEF(m_pwr); /**< Power service instance. */
//BLE_TPS_DEF(m_tps);                                                           /**< TX Power service instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */
static pm_peer_id_t m_peer_id; /**< Device reference handle to the current bonded central. */

static void advertising_start(bool erase_bonds); /**< Forward declaration of advertising start function */
static void whitelist_set(pm_peer_id_list_skip_t skip);
static void identities_set(pm_peer_id_list_skip_t skip);

static bool tof_sm_run_flag = false;

// Declare an app_timer id variable and define our timer interval and define a timer interval
APP_TIMER_DEF(m_tof_timer_id);
APP_TIMER_DEF(m_led_timer_id);
APP_TIMER_DEF(m_battery_timer_id);

#define TOF_SAMPLE_TIMER_INTERVAL               APP_TIMER_TICKS(10)     // intervals in ms
#define LED_ADV_TOGGLE_TIMER_INTERVAL           APP_TIMER_TICKS(125)    // intervals in ms
#define LED_CON_TOGGLE_TIMER_INTERVAL           APP_TIMER_TICKS(500)    // intervals in ms
#define BATTERY_LEVEL_MEAS_INTERVAL             APP_TIMER_TICKS(120000) // Battery level measurement interval (ticks).
// This value corresponds to 120 seconds (2 minutes).

// NOTE: AG - Make sure to update the number of vendor specific UUIDs in our project
//            by setting the NRF_SDH_BLE_VS_UUID_COUNT value in sdk_config.h

//            In this project we have 3x with:
//              1) BLE_DFU_SERVICE_UUID
//              2) BLE_UUID_TOF_SERVICE
//              3) BLE_UUID_PWR_SERVICE

// Advertised service UUIDs
// NOTE: AG - For our device lets just advertise the most important service
static ble_uuid_t m_adv_uuids[] = {
//    { BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE },
//    { BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE }
//    { BLE_UUID_TX_POWER_SERVICE,           BLE_UUID_TYPE_BLE}
//    { BLE_DFU_SERVICE_UUID,                (BLE_UUID_TYPE_VENDOR_BEGIN + 0) } // NOTE: AG - Offset to increment by is determined by order the services are initialized
		{ BLE_UUID_TOF_SERVICE, (BLE_UUID_TYPE_VENDOR_BEGIN + 1) } // NOTE: AG - Offset to increment by is determined by order the services are initialized
//    { BLE_UUID_PWR_SERVICE,                (BLE_UUID_TYPE_VENDOR_BEGIN + 2) } // NOTE: AG - Offset to increment by is determined by order the services are initialized
};

// Scan response service UUIDs..do not use more than one 128bit UUID
//static ble_uuid_t m_sr_uuids[] = {
//    { BLE_UUID_TOF_SERVICE,                (BLE_UUID_TYPE_VENDOR_BEGIN + 1) }
//};

static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event);
//static void on_bas_evt(ble_bas_t *p_bas, ble_bas_evt_t *p_evt);

// This is a timer event handler for the tof sensors
static void tof_timer_timeout_handler(void *p_context) {
    UNUSED_PARAMETER(p_context);
    tof_sm_run_flag = true;
}

// This is a timer event handler for the battery voltage sampling
static void tof_pwr_batt_meas_timeout_handler(void *p_context) {
    UNUSED_PARAMETER(p_context);
    tof_pwr_batt_sample_voltage();
}

// ble tof characteristic for selecting which sensor to use
static void tof_select_write_handler(uint16_t conn_handle, uint8_t tof_select) {
    tof_sensor_select(tof_select);
}

// ble tof characteristic for setting the sensor configurations
static void tof_config_write_handler(uint16_t conn_handle, uint8_t cmd, uint8_t id, int32_t value) {
    tof_sensor_config_cmd(cmd, id, value);
}

// ble tof characteristic to start/stop the sensor ranging
static void tof_ranging_enable_write_handler(uint16_t conn_handle, uint8_t value) {
    tof_sensor_ranging_enable_set(value);
}

// ble tof characteristic to reset the sensor
static void tof_reset_write_handler(uint16_t conn_handle, uint8_t reset_command) {
    switch(reset_command){
        case TOF_RESET_DEVICE:
            tof_pwr_reset();
            break;
        case TOF_RESET_SENSOR:
        case TOF_RESET_SENSOR_FACTORY:
            tof_sensor_reset(reset_command);
            break;
        default:
            break;
    }
}

// This callback updates the ble tof data characteristic
void tof_data_callback(device_t *device, snsr_data_type_t type) {
    switch (type) {
        case TOF_DATA_SELECTED:
            // Update sensor selected and type
            tof_select_characteristic_update(
                    &m_tof,
                    device->sensor->id,
                    device->sensor->type);
            break;
        case TOF_DATA_DISTANCE:
            // Update distance characteristic
            tof_range_characteristic_update(&m_tof, &device->distance_mm);
            break;
            // Update status characteristic
        case TOF_DATA_STATUS:
            tof_status_characteristic_update(&m_tof, &device->sensor->status);
            break;
        case TOF_DATA_CONFIG:
            // Update configuration characteristic
            tof_config_characteristic_update(
                    &m_tof,
                    device->config_data.cmd,
                    device->config_data.id,
                    device->config_data.value,
                    device->config_data.status);
            break;
        case TOF_DATA_SAMPLING_ENABLED:
            // Update tof enable_sampling characteristic
            tof_ranging_enable_characteristic_update(&m_tof, &device->is_ranging_enabled);
            break;
        case TOF_DATA_RESET: {
            // Update tof reset command characteristic
            tof_reset_characteristic_update(&m_tof, &device->reset_cmd);
            break;
        }
        default:
            break;
    }
}

/**@brief Function for handling the ADC interrupt.
 *
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
void tof_pwr_data_callback(pwr_mngt_data_t *m_pwr_mngt_data, tof_pwr_data_type_t type) {
	switch (type) {
	case TOF_PWR_DATA_INPUT_SOURCE:
		// Update power input source characteristic
		pwr_source_characteristic_update(&m_pwr, &m_pwr_mngt_data->pwr_source);
		break;
	case TOF_PWR_DATA_BATT_STATUS:
		// Update battery status characteristic
		pwr_batt_status_characteristic_update(&m_pwr, &m_pwr_mngt_data->batt_status);
		break;
    case TOF_PWR_DATA_BATT_LEVEL:
        // Update battery level characteristic
        pwr_batt_level_characteristic_update(&m_pwr, &m_pwr_mngt_data->batt_lvl_milli_volts);
        break;
	default:
		break;
	}
}

/**@brief Function for handling the ToF Service events.
 *
 * @details This function will be called for all ToF Service events which are passed to the
 |          application.
 *
 * @param[in] p_bas  ToF Service structure.
 * @param[in] p_evt  Event received from the ToF Service.
 */
static void on_tof_evt(ble_tof_t *p_tof, ble_evt_t const *p_ble_evt) {
  switch(p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
        // When connected; start the sensor timer
        app_timer_start(m_tof_timer_id, TOF_SAMPLE_TIMER_INTERVAL, NULL);
      break;
      break;
    case BLE_GAP_EVT_DISCONNECTED:
        // When disconnected; stop the sensor timer
        app_timer_stop(m_tof_timer_id);
      break;
    default:
      // No implementation needed
      break;
  }
}

/**@brief Function for handling the Pwr Service events.
 *
 * @details This function will be called for all Pwr Service events which are passed to the
 |          application.
 *
 * @param[in] p_bas  Pwr Service structure.
 * @param[in] p_evt  Event received from the Pwr Service.
 */
static void on_pwr_evt(ble_pwr_t *p_pwr, ble_evt_t const *p_ble_evt) {
  switch(p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
        // When connected; start battery monitor timer
        app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
        tof_pwr_batt_sample_voltage_delayed(250);
      break;
    case BLE_GAP_EVT_DISCONNECTED:
        // When disconnected; stop the battery timer
        app_timer_stop(m_battery_timer_id);
      break;
    default:
      // No implementation needed
      break;
  }
}

// This is a timer event handler
static void led_timer_timeout_handler(void *p_context) {
	UNUSED_PARAMETER(p_context);
	nrf_gpio_pin_toggle(PIN_PWR_ON_LED);
}

///**@brief Function for handling the Battery Service events.
// *
// * @details This function will be called for all Battery Service events which are passed to the
// |          application.
// *
// * @param[in] p_bas  Battery Service structure.
// * @param[in] p_evt  Event received from the Battery Service.
// */
//static void on_bas_evt(ble_bas_t *p_bas, ble_bas_evt_t *p_evt) {
//	ret_code_t err_code;
//
//	switch (p_evt->evt_type) {
//	case BLE_BAS_EVT_NOTIFICATION_ENABLED:
//		// Start battery timer
//		err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
//		APP_ERROR_CHECK(err_code);
//		break; // BLE_BAS_EVT_NOTIFICATION_ENABLED
//
//	case BLE_BAS_EVT_NOTIFICATION_DISABLED:
//		err_code = app_timer_stop(m_battery_timer_id);
//		APP_ERROR_CHECK(err_code);
//		break; // BLE_BAS_EVT_NOTIFICATION_DISABLED
//
//	default:
//		// No implementation needed.
//		break;
//	}
//}

/**@brief Function for initializing the Device Information Service.
 */
static void dis_init(void) {
	ret_code_t err_code;
	ble_dis_init_t dis_init_obj;

	// Initialize Device Information Service.
	ble_dis_sys_id_t sys_id;
	ble_dis_pnp_id_t pnp_id;
	ble_dis_reg_cert_data_list_t cert_list;
	uint8_t cert_list_data[] = BLE_DIS_CERT_LIST;

	memset(&dis_init, 0, sizeof(dis_init));

	ble_srv_ascii_to_utf8(&dis_init_obj.manufact_name_str, BLE_DIS_MANUFACTURER_NAME);
	ble_srv_ascii_to_utf8(&dis_init_obj.model_num_str, BLE_DIS_MODEL_NUMBER);
	ble_srv_ascii_to_utf8(&dis_init_obj.serial_num_str, BLE_DIS_SERIAL_NUMBER);
	ble_srv_ascii_to_utf8(&dis_init_obj.hw_rev_str, BLE_DIS_HW_REVISION);
	ble_srv_ascii_to_utf8(&dis_init_obj.fw_rev_str, BLE_DIS_FW_REVISION);
	ble_srv_ascii_to_utf8(&dis_init_obj.sw_rev_str, BLE_DIS_SW_REVISION);

	sys_id.manufacturer_id = BLE_DIS_MANUFACTURER_ID;
	sys_id.organizationally_unique_id = BLE_DIS_OU_ID;

	cert_list.p_list = cert_list_data;
	cert_list.list_len = sizeof(cert_list_data);

	pnp_id.vendor_id_source = BLE_DIS_VENDOR_ID_SRC_BLUETOOTH_SIG;
	pnp_id.vendor_id = BLE_DIS_VENDOR_ID;
	pnp_id.product_id = BLE_DIS_PRODUCT_ID;
	pnp_id.product_version = BLE_DIS_PRODUCT_VERSION;

	dis_init_obj.p_sys_id = &sys_id;
	dis_init_obj.p_reg_cert_data_list = &cert_list;
	dis_init_obj.p_pnp_id = &pnp_id;

	dis_init_obj.dis_char_rd_sec = SEC_OPEN;

	err_code = ble_dis_init(&dis_init_obj);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Device Firmware Update Service.
 */
static void dfu_init(void) {
	ret_code_t err_code;
	ble_dfu_buttonless_init_t dfus_init = { 0 };

	dfus_init.evt_handler = ble_dfu_evt_handler;

	err_code = ble_dfu_buttonless_init(&dfus_init);
	APP_ERROR_CHECK(err_code);
}

///**@brief Function for initializing the Battery Service.
// */
//static void bas_init(void) {
//	ret_code_t err_code;
//	ble_bas_init_t bas_init_obj;
//
//	memset(&bas_init_obj, 0, sizeof(bas_init_obj));
//
//	bas_init_obj.evt_handler = on_bas_evt;
//	bas_init_obj.support_notification = true;
//	bas_init_obj.p_report_ref = NULL;
//	bas_init_obj.initial_batt_level = 100;
//
//	bas_init_obj.bl_rd_sec = SEC_OPEN;
//	bas_init_obj.bl_cccd_wr_sec = SEC_OPEN;
//	bas_init_obj.bl_report_rd_sec = SEC_OPEN;
//
//	err_code = ble_bas_init(&m_bas, &bas_init_obj);
//	APP_ERROR_CHECK(err_code);
//}

///**@brief Function for initializing the TX Power Service.
// */
//static void tps_init(void)
//{
//    ret_code_t     err_code;
//    ble_tps_init_t tps_init_obj;
//
//    memset(&tps_init_obj, 0, sizeof(tps_init_obj));
//    tps_init_obj.initial_tx_power_level = TX_POWER_LEVEL;
//
//    tps_init_obj.tpl_rd_sec = SEC_OPEN;
//
//    err_code = ble_tps_init(&m_tps, &tps_init_obj);
//    APP_ERROR_CHECK(err_code);
//}

/**@brief Function for initializing the ToF Service.
 */
static void tof_init(void) {
	ret_code_t err_code;
	ble_tof_init_t tof_init_obj;

	tof_init_obj.evt_handler = on_tof_evt;
	tof_init_obj.tof_select_write_handler = tof_select_write_handler;
	tof_init_obj.tof_config_write_handler = tof_config_write_handler;
	tof_init_obj.tof_ranging_enable_write_handler = tof_ranging_enable_write_handler;
    tof_init_obj.tof_reset_write_handler = tof_reset_write_handler;

	err_code = ble_tof_init(&m_tof, &tof_init_obj);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the PWR Service.
 */
static void pwr_init(void) {
	ret_code_t err_code;
	ble_pwr_init_t pwr_init_obj;

	pwr_init_obj.evt_handler = on_pwr_evt;

	err_code = ble_pwr_init(&m_pwr, &pwr_init_obj);
	APP_ERROR_CHECK(err_code);
}

/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          until the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event) {
	switch (event) {
	case NRF_PWR_MGMT_EVT_PREPARE_SYSOFF:
		break;

	case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
		break;

	case NRF_PWR_MGMT_EVT_PREPARE_DFU:
		NRF_LOG_INFO("Power management wants to reset to DFU mode.")
		;
		// YOUR_JOB: Get ready to reset into DFU mode
		//
		// If you aren't finished with any ongoing tasks, return "false" to
		// signal to the system that reset is impossible at this stage.
		//
		// Here is an example using a variable to delay resetting the device.
		//
//             if (!m_ready_for_reset)
//             {
//                  return false;
//             }
//             else
//            {
		uint32_t err_code;

		// Disable rng sensors
		tof_sm_uninit();
		// Disable pwr monitor
		tof_pwr_uninit();

		// Stop all timers
		err_code = app_timer_stop_all();
		APP_ERROR_CHECK(err_code);

		// Device ready to enter
		// Shut down the soft device
		err_code = sd_softdevice_disable();
		APP_ERROR_CHECK(err_code);
//            }
		break;

	case NRF_PWR_MGMT_EVT_PREPARE_RESET:
		break;

	default:
		return true;
	}

	NRF_LOG_INFO("Power management allowed to reset to DFU mode.");

	return true;
}

//lint -esym(528, m_app_shutdown_handler)
/**@brief Register application shutdown handler with priority 0.
 */
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void *p_context) {
	if (state == NRF_SDH_EVT_STATE_DISABLED) {
		// Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
		nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

		//Go to system off.
		nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
	}
}

/* nrf_sdh state observer. */
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) = {
        .handler = buttonless_dfu_sdh_state_observer,
};

static void disconnect(uint16_t conn_handle, void *p_context) {
	UNUSED_PARAMETER(p_context);

	ret_code_t err_code = sd_ble_gap_disconnect(conn_handle,
			BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	if (err_code != NRF_SUCCESS) {
		NRF_LOG_WARNING("BLE Failed to disconnect connection. Connection handle: %d Error: %d",
				conn_handle, err_code);
	} else {
		NRF_LOG_DEBUG("BLE Disconnected connection handle %d", conn_handle);
	}
}

static void advertising_config_get(ble_adv_modes_config_t *p_config) {
    memset(p_config, 0, sizeof(ble_adv_modes_config_t));

//    p_config->ble_adv_whitelist_enabled          = true; // Only allows connections from whitelisted devices
//    p_config->ble_adv_directed_high_duty_enabled = true;
//    p_config->ble_adv_directed_enabled           = false;
//    p_config->ble_adv_directed_interval          = 0;
//    p_config->ble_adv_directed_timeout           = 0;
    p_config->ble_adv_fast_enabled               = true;
    p_config->ble_adv_fast_interval              = APP_ADV_FAST_INTERVAL;
    p_config->ble_adv_fast_timeout               = APP_ADV_FAST_DURATION;
//    p_config->ble_adv_slow_enabled               = false;
//    p_config->ble_adv_slow_interval              = APP_ADV_SLOW_INTERVAL;
//    p_config->ble_adv_slow_timeout               = APP_ADV_SLOW_DURATION;
}

/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event) {
	switch (event) {
	case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE: {
		NRF_LOG_INFO("DFU Device is preparing to enter bootloader mode.");

		// Prevent device from advertising on disconnect.
		ble_adv_modes_config_t config;
		advertising_config_get(&config);
		config.ble_adv_on_disconnect_disabled = true;
		ble_advertising_modes_config_set(&m_advertising, &config);

		// Disconnect all other bonded devices that currently are connected.
		// This is required to receive a service changed indication
		// on bootup after a successful (or aborted) Device Firmware Update.
		uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
		NRF_LOG_INFO("DFU Disconnected %d links.", conn_count);
		break;
	}

	case BLE_DFU_EVT_BOOTLOADER_ENTER:
		// YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
		//           by delaying reset by reporting false in app_shutdown_handler
		NRF_LOG_INFO("DFU Device will enter bootloader mode.")
		;
		break;

	case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
		NRF_LOG_ERROR("DFU Request to enter bootloader mode failed asynchronously.")
		;
		// YOUR_JOB: Take corrective measures to resolve the issue
		//           like calling APP_ERROR_CHECK to reset the device.
		break;

	case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
		NRF_LOG_ERROR("DFU Request to send a response to client failed.")
		;
		// YOUR_JOB: Take corrective measures to resolve the issue
		//           like calling APP_ERROR_CHECK to reset the device.
		APP_ERROR_CHECK(false);
		break;

	default:
		NRF_LOG_ERROR("DFU Unknown event from ble_dfu_buttonless.")
		;
		break;
	}
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name) {
	app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const *p_evt) {
	// Typical peer handlers
	pm_handler_on_pm_evt(p_evt);
	// spm_handler_disconnect_on_sec_failure(p_evt); // TODO AG - This can apparently cause bonding failures

	// Run garbage collection
	pm_handler_flash_clean(p_evt);

	switch (p_evt->evt_id) {
	case PM_EVT_CONN_SEC_SUCCEEDED:
		m_peer_id = p_evt->peer_id;
		NRF_LOG_INFO("BLE Connection to peer succeeded");
		break;

	case PM_EVT_BONDED_PEER_CONNECTED:
		NRF_LOG_INFO("BLE Bond to peer succeeded");
		break;

	case PM_EVT_PEERS_DELETE_SUCCEEDED:
		advertising_start(false);
		NRF_LOG_INFO("BLE Peers deletion succeeded");
		break;

	case PM_EVT_CONN_SEC_CONFIG_REQ: {
        // TODO AG - This block should allow a "re-pairing" if the central (i.e. mobile phone) requests a pairing
        // but a bond already exists. This can handle the case in which the user has denied the bluetooth
        // device even after a successful previous paring. For security reasons this is not recommended
        // and it is probably better to delete previous bonds instead.
		pm_conn_sec_config_t config = {
		        .allow_repairing = true // <-- This flag
		};
		pm_conn_sec_config_reply(p_evt->conn_handle, &config);
		NRF_LOG_INFO("BLE Requested to bond");
	}
		break;

	case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
		if (p_evt->params.peer_data_update_succeeded.flash_changed
				&& (p_evt->params.peer_data_update_succeeded.data_id == PM_PEER_DATA_ID_BONDING)) {
			NRF_LOG_INFO("BLE New Bond, add the peer to the whitelist if possible");
			whitelist_set(PM_PEER_ID_LIST_SKIP_NO_ID_ADDR); // TODO AG - Check which peer type should be added to whitelist
		}
		break;

	default:
		break;
	}
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void) {
	// Initialize timer module.
	uint32_t err_code = app_timer_init();
	APP_ERROR_CHECK(err_code);

	// Create timers.
	err_code = app_timer_create(&m_led_timer_id, APP_TIMER_MODE_REPEATED, led_timer_timeout_handler);
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_create(&m_tof_timer_id, APP_TIMER_MODE_REPEATED, tof_timer_timeout_handler);
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_create(&m_battery_timer_id, APP_TIMER_MODE_REPEATED, tof_pwr_batt_meas_timeout_handler);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void) {
	uint32_t err_code;
	ble_gap_conn_params_t gap_conn_params;
	ble_gap_conn_sec_mode_t sec_mode;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

	err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t*) DEVICE_NAME, strlen(DEVICE_NAME));
	APP_ERROR_CHECK(err_code);

	err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
	APP_ERROR_CHECK(err_code);

	memset(&gap_conn_params, 0, sizeof(gap_conn_params));

	gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
	gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
	gap_conn_params.slave_latency = SLAVE_LATENCY;
	gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

	err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
	APP_ERROR_CHECK(err_code);
}

/**@brief   Function for initializing the GATT module.
 * @details The GATT module handles ATT_MTU and Data Length update procedures automatically.
 */
static void gatt_init(void) {
	ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error) {
	APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void) {
	uint32_t err_code;
	nrf_ble_qwr_init_t qwr_init = { 0 };

	// Initialize Queued Write Module.
	qwr_init.error_handler = nrf_qwr_error_handler;

	err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
	APP_ERROR_CHECK(err_code);

	/*
	 * Vendor specific services
	 */
	// Device firmware update service init
	dfu_init();
	// ToF service init
	tof_init();
	// Pwr service init
	pwr_init();

	/*
	 * BLE registered services
	 */
	// Device information service init
	dis_init();
//	  // Battery management service init
//	  bas_init();
//    // Transmission power service init
//    tps_init();
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt) {
	uint32_t err_code;

	switch(p_evt->evt_type) {
        case BLE_CONN_PARAMS_EVT_SUCCEEDED:
            break;
        case BLE_CONN_PARAMS_EVT_FAILED:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
            APP_ERROR_CHECK(err_code);
            break;
	}
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error) {
	APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void) {
	uint32_t err_code;
	ble_conn_params_init_t cp_init;

	memset(&cp_init, 0, sizeof(cp_init));

	cp_init.p_conn_params = NULL;
	cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
	cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
	cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
	cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
	cp_init.disconnect_on_fail = false;
	cp_init.evt_handler = on_conn_params_evt;
	cp_init.error_handler = conn_params_error_handler;

	err_code = ble_conn_params_init(&cp_init);
	APP_ERROR_CHECK(err_code);
}

///**@brief Function for starting timers.
// */
//static void application_timers_start(void)
//{
//    // NOTE AG - Only uncomment when needing to test ToF sensors without connecting to bluetooth
//    //app_timer_start(m_tof_timer_id, TOF_SAMPLE_TIMER_INTERVAL, NULL);
//}

// NOTE: AG - Application now shuts down after the advertising timout; thereore, no need to sleep
///**@brief Function for putting the chip into sleep mode.
// *
// * @note This function will not return.
// */
//static void sleep_mode_enter(void)
//{
//    // Make sure the main LED is in the off state
//    nrf_gpio_pin_set(PIN_PWR_ON_LED);
//
//    NRF_LOG_INFO("SYS Going to sleep...");
//
//    //Disable SoftDevice. It is required to be able to write to GPREGRET2 register (SoftDevice API blocks it).
//    //GPREGRET2 register holds the information about skipping CRC check on next boot.
//    uint32_t err_code = nrf_sdh_disable_request();
//    APP_ERROR_CHECK(err_code);
//}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
	ret_code_t err_code = NRF_SUCCESS;

	switch (ble_adv_evt) {
	case BLE_ADV_EVT_IDLE:
		tof_pwr_shutdown();
		break;

	case BLE_ADV_EVT_DIRECTED_HIGH_DUTY:
		NRF_LOG_INFO("BLE High Duty Directed advertising");
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_ADV_EVT_DIRECTED:
		NRF_LOG_INFO("BLE Directed advertising");
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_ADV_EVT_FAST:
		NRF_LOG_INFO("BLE Fast advertising");
		APP_ERROR_CHECK(err_code);

        // When advertising; start LED_Advertise timer
        app_timer_stop(m_led_timer_id);
        app_timer_start(m_led_timer_id, LED_ADV_TOGGLE_TIMER_INTERVAL, NULL);
		break;

	case BLE_ADV_EVT_SLOW:
		NRF_LOG_INFO("BLE Slow advertising");
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_ADV_EVT_FAST_WHITELIST:
		NRF_LOG_INFO("BLE Fast advertising with whitelist");
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_ADV_EVT_SLOW_WHITELIST:
		NRF_LOG_INFO("BLE Slow advertising with whitelist");
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_ADV_EVT_WHITELIST_REQUEST: {
		ble_gap_addr_t whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
		ble_gap_irk_t whitelist_irks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
		uint32_t addr_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
		uint32_t irk_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

		err_code = pm_whitelist_get(whitelist_addrs, &addr_cnt, whitelist_irks, &irk_cnt);
		APP_ERROR_CHECK(err_code);
		NRF_LOG_DEBUG("BLE pm_whitelist_get returns %d addr in whitelist and %d irk whitelist", addr_cnt, irk_cnt);

		// Set the correct identities list (no excluding peers with no Central Address Resolution).
		identities_set(PM_PEER_ID_LIST_SKIP_NO_IRK);

		// Apply the whitelist.
		err_code = ble_advertising_whitelist_reply(&m_advertising, whitelist_addrs, addr_cnt, whitelist_irks, irk_cnt);
		APP_ERROR_CHECK(err_code);
	}
		break; //BLE_ADV_EVT_WHITELIST_REQUEST

	case BLE_ADV_EVT_PEER_ADDR_REQUEST: {
		pm_peer_data_bonding_t peer_bonding_data;

		// Only Give peer address if we have a handle to the bonded peer.
		if (m_peer_id != PM_PEER_ID_INVALID) {
			err_code = pm_peer_data_bonding_load(m_peer_id, &peer_bonding_data);
			if (err_code != NRF_ERROR_NOT_FOUND) {
				APP_ERROR_CHECK(err_code);

				// Manipulate identities to exclude peers with no Central Address Resolution.
				identities_set(PM_PEER_ID_LIST_SKIP_ALL);

				ble_gap_addr_t *p_peer_addr = &(peer_bonding_data.peer_ble_id.id_addr_info);
				err_code = ble_advertising_peer_addr_reply(&m_advertising, p_peer_addr);
				APP_ERROR_CHECK(err_code);
			}
		}
	}
		break; //BLE_ADV_EVT_PEER_ADDR_REQUEST

	default:
		break;
	}
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
	uint32_t err_code = NRF_SUCCESS;

	switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("BLE Connected");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);

            // Set connection tx power level
            err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, APP_CONN_TX_PWR);
            APP_ERROR_CHECK(err_code);

            // Start connection state LED timer
            app_timer_stop(m_led_timer_id);
            app_timer_start(m_led_timer_id, LED_CON_TOGGLE_TIMER_INTERVAL, NULL);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("BLE Disconnected")
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
            NRF_LOG_DEBUG("BLE PHY update request");
            ble_gap_phys_t const phys = { .rx_phys = BLE_GAP_PHY_AUTO, .tx_phys = BLE_GAP_PHY_AUTO, };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
            break;
        }

        case BLE_GATTS_EVT_SYS_ATTR_MISSING: // Note: This should be handled by the peer manager
            // No system attributes have been stored.
            NRF_LOG_DEBUG("BLE GATT Attribute Missing");
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("BLE GATT Client Timeout");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("BLE GATT Server Timeout");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case SD_BLE_GAP_WHITELIST_SET:
            NRF_LOG_DEBUG("BLE GAP whitelist set");
            break;

        default:
            // No implementation needed.
            break;
	}
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void) {
	ret_code_t err_code;

	err_code = nrf_sdh_enable_request();
	APP_ERROR_CHECK(err_code);

	// Configure the BLE stack using the default settings.
	// Fetch the start address of the application RAM.
	uint32_t ram_start = 0;
	err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
	APP_ERROR_CHECK(err_code);

	// Enable BLE stack.
	err_code = nrf_sdh_ble_enable(&ram_start);
	APP_ERROR_CHECK(err_code);

	// Register a handler for BLE events.
	NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void) {
	ble_gap_sec_params_t sec_param;
	ret_code_t err_code;

	err_code = pm_init();
	APP_ERROR_CHECK(err_code);

	memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

	// Security parameters to be used for all security procedures.
	sec_param.bond = SEC_PARAM_BOND;
	sec_param.mitm = SEC_PARAM_MITM;
	sec_param.lesc = SEC_PARAM_LESC;
	sec_param.keypress = SEC_PARAM_KEYPRESS;
	sec_param.io_caps = SEC_PARAM_IO_CAPABILITIES;
	sec_param.oob = SEC_PARAM_OOB;
	sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
	sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
	sec_param.kdist_own.enc = 1;
	sec_param.kdist_own.id = 1;
	sec_param.kdist_peer.enc = 1;
	sec_param.kdist_peer.id = 1;

	err_code = pm_sec_params_set(&sec_param);
	APP_ERROR_CHECK(err_code);

	err_code = pm_register(pm_evt_handler);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for setting filtered whitelist.
 *
 * @param[in] skip  Filter passed to @ref pm_peer_id_list.
 */
static void whitelist_set(pm_peer_id_list_skip_t skip) {
	pm_peer_id_t peer_ids[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
	uint32_t peer_id_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

	ret_code_t err_code = pm_peer_id_list(peer_ids, &peer_id_count, PM_PEER_ID_INVALID, skip);
	APP_ERROR_CHECK(err_code);

	NRF_LOG_INFO("BLE m_whitelist_peer_cnt %d, MAX_PEERS_WLIST %d", peer_id_count, BLE_GAP_WHITELIST_ADDR_MAX_COUNT);

	err_code = pm_whitelist_set(peer_ids, peer_id_count);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for setting filtered device identities.
 *
 * @param[in] skip  Filter passed to @ref pm_peer_id_list.
 */
static void identities_set(pm_peer_id_list_skip_t skip) {
	pm_peer_id_t peer_ids[BLE_GAP_DEVICE_IDENTITIES_MAX_COUNT];
	uint32_t peer_id_count = BLE_GAP_DEVICE_IDENTITIES_MAX_COUNT;

	ret_code_t err_code = pm_peer_id_list(peer_ids, &peer_id_count, PM_PEER_ID_INVALID, skip);
	APP_ERROR_CHECK(err_code);

	err_code = pm_device_identities_list_set(peer_ids, peer_id_count);
	APP_ERROR_CHECK(err_code);
}

/** @brief Clear bonding information from persistent storage.
 */
static void delete_bonds(void) {
	ret_code_t err_code;

	NRF_LOG_INFO("BLE Erase bonds!");

	err_code = pm_peers_delete();
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for check if delete bonds flag was set in GPREGRET2.
 */
static bool delete_bonds_flag_get(void){
    uint8_t gpregret2 = nrf_power_gpregret2_get();
    bool ret_val = false;
    // Clear Delete Bonds Flag mark in GPREGRET2 register.
    if ((gpregret2 & APP_NOTIFY_DELETE_BONDS_MASK) == APP_NOTIFY_DELETE_BONDS){
        nrf_power_gpregret2_set(gpregret2 & ~APP_NOTIFY_DELETE_BONDS);
        ret_val = true;
    }

    NRF_LOG_INFO("BLE GPREGRET2 value: 0x%X, delete bonds: %s", gpregret2, (ret_val ? "Yes": "No"));

    return ret_val;
}

/**@brief Function for the Power manager.
 */
static void log_init(void) {
    uint32_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void) {
	uint32_t err_code;
	ble_advertising_init_t init;

	memset(&init, 0, sizeof(init));

	init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
	init.advdata.include_appearance = false; // NOTE: AG - Turning this flag off fixed the adv name from being truncated
	init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

	// Advertising UUDIs
	init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
	init.advdata.uuids_complete.p_uuids = m_adv_uuids;

	// Scan response UUIDs if more space needed for advertising however
	// only one 128bit UUID can be used here
//    init.srdata.uuids_complete .uuid_cnt = sizeof(m_sr_uuids) / sizeof(m_sr_uuids[0]);
//    init.srdata.uuids_complete .p_uuids  = m_sr_uuids;

	advertising_config_get(&init.config);

	init.evt_handler = on_adv_evt;

	err_code = ble_advertising_init(&m_advertising, &init);
	APP_ERROR_CHECK(err_code);

	ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

	// Set the advertising power level
	err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, APP_ADV_TX_PWR);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds) {
	if (erase_bonds == true) {
		delete_bonds();
		// Note: Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
	}
	else {
		whitelist_set(PM_PEER_ID_LIST_SKIP_NO_ID_ADDR);

		uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
		APP_ERROR_CHECK(err_code);

		NRF_LOG_DEBUG("BLE advertising is started");
	}
}

static void power_management_init(void) {
	uint32_t err_code = nrf_pwr_mgmt_init();
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void) {
	// ToF loop
	if (tof_sm_run_flag) {
	    tof_sm_run_flag = false;
		tof_sm_run();
	}

	// Process the tof_service hvx indications queue if not empty
	tof_hvx_gatts_queue_process();

	// Check debug commands
	rtt_debug_cmd_check();

	// Process logs before idle
	if (!NRF_LOG_PROCESS()){
		nrf_pwr_mgmt_run();
	}
}

/**@brief Function for application main entry.
 */
int main(void) {
	ret_code_t err_code;

	log_init();

	// Initialize the async SVCI interface to bootloader before any interrupts are enabled.
	err_code = ble_dfu_buttonless_async_svci_init();
	APP_ERROR_CHECK(err_code);

	timers_init(); // NOTE: This needs run here for PB_OUT app_button to work properly

	// Set up indicator LED
	nrf_gpio_cfg_output(PIN_PWR_ON_LED);
	// Turn on LED when system online
	nrf_gpio_pin_set(PIN_PWR_ON_LED);

	power_management_init();
	ble_stack_init();

	tof_pwr_init();
	tof_sm_init();

	peer_manager_init();
	gap_params_init();
	gatt_init();

	services_init(); // NOTE: AG - Run before advertising_init() if using vendor specific UUIDs in advertising data
	advertising_init();

	conn_params_init();

	NRF_LOG_INFO("ToF application started");

	// Start execution.

    // Check if erase bonds flag was set
	bool delete_bonds = delete_bonds_flag_get();
	advertising_start(delete_bonds);

	rtt_debug_cmd_init();

	// Enter main loop.
	for (;;) {
		idle_state_handle();
	}
}

/**
 * @}
 */
