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

#include "boards.h"

#include "tof_pwr_monitor.h"

#include "nrf_drv_gpiote.h"
#include "nrf_drv_saadc.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"

#include "app_timer.h"
#include "app_button.h"

#include "timer_delay.h"

typedef enum {
  TOF_PWR_SRC_BATT = 0,
  TOF_PWR_SRC_USB
}pwr_source_t;

typedef enum {
  TOF_PWR_CHARG_OFF = 0,
  TOF_PWR_CHARG_ON
}pwr_charg_state_t;

typedef enum {
  TOF_PWR_LOW_VOLTAGE_SHUT_DOWN_INACTIVE = 0,
  TOF_PWR_LOW_VOLTAGE_SHUT_DOWN_ACTIVE
}pwr_low_voltage_sd_state_t;

typedef enum {
  TOF_BATT_OK = 0,
  TOF_BATT_LOW,
  TOF_BATT_VERY_LOW,
  TOF_BATT_CHARGING,
  TOF_BATT_CHARGING_COMPLETE,
  TOF_BATT_MISSING,
  TOF_BATT_UNKNOWN,
  NUM_TOF_BATT_STATUS
}pwr_batt_status_t;

#define PWR_BTN_DEBOUNCE_DELAY_TIMER_INTERVAL   APP_TIMER_TICKS(10) // The built in debounce time in the STM6601CQ2BDM6F supervisor chip is ~30ms at 3.7V supply voltage

#define ADC_BATT_MVOLTS_OK_THRESHOLD           3295 // Voltage at or above this will be considered OK
#define ADC_BATT_MVOLTS_LOW_THRESHOLD          3275 // The STM6601CQ2BDM6F will shut down at 3.1V so set the threshold for Low Voltage to something higher

/**
 * The BT PCB has a battery voltage monitoring circuit
 * that is turned on/off by the PIN_BM_EN pin.
 * The 1C LiPo battery typically has a max voltage at 4.2V
 * which is voltage divided from 4.2V to 1.8V as an input
 * to the nRF5X chip. T
 */
#define SAADC_SAMPLES_IN_BUFFER                 1    //Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1.
                                                     //  Otherwise the EasyDMA will be enabled for an extended time which consumes high current.
#define SAADC_OVERSAMPLE                        NRF_SAADC_OVERSAMPLE_4X  //Oversampling setting for the SAADC. Setting oversample to 4x This will make the SAADC output
                                                                         //  a single averaged value when the SAMPLE task is triggered 4 times.
                                                                         //  Enable BURST mode to make the SAADC sample 4 times when triggering SAMPLE task once.
#define SAADC_BURST_MODE                        1    //Set to 1 to enable BURST mode, otherwise set to 0.
#define SAADC_GAIN                              NRF_SAADC_GAIN1_3

#define ADC_VIN_TO_VBATT_FACTOR                 (4200.0f / 1800.0f) // The voltage divider is set to convert 4.2V (typical LiPo batt voltage) to 1.8V so 4200/1800 = ~2.333
#define ADC_MVOLTS_TO_VBATT_MVOLTS(MVOLTS)      (MVOLTS * ADC_VIN_TO_VBATT_FACTOR)

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS           600  /**< Reference voltage 0.6V (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION            3    /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be
                                                          multiplied by 3 to get the actual value of the battery voltage.*/
#define ADC_RESOLUTION                          4096 /**< Maximum digital value for 12-bit ADC conversion 2^12 = 4096. */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((ADC_VALUE * ADC_REF_VOLTAGE_IN_MILLIVOLTS * ADC_PRE_SCALING_COMPENSATION) / ADC_RESOLUTION)

APP_TIMER_DEF(m_delay_timer_id);

// SAADC buffer
static nrf_saadc_value_t adc_buf[2];
static pwr_mngt_data_t m_pwr_mngt_data;
static uint8_t _print_batt_sample_flag = 0;

static void button_signal_handler(uint8_t pin_no, uint8_t button_action);
static void input_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

static void test_pin_pwr_on_status(void);
static void test_pin_charge_status(void);
static void test_pin_low_voltage_shutdown(void);
static void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event);
static void adc_configure(void);
//static uint8_t batt_mvolts_to_percent(const uint16_t mvolts);

static void delay_timer_handler(void *p_context) {
  uint8_t* _delay = (uint8_t*)p_context;
  *_delay = false;
  tof_pwr_batt_sample_voltage();
}

ret_code_t tof_pwr_init(void){
  ret_code_t err_code;

  // PB_OUT used to detect when the power button is pressed
  static app_button_cfg_t buttons[] =
  {
      { PIN_PB_OUT, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_NOPULL, button_signal_handler }
  };

  // Set up pin for PS_HOLD for regulator Enable
  nrf_gpio_cfg_output(PIN_PS_HOLD);
  // Battery monitor enable
  nrf_gpio_cfg_output(PIN_BM_EN);
  // Set regulator enable high
  nrf_gpio_pin_set(PIN_PS_HOLD);
  // Turn battery monitor off
  nrf_gpio_pin_clear(PIN_BM_EN);

  // Set up interrupt pins
  if(!nrfx_gpiote_is_init()){
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
  }

  err_code = app_button_init(buttons, ARRAY_SIZE(buttons), PWR_BTN_DEBOUNCE_DELAY_TIMER_INTERVAL);
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);

  // Set up High-to-Low interrupt for PIN_PWR_ON_STATUS pin
  err_code = nrf_drv_gpiote_in_init(PIN_PWR_ON_STATUS, &in_config, input_pin_handler);
  APP_ERROR_CHECK(err_code);

  // Set up High-to-Low interrupt for PIN_CHARGE_STATUS pin
  err_code = nrf_drv_gpiote_in_init(PIN_CHARGE_STATUS, &in_config, input_pin_handler);
  APP_ERROR_CHECK(err_code);

  // Set up High-to-Low interrupt for PIN_INT pin
  err_code = nrf_drv_gpiote_in_init(PIN_INT, &in_config, input_pin_handler);
  APP_ERROR_CHECK(err_code);

  // Enable the pin events
  nrf_drv_gpiote_in_event_enable(PIN_PWR_ON_STATUS, true);
  nrf_drv_gpiote_in_event_enable(PIN_CHARGE_STATUS, true);
  nrf_drv_gpiote_in_event_enable(PIN_INT,           true);

  // Enable the power button
  err_code = app_button_enable();
  APP_ERROR_CHECK(err_code);

  // Set up the saadc
  adc_configure();

  // Initialize the pin states
  test_pin_pwr_on_status();
  test_pin_charge_status();
  test_pin_low_voltage_shutdown();

  return err_code;
}

void tof_pwr_uninit(void){
  nrf_gpio_pin_clear(PIN_BM_EN);
  nrf_drv_saadc_uninit();
}

void tof_pwr_batt_sample_voltage(void){
    // Turn battery monitor on
    nrf_gpio_pin_set(PIN_BM_EN);
    // Run the saadc sample
    ret_code_t err_code = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);
}

void tof_pwr_batt_sample_voltage_delayed(uint32_t delay_time_ms){
    static uint8_t m_delay_active = false;
    // If time > zero and timer not already started, then start it
    if(!m_delay_active){
      m_delay_active = true;
      // Set up single shot delay timer to allow settle time before triggering a saadc sample
      ret_code_t err_code = app_timer_create(&m_delay_timer_id, APP_TIMER_MODE_SINGLE_SHOT, delay_timer_handler);
      APP_ERROR_CHECK(err_code);
      err_code = app_timer_start(m_delay_timer_id, APP_TIMER_TICKS(delay_time_ms), &m_delay_active);
      APP_ERROR_CHECK(err_code);
    }
}

void tof_pwr_batt_status_update(void) {
    static uint8_t prev_batt_status = TOF_BATT_UNKNOWN;
    // Update the battery status
    if (TOF_PWR_LOW_VOLTAGE_SHUT_DOWN_ACTIVE == m_pwr_mngt_data.low_voltage_sd_state) {
        m_pwr_mngt_data.batt_status = TOF_BATT_VERY_LOW;
    }
    else if (TOF_PWR_SRC_USB == m_pwr_mngt_data.pwr_source && TOF_PWR_CHARG_ON == m_pwr_mngt_data.charge_state) {
        m_pwr_mngt_data.batt_status = TOF_BATT_CHARGING;
    }
    else if (TOF_PWR_SRC_USB == m_pwr_mngt_data.pwr_source && TOF_BATT_CHARGING == prev_batt_status) {
        // NOTE: AG - This status triggers once to indicate to the user to be handled as an event
        m_pwr_mngt_data.batt_status = TOF_BATT_CHARGING_COMPLETE;
    }
    else if (m_pwr_mngt_data.batt_lvl_milli_volts <= ADC_BATT_MVOLTS_LOW_THRESHOLD) {
        m_pwr_mngt_data.batt_status = TOF_BATT_LOW;
    }
    else if (m_pwr_mngt_data.batt_lvl_milli_volts >= ADC_BATT_MVOLTS_OK_THRESHOLD) {
        m_pwr_mngt_data.batt_status = TOF_BATT_OK;
    }
    else {
        // Do nothing if battery voltage in the dead-band window ADC_BATT_MVOLTS_LOW_THRESHOLD <-> ADC_BATT_MVOLTS_OK_THRESHOLD
    }

    if(prev_batt_status != m_pwr_mngt_data.batt_status){
        prev_batt_status = m_pwr_mngt_data.batt_status;
        // Callback for the application to call tasks such as BLE service characteristic updates
        tof_pwr_data_callback(&m_pwr_mngt_data, TOF_PWR_DATA_BATT_STATUS);

        // Print batt status message
        switch(m_pwr_mngt_data.batt_status){
            case TOF_BATT_VERY_LOW:
                NRF_LOG_INFO("SYS batt voltage very low, shutdown imminent");
              break;
            case TOF_BATT_CHARGING:
                NRF_LOG_INFO("SYS batt charging");
              break;
            case TOF_BATT_CHARGING_COMPLETE:
                NRF_LOG_INFO("SYS batt charging complete");
              break;
            case TOF_BATT_LOW:
                NRF_LOG_INFO("SYS batt voltage low");
              break;
            case TOF_BATT_OK:
                NRF_LOG_INFO("SYS batt voltage ok");
              break;
            default:
                NRF_LOG_INFO("SYS batt status unknown");
                break;
        }
    }
}

const pwr_mngt_data_t* tof_pwr_get_mngt_data(void){
  return &m_pwr_mngt_data;
}

void tof_pwr_batt_print_enable(uint8_t value){
  _print_batt_sample_flag = value;
}

void tof_pwr_reset(void){
    NRF_LOG_INFO("SYS reset...");
#ifdef SOFTDEVICE_PRESENT
    sd_nvic_SystemReset();
#else
    NVIC_SystemReset();
#endif
}

void tof_pwr_shutdown(void){
  NRF_LOG_INFO("SYS shutdown...");
  NRF_LOG_FLUSH();
  // Remove system power
  nrf_gpio_pin_clear(PIN_PS_HOLD);
}

static void adc_configure(void)
{
    ret_code_t err_code;
    nrf_drv_saadc_config_t saadc_config;
    nrf_saadc_channel_config_t channel_config;

    //Configure SAADC
    saadc_config.low_power_mode = true;                                   //Enable low power mode.
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;                 //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V)
                                                                          //  to 2^12=4096 (when input voltage is 1.8V for channel gain setting of 1/3).
    saadc_config.oversample = SAADC_OVERSAMPLE;                           //Set over-sample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOWEST;            //Set SAADC interrupt to priority.
    //Initialize SAADC
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_event_handler);    //Initialize the SAADC with configuration and callback function. The application must then implement the saadc_callback function,
                                                                          //  which will be called when SAADC interrupt is triggered
    APP_ERROR_CHECK(err_code);
    channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;              //Set internal reference of fixed 0.6 volts
    channel_config.gain = SAADC_GAIN;                                     //Set input gain to 1/3. The maximum SAADC input voltage is then 0.6V/(1/3)=1.8V. The single ended input range is then 0V-1.8V
    channel_config.acq_time = NRF_SAADC_ACQTIME_10US;                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz.
                                                                          //  Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS.
    channel_config.mode = NRF_SAADC_MODE_SINGLE_ENDED;                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    if(SAADC_BURST_MODE)
    {
        channel_config.burst = NRF_SAADC_BURST_ENABLED;                   //Configure burst mode for channel 0. Burst is useful together with over-sampling. When triggering the SAMPLE task in burst mode,
                                                                          //  the SAADC will sample "Oversample" number of times as fast as it can and then output a single averaged value to the RAM buffer.
                                                                          //  If burst mode is not enabled, the SAMPLE task needs to be triggered "Oversample" number of times to output a single averaged value to the RAM buffer.
    }
    channel_config.pin_p = NRF_SAADC_INPUT_AIN1;                          //Select the input pin for the channel. AIN1 pin maps to physical pin P0.03.
    channel_config.pin_n = NRF_SAADC_INPUT_DISABLED;                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;              //Disable pull-up resistor on the input pin
    channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);            // 0-7 (8 channels available or find max from NRF_SAADC_CHANNEL_COUNT)
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[0], SAADC_SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[1], SAADC_SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}

static void button_signal_handler(uint8_t pin_no, uint8_t button_action){
  static uint8_t button_action_prev = APP_BUTTON_RELEASE;

  switch(pin_no){
    case PIN_PB_OUT:
        if(button_action){
            NRF_LOG_INFO("SYS ISR: pwr button pressed");
        }
        else{
            NRF_LOG_INFO("SYS ISR: pwr button released");
        }
      // Was button pressed?
      if(button_action_prev != button_action && APP_BUTTON_RELEASE == button_action){
        tof_pwr_shutdown();
      }
      break;
    default:
        // Do nothing..
      break;
  }

  button_action_prev = button_action;
}

/**
 * This is an interrupt pin handler.
 *
 * @param pin
 * @param action
 */
static void input_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  switch(pin){
    /**
     * Check which power source we are on.
     */
    case PIN_PWR_ON_STATUS:
      test_pin_pwr_on_status();
      break;
    /**
     * Check if we are charging the battery.
     */
    case PIN_CHARGE_STATUS:
      test_pin_charge_status();
      break;
    /**
     * Check if the critically low voltage status detected.
     * The device will auto shut off when it is triggered.
     */
    case PIN_INT:
      test_pin_low_voltage_shutdown();
      break;
    /**
     * Do nothing..
     */
    default:
      break;
  }
}

/**@brief Function for handling the ADC interrupt.
 *
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
static void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
    // TODO: AG - should run nrfx_saadc_calibrate_offset() periodically

    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        nrf_saadc_value_t adc_result;
        uint32_t          err_code;

        adc_result = p_event->data.done.p_buffer[0];

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        // Turn battery monitor off
        nrf_gpio_pin_clear(PIN_BM_EN);

        // Get the raw milli volts
        double mvolts = ADC_RESULT_IN_MILLI_VOLTS(adc_result);
        m_pwr_mngt_data.batt_lvl_milli_volts = (uint32_t)ADC_MVOLTS_TO_VBATT_MVOLTS(mvolts);
        // Convert milli volts to percentage
        //m_pwr_mngt_data.batt_lvl_percent = batt_mvolts_to_percent(m_pwr_mngt_data.batt_lvl_milli_volts);

        if(_print_batt_sample_flag){
          NRF_LOG_INFO("SYS batt: %d cnt, %d mV", adc_result, m_pwr_mngt_data.batt_lvl_milli_volts);
        }

        // Update the battery level after a successful voltage sample
        tof_pwr_data_callback(&m_pwr_mngt_data, TOF_PWR_DATA_BATT_LEVEL);

        // Update the battery status after a successful voltage sample
        tof_pwr_batt_status_update();
    }
}

static void test_pin_pwr_on_status(void){
  if(nrf_gpio_pin_read(PIN_PWR_ON_STATUS)){
    m_pwr_mngt_data.pwr_source = TOF_PWR_SRC_BATT;
    NRF_LOG_INFO("SYS ISR: pwr input BATT");
  }
  else{
    m_pwr_mngt_data.pwr_source = TOF_PWR_SRC_USB;
    NRF_LOG_INFO("SYS ISR: pwr input USB");
  }
  tof_pwr_data_callback(&m_pwr_mngt_data, TOF_PWR_DATA_INPUT_SOURCE);
  tof_pwr_batt_sample_voltage_delayed(1500);
}

static void test_pin_charge_status(void){
  if(nrf_gpio_pin_read(PIN_CHARGE_STATUS)){
    m_pwr_mngt_data.charge_state = TOF_PWR_CHARG_ON;
    NRF_LOG_INFO("SYS ISR: batt charger enabled");
  }
  else{
    m_pwr_mngt_data.charge_state = TOF_PWR_CHARG_OFF;
    NRF_LOG_INFO("SYS ISR: batt charger disabled");
  }
  tof_pwr_batt_sample_voltage_delayed(1500);
}

static void test_pin_low_voltage_shutdown(void){
  // PIN_INT is held high during start up then driven low
  if(nrf_gpio_pin_read(PIN_INT)){
    m_pwr_mngt_data.low_voltage_sd_state = TOF_PWR_LOW_VOLTAGE_SHUT_DOWN_INACTIVE;
    NRF_LOG_INFO("SYS ISR: lsvd inactive");
  }
  // If PIN_INT is low and PIN_PB_OUT is high then low voltage shutdown started
  else {
	  if(nrf_gpio_pin_read(PIN_PB_OUT)){
		m_pwr_mngt_data.low_voltage_sd_state = TOF_PWR_LOW_VOLTAGE_SHUT_DOWN_ACTIVE;
		NRF_LOG_INFO("SYS ISR: lsvd started");
	  }
  }
  tof_pwr_batt_sample_voltage_delayed(100);
}
