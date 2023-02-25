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

#ifndef SRC_TOF_PWR_MONITOR_H_
#define SRC_TOF_PWR_MONITOR_H_

#include <stdint.h>

typedef struct{
  uint8_t pwr_source;
  uint8_t batt_status;
  uint8_t charge_state;
  uint32_t batt_lvl_milli_volts;
  uint8_t batt_lvl_percent;
  uint8_t low_voltage_sd_state;
}pwr_mngt_data_t;

/** The data types that the pwr monitor can send */
typedef enum{
  TOF_PWR_DATA_INPUT_SOURCE = 0,
  TOF_PWR_DATA_BATT_STATUS,
  TOF_PWR_DATA_BATT_LEVEL,
  NUM_TOF_PWR_DATA_TYPE
}tof_pwr_data_type_t;

ret_code_t tof_pwr_init(void);
void tof_pwr_uninit(void);
void tof_pwr_batt_sample_voltage(void);
void tof_pwr_batt_sample_voltage_delayed(uint32_t delay_time_ms);
void tof_pwr_batt_print_enable(uint8_t value);
const pwr_mngt_data_t* tof_pwr_get_mngt_data(void);
void tof_pwr_reset(void);
void tof_pwr_shutdown(void);
void tof_pwr_data_callback(pwr_mngt_data_t* m_pwr_mngt_data, tof_pwr_data_type_t type);

#endif /* SRC_TOF_PWR_MONITOR_H_ */
