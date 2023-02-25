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

#include "timer_delay.h"

#include "boards.h"
#include "app_timer.h"

#ifdef SOFTDEVICE_PRESENT
#include "nrf_sdh.h"
#endif

// See timer_delay.h for info on this macro
#define APP_TIMER_TICKS_US(US)          \
            ((uint32_t)MAX(5, (US/30)))

APP_TIMER_DEF(m_polling_delay_timer_id);

static void polling_delay_timer_handler(void *p_context) {
    uint8_t *_delay = (uint8_t*) p_context;
    *_delay = false;
}

static uint32_t createTimer(uint32_t ticks){
    static uint8_t m_delay_active = true;
    m_delay_active = true;

    uint32_t err_code = app_timer_create(&m_polling_delay_timer_id, APP_TIMER_MODE_SINGLE_SHOT, polling_delay_timer_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(m_polling_delay_timer_id, ticks, &m_delay_active);
    APP_ERROR_CHECK(err_code);

    wfe(&m_delay_active);

    return err_code;
}

uint32_t PollingDelayUS(uint32_t time_us) {
    return createTimer(APP_TIMER_TICKS_US(time_us));
}

uint32_t PollingDelayMS(uint32_t time_ms) {
    return createTimer(APP_TIMER_TICKS(time_ms));
}

void wfe(volatile uint8_t* is_active) {
#ifdef SOFTDEVICE_PRESENT
    uint32_t err_code = NRF_SUCCESS;
#endif // SOFTDEVICE_PRESENT
    if (is_active) {
        while (*is_active){
#ifdef SOFTDEVICE_PRESENT
            if (nrf_sdh_is_enabled()) {
                err_code = sd_app_evt_wait();
                APP_ERROR_CHECK(err_code);
            }
#else
            {
              __WFE();
            }
#endif // SOFTDEVICE_PRESENT
        }
    }
    else {
#ifdef SOFTDEVICE_PRESENT
        if (nrf_sdh_is_enabled()) {
            err_code = sd_app_evt_wait();
            APP_ERROR_CHECK(err_code);
        }
#else
        {
          __WFE();
        }
#endif // SOFTDEVICE_PRESENT
    }
}
