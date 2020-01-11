/*
 *  Copyright (c) 2020, Vit Holasek
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "hal/nrf_gpio.h"
#include "nrf_delay.h"
#include "driver_htu21d.h"

#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// TWI instance
static nrfx_twim_t twi = NRFX_TWIM_INSTANCE(0);
// HTU21D driver instance using twim 0
static driver_htu21d_t htu21d = HTU21D_INSTANCE(&twi);

int main(int aArgc, char *aArgv[])
{
    float temp = 0;
    float hum = 0;
    float dew_point = 0;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\nHTU21D sensor example started.");
    NRF_LOG_FLUSH();

    // Intitalize HTU21D sensor driver and TWI interface
    htu21d_twi_config_t twi_config = {
        .scl_pin = NRF_GPIO_PIN_MAP(0, 17),
        .sda_pin = NRF_GPIO_PIN_MAP(0, 19)
    };
    driver_htu21d_init(&htu21d, &twi_config);
    // Wait 15 ms for idle state
    nrf_delay_ms(15);
    // Reset HTU21D sensor after initialization
    driver_htu21d_soft_reset(&htu21d);

    // Write HTU21D user register configuration
    htu21d_config_t config = HTU21D_DEFAULT_CONFIG;
    config.resolution = RESOLUTION_RH11_TEMP11;
    config.heater_enabled = true;
    driver_htu21d_write_register(&htu21d, &config);

    // Read measurement values
    driver_htu21d_get_temp_no_hold(&htu21d, &temp);
    driver_htu21d_get_hum_no_hold(&htu21d, &hum);
    dew_point = driver_htu21d_calc_dew_point(temp, hum);
    NRF_LOG_INFO("\r\nTemperature: %f, relative humidity: %f, dew point: %f.",
        temp, hum, dew_point);
    return 0;
}
