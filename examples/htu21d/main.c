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
#include "driver_htu21d.h"

#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

int main(int aArgc, char *aArgv[])
{
    float temp = 0;
    float hum = 0;
    float dew_point = 0;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\nHTU21D sensor example started.");
    NRF_LOG_FLUSH();

    // Intitalize HTU21D sensor driver
    htu21d_twi_config_t twi_config = {
        .scl_pin = NRF_GPIO_PIN_MAP(0, 17),
        .sda_pin = NRF_GPIO_PIN_MAP(0, 19)
    };
    htu21d_driver_init(&twi_config);
    // Reset HTU21D sensor after initialization
    htu21d_driver_soft_reset();

    // Write HTU21D user register configuration
    htu21d_config_t config = HTU21D_DEFAULT_CONFIG;
    config.resolution = RESOLUTION_RH11_TEMP11;
    config.heater_enabled = true;
    htu21d_driver_write_register(&config);

    // Read measurement values
    htu21d_driver_get_temp_no_hold(&temp);
    htu21d_driver_get_hum_no_hold(&hum);
    dew_point = htu21d_driver_calc_dew_point(temp, hum);
    NRF_LOG_INFO("\r\nTemperature: %f, relative humidity: %f, dew point: %f.",
        temp, hum, dew_point);
    return 0;
}
