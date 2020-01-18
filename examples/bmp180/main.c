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

#include <stdlib.h>

#include "hal/nrf_gpio.h"
#include "nrf_delay.h"
#include "driver_bmp180.h"

#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// TWI instance
static nrfx_twim_t twi = NRFX_TWIM_INSTANCE(0);
// BMP180 driver instance using twim 0
static driver_bmp180_t bmp180 = BMP180_INSTANCE(&twi);

int main(int aArgc, char *aArgv[])
{
    float temp, press, altitude;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\nHTU21D sensor example started.");
    NRF_LOG_FLUSH();

    // Intitalize BMP180 sensor driver and TWI interface
    bmp180_twi_config_t twi_config = {
        .scl_pin = NRF_GPIO_PIN_MAP(0, 17),
        .sda_pin = NRF_GPIO_PIN_MAP(0, 19)
    };
    driver_bmp180_init(&bmp180, &twi_config);
    // Start communication and download calibration values
    driver_bmp180_start(&bmp180);
    // Set measurement accuracy mode
    driver_bmp180_set_mode(&bmp180, BMP180_MODE_ULTRA_HIGH_RESOLUTION);
    // Get pressure [Pa] and temperature [C] measurement
    driver_bmp180_get_press(&bmp180, &temp, &press);
    // Calculate estimated altitude [m]
    altitude = driver_bmp180_calc_abs_alt(BMP180_AVERAGE_SEALVL_PRESS, press);

    NRF_LOG_INFO("\r\nTemperature: %f, pressure: %f, altitude: %f.",
        temp, press, altitude);

    // Reset
    driver_bmp180_soft_reset(&bmp180);
    return 0;
}
