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

#include <math.h>

#include "nrf_delay.h"
#include "driver_htu21d.h"

/* Constants for calculations */
#define A 8.1332
#define B 1762.39
#define C 235.66

#define HTU21D_READ_VALUE_TIMEOUT_MS 100
#define HTU21D_POLLING_INTERVAL_MS 5

static nrfx_err_t driver_htu21d_twi_init(nrfx_twim_t *twi, const htu21d_twi_config_t *config)
{
    const nrfx_twim_config_t twi_config = {
       .scl                = config->scl_pin,
       .sda                = config->sda_pin,
       .frequency          = NRF_TWIM_FREQ_250K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .hold_bus_uninit    = false
    };

    return nrfx_twim_init(twi, &twi_config, NULL, NULL);
}

static float driver_htu21d_calculate_temp(uint16_t raw_temp)
{
    return (175.72 * raw_temp) / 65536.0 - 46.85;
}

static float driver_htu21d_calculate_hum(uint16_t raw_temp)
{
    return (125.0 * raw_temp) / 65536.0 - 6.0;
}

static bool driver_htu21d_check_crc(uint8_t *received)
{
    uint32_t remainder = ((uint32_t)received[0] << 8 | (uint32_t)received[1]) << 8;
    remainder |= received[2];
    uint32_t divisor = 0x988000;
    for (int i = 0 ; i < 16 ; i++)
    {
      if (remainder & (uint32_t)1 << (23 - i))
      {
        remainder ^= divisor;
      }
      divisor >>= 1;
    }
    return remainder == 0;
}

static uint16_t driver_htu21d_get_raw_value(uint8_t *received)
{
    uint16_t value = (uint16_t)received[0] << 8 | (uint16_t)received[1];
    value &= 0xfffc; // Mask status bytes
    return value;
}

static nrfx_err_t driver_htu21d_send_command(driver_htu21d_t *htu21d, uint8_t command)
{
    nrfx_twim_xfer_desc_t desc = NRFX_TWIM_XFER_DESC_TX(htu21d->address, &command, 1);
    return nrfx_twim_xfer(htu21d->twi, &desc, 0);
}

static nrfx_err_t driver_htu21d_read_value_hold(driver_htu21d_t *htu21d, uint16_t *value)
{
    uint8_t received[3] = {0};
    nrfx_err_t error = NRFX_SUCCESS;
    // Request read, master is hold by slave until the measurement is complete
    nrfx_twim_xfer_desc_t desc = NRFX_TWIM_XFER_DESC_RX(htu21d->address, received, 3);
    RETURN_ON_ERROR(error = nrfx_twim_xfer(htu21d->twi, &desc, 0));

    if (!driver_htu21d_check_crc(received)) return NRF_DRIVERS_ERROR_CHECKSUM;
    *value = driver_htu21d_get_raw_value(received);
    return error;
}

static nrfx_err_t driver_htu21d_read_value_no_hold(driver_htu21d_t *htu21d, uint16_t *value)
{
    uint8_t received[3] = {0};
    nrfx_twim_xfer_desc_t desc = NRFX_TWIM_XFER_DESC_RX(htu21d->address, received, 3);
    uint32_t wait_time = 0;
    nrfx_err_t error = NRFX_SUCCESS;
    // Poll for measurement until timeout is reached or sensor stops sending NACK
    while (wait_time <= HTU21D_READ_VALUE_TIMEOUT_MS
        && (error = nrfx_twim_xfer(htu21d->twi, &desc, 0)) == NRFX_ERROR_DRV_TWI_ERR_ANACK)
    {
        nrf_delay_ms(HTU21D_POLLING_INTERVAL_MS);
        wait_time += HTU21D_POLLING_INTERVAL_MS;
    }
    // Check error states
    if (wait_time > HTU21D_READ_VALUE_TIMEOUT_MS) return NRF_DRIVERS_ERROR_TIMEOUT;
    RETURN_ON_ERROR(error);

    if (!driver_htu21d_check_crc(received)) return NRF_DRIVERS_ERROR_CHECKSUM;
    *value = driver_htu21d_get_raw_value(received);
    return error;
}

nrfx_err_t driver_htu21d_init(driver_htu21d_t *htu21d, const htu21d_twi_config_t *config)
{
    nrfx_err_t error = NRFX_SUCCESS;
    VERIFY_OR_RETURN(htu21d != NULL || htu21d->twi != NULL, NRFX_ERROR_INVALID_PARAM);
    if (config != NULL)
    {
        RETURN_ON_ERROR(error = driver_htu21d_twi_init(htu21d->twi, config));
        htu21d->twi_init = true;
    }
    // According to specification it is necessary to wait 15 ms
    nrfx_twim_enable(htu21d->twi);
    return error;
}

void driver_htu21d_uninit(driver_htu21d_t *htu21d)
{
    if (htu21d->twi_init)
    {
        nrfx_twim_disable(htu21d->twi);
        nrfx_twim_uninit(htu21d->twi);
    }
}

nrfx_err_t driver_htu21d_get_temp_hold(driver_htu21d_t *htu21d, float *value)
{
    uint16_t raw_value = 0;
    nrfx_err_t error = NRFX_SUCCESS;

    VERIFY_OR_RETURN(!htu21d->busy, NRF_DRIVERS_ERROR_TIMEOUT);
    htu21d->busy = true;
    EXIT_ON_ERROR(error = driver_htu21d_send_command(htu21d, HTU21D_TRIGGER_TEMP_MEASURE_HOLD));
    EXIT_ON_ERROR(error = driver_htu21d_read_value_hold(htu21d, &raw_value));
    *value = driver_htu21d_calculate_temp(raw_value);

exit:
    htu21d->busy = false;
    return error;
}

nrfx_err_t driver_htu21d_get_temp_no_hold(driver_htu21d_t *htu21d, float *value)
{
    uint16_t raw_value = 0;
    nrfx_err_t error = NRFX_SUCCESS;

    VERIFY_OR_RETURN(!htu21d->busy, NRF_DRIVERS_ERROR_TIMEOUT);
    htu21d->busy = true;
    EXIT_ON_ERROR(error = driver_htu21d_send_command(htu21d, HTU21D_TRIGGER_TEMP_MEASURE_NOHOLD));
    EXIT_ON_ERROR(error = driver_htu21d_read_value_no_hold(htu21d, &raw_value));
    *value = driver_htu21d_calculate_temp(raw_value);

exit:
    htu21d->busy = false;
    return error;
}

nrfx_err_t driver_htu21d_get_hum_hold(driver_htu21d_t *htu21d, float *value)
{
    uint16_t raw_value = 0;
    nrfx_err_t error = NRFX_SUCCESS;

    VERIFY_OR_RETURN(!htu21d->busy, NRF_DRIVERS_ERROR_TIMEOUT);
    htu21d->busy = true;
    EXIT_ON_ERROR(error = driver_htu21d_send_command(htu21d, HTU21D_TRIGGER_HUM_MEASURE_HOLD));
    EXIT_ON_ERROR(error = driver_htu21d_read_value_hold(htu21d, &raw_value));
    *value = driver_htu21d_calculate_hum(raw_value);

exit:
    htu21d->busy = false;
    return error;
}

nrfx_err_t driver_htu21d_get_hum_no_hold(driver_htu21d_t *htu21d, float *value)
{
    uint16_t raw_value = 0;
    nrfx_err_t error = NRFX_SUCCESS;

    VERIFY_OR_RETURN(!htu21d->busy, NRF_DRIVERS_ERROR_TIMEOUT);
    htu21d->busy = true;
    EXIT_ON_ERROR(error = driver_htu21d_send_command(htu21d, HTU21D_TRIGGER_HUM_MEASURE_NOHOLD));
    EXIT_ON_ERROR(error = driver_htu21d_read_value_no_hold(htu21d, &raw_value));
    *value = driver_htu21d_calculate_hum(raw_value);

exit:
    htu21d->busy = false;
    return error;
}

static nrfx_err_t driver_htu21d_read_register_data(driver_htu21d_t *htu21d, uint8_t *register_data)
{
    nrfx_err_t error = NRFX_SUCCESS;
    RETURN_ON_ERROR(error = driver_htu21d_send_command(htu21d, HTU21D_READ_USER_REG));
    nrfx_twim_xfer_desc_t desc = NRFX_TWIM_XFER_DESC_RX(htu21d->address, register_data, 1);
    error = nrfx_twim_xfer(htu21d->twi, &desc, 0);
    return error;
}

nrfx_err_t driver_htu21d_read_register(driver_htu21d_t *htu21d, htu21d_config_t *config)
{
    uint8_t register_data = 0;
    nrfx_err_t error = NRFX_SUCCESS;

    VERIFY_OR_RETURN(!htu21d->busy, NRF_DRIVERS_ERROR_TIMEOUT);
    htu21d->busy = true;
    error = driver_htu21d_read_register_data(htu21d, &register_data);
    htu21d->busy = false;
    RETURN_ON_ERROR(error);

    config->resolution = USER_REGISTER_RESOLUTION_MASK & register_data;
    config->end_of_battery = (USER_REGISTER_END_OF_BATTERY_MASK & register_data) != 0;
    config->heater_enabled = (USER_REGISTER_HEATER_ENABLED_MASK & register_data) != 0;
    config->disable_otp_reload = (USER_REGISTER_DISABLE_OTP_RELOAD_MASK & register_data) != 0;
    return error;
}

nrfx_err_t driver_htu21d_write_register(driver_htu21d_t *htu21d, const htu21d_config_t *config)
{
    uint8_t register_data = 0;
    nrfx_err_t error = NRFX_SUCCESS;

    VERIFY_OR_RETURN(!htu21d->busy, NRF_DRIVERS_ERROR_TIMEOUT);
    htu21d->busy = true;
    // Read current register data, reserved bits must not be changed
    EXIT_ON_ERROR(error = driver_htu21d_read_register_data(htu21d, &register_data));

    // Set register flags
    register_data |= USER_REGISTER_RESOLUTION_MASK & config->resolution;
    register_data |= USER_REGISTER_END_OF_BATTERY_MASK & (config->end_of_battery ? 0xff : 0x0);
    register_data |= USER_REGISTER_HEATER_ENABLED_MASK & (config->heater_enabled ? 0xff : 0x0);
    register_data |= USER_REGISTER_DISABLE_OTP_RELOAD_MASK & (config->disable_otp_reload ? 0xff : 0x0);

    // write data to register
    uint8_t data[2] = {0};
    data[0] = HTU21D_WRITE_USER_REG;
    data[1] = register_data;
    nrfx_twim_xfer_desc_t desc = NRFX_TWIM_XFER_DESC_TX(htu21d->address, data, 2);
    error = nrfx_twim_xfer(htu21d->twi, &desc, 0);

exit:
    htu21d->busy = false;
    return error;
}

nrfx_err_t driver_htu21d_soft_reset(driver_htu21d_t *htu21d)
{
    nrfx_err_t error = NRFX_SUCCESS;
    VERIFY_OR_RETURN(!htu21d->busy, NRF_DRIVERS_ERROR_TIMEOUT);
    htu21d->busy = true;
    error = driver_htu21d_send_command(htu21d, HTU21D_SOFT_RESET);
    htu21d->busy = false;
    return error;
}

float driver_htu21d_calc_dew_point(float temp, float hum)
{
    // The equation for calculating dew point is defined in HTU21D specification.
    float pp = powf(10, A - (B / (temp + C)));
    float log_p = (hum * pp) / 100;
    float temp_dew = -1.0 * ((B / (log10f(log_p) - A)) + C);
    return temp_dew;
}

