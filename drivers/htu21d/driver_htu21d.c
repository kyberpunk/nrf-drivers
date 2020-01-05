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
#include <stdbool.h>

#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "hal/nrf_gpio.h"

#include "config_htu21d.h"
#include "driver_htu21d.h"

/* Constants for calculations */
#define A 8.1332
#define B 1762.39
#define C 235.66

static const nrf_drv_twi_t htu21d_driver_twi = NRF_DRV_TWI_INSTANCE(HTU21D_TWI_INSTANCE_ID);

static htu21d_error_t htu21d_driver_resolve_error(ret_code_t error)
{
    switch (error)
    {
    case NRFX_SUCCESS:
        return HTU21D_ERROR_NONE;
    case NRFX_ERROR_BUSY:
        return HTU21D_ERROR_BUSY;
    default:
        return HTU21D_ERROR_FAIL;
    }
}

static htu21d_error_t twi_init(const htu21d_twi_config_t *config)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = config->scl_pin,
       .sda                = config->sda_pin,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false,
       .hold_bus_uninit    = false
    };

    err_code = nrf_drv_twi_init(&htu21d_driver_twi, &twi_config, NULL, NULL);
    if (err_code != NRFX_SUCCESS) return htu21d_driver_resolve_error(err_code);
    nrf_drv_twi_enable(&htu21d_driver_twi);
    return HTU21D_ERROR_NONE;
}

static float htu21d_driver_calculate_temp(uint16_t raw_temp)
{
    return (175.72 * raw_temp) / 65536.0 - 46.85;
}

static float htu21d_driver_calculate_hum(uint16_t raw_temp)
{
    return (125.0 * raw_temp) / 65536.0 - 6.0;
}

static bool htu21d_driver_check_crc(uint8_t *received)
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

static uint16_t htu21d_driver_get_raw_value(uint8_t *received)
{
    uint16_t value = (uint16_t)received[0] << 8 | (uint16_t)received[1];
    value &= 0xfffc; // Mask status bytes
    return value;
}

static ret_code_t htu21d_driver_send_command(uint8_t command)
{
    nrf_drv_twi_xfer_desc_t desc = NRF_DRV_TWI_XFER_DESC_TX(HTU21D_ADDRESS, &command, 1);
    return nrf_drv_twi_xfer(&htu21d_driver_twi, &desc, 0);
}

static htu21d_error_t htu21d_driver_read_value_hold(uint16_t *value)
{
    uint8_t received[3] = {0};
    // Request read, master is hold by slave until the measurement is complete
    nrf_drv_twi_xfer_desc_t desc = NRF_DRV_TWI_XFER_DESC_RX(HTU21D_ADDRESS, received, 3);
    ret_code_t err_code = nrf_drv_twi_xfer(&htu21d_driver_twi, &desc, 0);
    if (err_code != NRFX_SUCCESS) return htu21d_driver_resolve_error(err_code);

    if (!htu21d_driver_check_crc(received)) return HTU21D_ERROR_CRC;
    *value = htu21d_driver_get_raw_value(received);
    return HTU21D_ERROR_NONE;
}

static htu21d_error_t htu21d_driver_read_value_no_hold(uint16_t *value)
{
    uint8_t received[3] = {0};
    nrf_drv_twi_xfer_desc_t desc = NRF_DRV_TWI_XFER_DESC_RX(HTU21D_ADDRESS, received, 3);
    uint32_t wait_time = 0;
    ret_code_t err_code;
    // Poll for measurement until timeout is reached or sensor stops sending NACK
    while (wait_time <= HTU21D_READ_VALUE_TIMEOUT_MS
        && (err_code = nrf_drv_twi_xfer(&htu21d_driver_twi, &desc, 0)) == NRFX_ERROR_DRV_TWI_ERR_ANACK)
    {
        nrf_delay_ms(HTU21D_POLLING_INTERVAL_MS);
        wait_time += HTU21D_POLLING_INTERVAL_MS;
    }
    // Check error states
    if (wait_time > HTU21D_READ_VALUE_TIMEOUT_MS) return HTU21D_ERROR_TIMEOUT;
    if (err_code != NRFX_SUCCESS) return htu21d_driver_resolve_error(err_code);

    if (!htu21d_driver_check_crc(received)) return HTU21D_ERROR_CRC;
    *value = htu21d_driver_get_raw_value(received);
    return HTU21D_ERROR_NONE;
}

htu21d_error_t htu21d_driver_init(const htu21d_twi_config_t *config)
{
    if (config == NULL) return HTU21D_ERROR_ARGS;
    htu21d_error_t error = twi_init(config);
    if (error == HTU21D_ERROR_NONE)
    {
        // According to specification it is necessary to wait 15 ms
        nrf_delay_ms(15);
    }
    return error;
}

void htu21d_driver_uninit(void)
{
    nrf_drv_twi_disable(&htu21d_driver_twi);
    nrf_drv_twi_uninit(&htu21d_driver_twi);
}

htu21d_error_t htu21d_driver_get_temp_hold(float *value)
{
    uint16_t raw_value = 0;
    ret_code_t err_code = htu21d_driver_send_command(HTU21D_TRIGGER_TEMP_MEASURE_HOLD);
    if (err_code != NRFX_SUCCESS) return htu21d_driver_resolve_error(err_code);

    htu21d_error_t error = htu21d_driver_read_value_hold(&raw_value);
    if (error != HTU21D_ERROR_NONE) return error;

    *value = htu21d_driver_calculate_temp(raw_value);
    return HTU21D_ERROR_NONE;
}

htu21d_error_t htu21d_driver_get_temp_no_hold(float *value)
{
    uint16_t raw_value = 0;
    ret_code_t err_code = htu21d_driver_send_command(HTU21D_TRIGGER_TEMP_MEASURE_NOHOLD);
    if (err_code != NRFX_SUCCESS) return htu21d_driver_resolve_error(err_code);

    htu21d_error_t error = htu21d_driver_read_value_no_hold(&raw_value);
    if (error != HTU21D_ERROR_NONE) return error;

    *value = htu21d_driver_calculate_temp(raw_value);
    return HTU21D_ERROR_NONE;
}

htu21d_error_t htu21d_driver_get_hum_hold(float *value)
{
    uint16_t raw_value = 0;
    ret_code_t err_code = htu21d_driver_send_command(HTU21D_TRIGGER_HUM_MEASURE_HOLD);
    if (err_code != NRFX_SUCCESS) return htu21d_driver_resolve_error(err_code);

    htu21d_error_t error = htu21d_driver_read_value_hold(&raw_value);
    if (error != HTU21D_ERROR_NONE) return error;

    *value = htu21d_driver_calculate_hum(raw_value);
    return HTU21D_ERROR_NONE;
}

htu21d_error_t htu21d_driver_get_hum_no_hold(float *value)
{
    uint16_t raw_value = 0;
    ret_code_t err_code = htu21d_driver_send_command(HTU21D_TRIGGER_HUM_MEASURE_NOHOLD);
    if (err_code != NRFX_SUCCESS) return htu21d_driver_resolve_error(err_code);

    htu21d_error_t error = htu21d_driver_read_value_no_hold(&raw_value);
    if (error != HTU21D_ERROR_NONE) return error;

    *value = htu21d_driver_calculate_hum(raw_value);
    return HTU21D_ERROR_NONE;
}

static htu21d_error_t htu21d_driver_read_register_data(uint8_t *register_data)
{
    ret_code_t err_code = htu21d_driver_send_command(HTU21D_READ_USER_REG);
    if (err_code != NRFX_SUCCESS) return htu21d_driver_resolve_error(err_code);
    nrf_drv_twi_xfer_desc_t desc = NRF_DRV_TWI_XFER_DESC_RX(HTU21D_ADDRESS, register_data, 1);
    err_code = nrf_drv_twi_xfer(&htu21d_driver_twi, &desc, 0);
    return htu21d_driver_resolve_error(err_code);
}

htu21d_error_t htu21d_driver_read_register(htu21d_config_t *config)
{
    uint8_t register_data = 0;
    htu21d_error_t error = htu21d_driver_read_register_data(&register_data);
    if (error != HTU21D_ERROR_NONE) return error;

    config->resolution = USER_REGISTER_RESOLUTION_MASK & register_data;
    config->end_of_battery = (USER_REGISTER_END_OF_BATTERY_MASK & register_data) != 0;
    config->heater_enabled = (USER_REGISTER_HEATER_ENABLED_MASK & register_data) != 0;
    config->disable_otp_reload = (USER_REGISTER_DISABLE_OTP_RELOAD_MASK & register_data) != 0;
    return HTU21D_ERROR_NONE;
}

htu21d_error_t htu21d_driver_write_register(const htu21d_config_t *config)
{
    uint8_t register_data = 0;
    // Read current register data, reserved bits must not be changed
    ret_code_t err_code = htu21d_driver_read_register_data(&register_data);
    if (err_code != HTU21D_ERROR_NONE) return htu21d_driver_resolve_error(err_code);

    // Set register flags
    register_data |= USER_REGISTER_RESOLUTION_MASK & config->resolution;
    register_data |= USER_REGISTER_END_OF_BATTERY_MASK & (config->end_of_battery ? 0xff : 0x0);
    register_data |= USER_REGISTER_HEATER_ENABLED_MASK & (config->heater_enabled ? 0xff : 0x0);
    register_data |= USER_REGISTER_DISABLE_OTP_RELOAD_MASK & (config->disable_otp_reload ? 0xff : 0x0);

    // write data to register
    uint8_t data[2] = {0};
    data[0] = HTU21D_WRITE_USER_REG;
    data[1] = register_data;
    nrf_drv_twi_xfer_desc_t desc = NRF_DRV_TWI_XFER_DESC_TX(HTU21D_ADDRESS, data, 2);
    err_code = nrf_drv_twi_xfer(&htu21d_driver_twi, &desc, 0);
    return htu21d_driver_resolve_error(err_code);
}

htu21d_error_t htu21d_driver_soft_reset(void)
{
    ret_code_t err_code = htu21d_driver_send_command(HTU21D_SOFT_RESET);
    return htu21d_driver_resolve_error(err_code);
}

float htu21d_driver_calc_dew_point(float temp, float hum)
{
    // The equation for calculating dew point is defined in HTU21D specification.
    float pp = powf(10, A - (B / (temp + C)));
    float log_p = (hum * pp) / 100;
    float temp_dew = -1.0 * ((B / (log10f(log_p) - A)) + C);
    return temp_dew;
}

