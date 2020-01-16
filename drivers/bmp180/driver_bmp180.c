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
#include <stdlib.h>

#include "nrf_delay.h"
#include "driver_bmp180.h"

#define BMP180_CMD_SOFT_RESET 0xB6

typedef struct
{
    int32_t temp;
    int32_t press;
} bmp180_output_t;

static nrfx_err_t driver_bmp180_twi_init(nrfx_twim_t *twi, const bmp180_twi_config_t *config)
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

static nrfx_err_t driver_bmp180_send_reg_addr(driver_bmp180_t *bmp180, bmp180_register_t address)
{
    nrfx_twim_xfer_desc_t desc = NRFX_TWIM_XFER_DESC_TX(bmp180->address, &address, 1);
    return nrfx_twim_xfer(bmp180->twi, &desc, 0);
}

static nrfx_err_t driver_bmp180_read_ushort(driver_bmp180_t *bmp180, bmp180_register_t address, uint16_t *data)
{
    nrfx_err_t error = NRFX_SUCCESS;
    uint8_t received[2];
    // Set register address for read
    RETURN_ON_ERROR(error = driver_bmp180_send_reg_addr(bmp180, address));
    // Read 2 bytes
    nrfx_twim_xfer_desc_t desc = NRFX_TWIM_XFER_DESC_RX(bmp180->address, received, 2);
    RETURN_ON_ERROR(error = nrfx_twim_xfer(bmp180->twi, &desc, 0));
    *data = (((uint16_t)received[0]) << 8) + ((uint16_t)received[1]);
    return error;
}

static nrfx_err_t driver_bmp180_read_ubyte(driver_bmp180_t *bmp180, bmp180_register_t address, uint8_t *data)
{
    nrfx_err_t error = NRFX_SUCCESS;
    uint8_t received = 0;
    // Set register address for read
    RETURN_ON_ERROR(error = driver_bmp180_send_reg_addr(bmp180, address));
    // Read 1 byte
    nrfx_twim_xfer_desc_t desc = NRFX_TWIM_XFER_DESC_RX(bmp180->address, &received, 1);
    RETURN_ON_ERROR(error = nrfx_twim_xfer(bmp180->twi, &desc, 0));
    *data = received;
    return error;
}

static nrfx_err_t driver_bmp180_write_ubyte(driver_bmp180_t *bmp180, bmp180_register_t address, uint8_t value)
{
    uint8_t data[2];
    data[0] = address;
    data[1] = value;
    nrfx_twim_xfer_desc_t desc = NRFX_TWIM_XFER_DESC_TX(bmp180->address, data, 2);
    return nrfx_twim_xfer(bmp180->twi, &desc, 0);
}

static int32_t driver_bmp180_calc_b5(driver_bmp180_t *bmp180, int32_t utemp)
{
    bmp180_cal_t *cal = &bmp180->calibrations;
    int32_t x1 = ((utemp - cal->ac6) * cal->ac5) >> 15;
    int32_t x2 = (((int32_t)cal->mc) << 11) / (x1 + cal->md);
    return x1 + x2;
}

/**
 * Get temperature in 0.1 Celsius degree.
 */
static int32_t driver_bmp180_calc_true_temp(driver_bmp180_t *bmp180, int32_t temp_raw)
{
    int32_t b5 = driver_bmp180_calc_b5(bmp180, temp_raw);
    return (int32_t)((b5 + 8) >> 4);
}

/**
 * Get temperature in 0.1 Celsius degree and pressure in Pa.
 */
static bmp180_output_t driver_bmp180_calc_true_press(driver_bmp180_t *bmp180, int32_t temp_raw, int32_t press_raw)
{
    bmp180_output_t output = {0};
    bmp180_cal_t *cal = &bmp180->calibrations;

    // Calculate true temperature
    int32_t b5 = driver_bmp180_calc_b5(bmp180, temp_raw);
    output.temp = (b5 + 8) >> 4;

    // Calculate true pressure
    int32_t b6 = b5 - 4000;
    int32_t x1 = (((int32_t)cal->b2) * ((b6 * b6) >> 12)) >> 11;
    int32_t x2 = (((int32_t)cal->ac2) * b6) >> 11;
    int32_t x3 = x1 + x2;
    int32_t b3 = (((((int32_t)cal->ac1) * 4 + x3) << bmp180->mode) + 2) >> 2;
    x1 = (((int32_t)cal->ac3) * b6) >> 13;
    x2 = (((int32_t)cal->b1) * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    uint32_t b4 = (((uint32_t)cal->ac4) * (uint32_t)(x3 + 32768)) >> 15;
    uint32_t b7 = (((uint32_t)press_raw) - b3) * (50000 >> bmp180->mode);
    int32_t p = 0;
    if (b7 < 0x80000000)
        p = (int32_t)((b7 * 2) / b4);
    else
        p = (int32_t)((b7 / b4) * 2);
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    output.press = p + ((x1 + x2 + 3791) >> 4);
    return output;
}

static nrfx_err_t driver_bmp180_read_temp_raw(driver_bmp180_t *bmp180, int32_t *temp_raw)
{
    uint16_t received = 0;
    nrfx_err_t error = NRF_SUCCESS;
    // Write read temperature command
    RETURN_ON_ERROR(error = driver_bmp180_write_ubyte(bmp180, BMP180_REGISTER_CTRL_MEAS, BMP180_CMD_TEMP));
    // Wait at least 4.5 ms
    nrf_delay_ms(5);
    // Read output
    RETURN_ON_ERROR(error = driver_bmp180_read_ushort(bmp180, BMP180_REGISTER_OUT_MSB, &received));
    *temp_raw = received;
    return error;
}

static nrfx_err_t driver_bmp180_read_press_raw(driver_bmp180_t *bmp180, int32_t *press_raw)
{
    nrfx_err_t error = NRF_SUCCESS;
    uint16_t msb = 0;
    uint8_t xlsb;
    uint8_t cmd = 0;

    // Write read pressure command
    // Choose oss mode
    cmd = BMP180_CMD_PRESS | ((bmp180->mode << 6) & BMP180_CTRL_MEAS_OSS_MASK);
    RETURN_ON_ERROR(error = driver_bmp180_write_ubyte(bmp180, BMP180_REGISTER_CTRL_MEAS, cmd));

    // Wait for some time depending on OSS settings
    switch (bmp180->mode)
    {
    case BMP180_MODE_ULTRA_LOW_POWER:
        nrf_delay_ms(5);
        break;
    case BMP180_MODE_HIGH_RESOLUTION:
        nrf_delay_ms(14);
        break;
    case BMP180_MODE_ULTRA_HIGH_RESOLUTION:
        nrf_delay_ms(26);
        break;
    case BMP180_MODE_STANDARD:
    default:
        nrf_delay_ms(8);
    }

    // Read output
    RETURN_ON_ERROR(error = driver_bmp180_read_ushort(bmp180, BMP180_REGISTER_OUT_MSB, &msb));
    RETURN_ON_ERROR(error = driver_bmp180_read_ubyte(bmp180, BMP180_REGISTER_OUT_XLSB, &xlsb));
    *press_raw = (((int32_t)msb) << 8) + xlsb;
    *press_raw >>= (8 - bmp180->mode);
    return error;
}

nrfx_err_t driver_bmp180_init(driver_bmp180_t *bmp180, bmp180_twi_config_t *twi_config)
{
    nrfx_err_t error = NRFX_SUCCESS;
    VERIFY_OR_RETURN(bmp180 != NULL || bmp180->twi != NULL, NRFX_ERROR_INVALID_PARAM);
    if (twi_config != NULL)
    {
        RETURN_ON_ERROR(error = driver_bmp180_twi_init(bmp180->twi, twi_config));
        bmp180->twi_init = true;
    }
    nrfx_twim_enable(bmp180->twi);
    return error;
}

void driver_bmp180_uninit(driver_bmp180_t *bmp180)
{
    if (bmp180->twi_init)
    {
        nrfx_twim_disable(bmp180->twi);
        nrfx_twim_uninit(bmp180->twi);
    }
}

static nrfx_err_t driver_bmp180_verify_calibration(uint16_t cal)
{
    if (cal == 0 || cal == 0xFFFF)
    {
        return NRF_DRIVERS_ERROR_INV_DATA;
    }
    return NRFX_SUCCESS;
}

nrfx_err_t driver_bmp180_start(driver_bmp180_t *bmp180)
{
    nrfx_err_t error = NRFX_SUCCESS;
    uint8_t chip_id = 0;
    bmp180_cal_t *cal = &bmp180->calibrations;
    bmp180->busy = true;
    // Check readout of chip-id and compare with expected value (0x55)
    EXIT_ON_ERROR(error = driver_bmp180_read_ubyte(bmp180, BMP180_REGISTER_ID, &chip_id));
    VERIFY_OR_EXIT(chip_id == BMP180_DEFAULT_CHIP_ID, error = NRF_DRIVERS_ERROR_INV_DATA);

    // Readout calibration data
    EXIT_ON_ERROR(error = driver_bmp180_read_ushort(bmp180, BMP180_REGISTER_CAL_AC1_MSB, (uint16_t*)&cal->ac1));
    EXIT_ON_ERROR(error = driver_bmp180_read_ushort(bmp180, BMP180_REGISTER_CAL_AC2_MSB, (uint16_t*)&cal->ac2));
    EXIT_ON_ERROR(error = driver_bmp180_read_ushort(bmp180, BMP180_REGISTER_CAL_AC3_MSB, (uint16_t*)&cal->ac3));
    EXIT_ON_ERROR(error = driver_bmp180_read_ushort(bmp180, BMP180_REGISTER_CAL_AC4_MSB, &cal->ac4));
    EXIT_ON_ERROR(error = driver_bmp180_read_ushort(bmp180, BMP180_REGISTER_CAL_AC5_MSB, &cal->ac5));
    EXIT_ON_ERROR(error = driver_bmp180_read_ushort(bmp180, BMP180_REGISTER_CAL_AC6_MSB, &cal->ac6));
    EXIT_ON_ERROR(error = driver_bmp180_read_ushort(bmp180, BMP180_REGISTER_CAL_B1_MSB, (uint16_t*)&cal->b1));
    EXIT_ON_ERROR(error = driver_bmp180_read_ushort(bmp180, BMP180_REGISTER_CAL_B2_MSB, (uint16_t*)&cal->b2));
    EXIT_ON_ERROR(error = driver_bmp180_read_ushort(bmp180, BMP180_REGISTER_CAL_MB_MSB, (uint16_t*)&cal->mb));
    EXIT_ON_ERROR(error = driver_bmp180_read_ushort(bmp180, BMP180_REGISTER_CAL_MC_MSB, (uint16_t*)&cal->mc));
    EXIT_ON_ERROR(error = driver_bmp180_read_ushort(bmp180, BMP180_REGISTER_CAL_MD_MSB, (uint16_t*)&cal->md));
    // Validate calibration data
    EXIT_ON_ERROR(error = driver_bmp180_verify_calibration((uint16_t)cal->ac1));
    EXIT_ON_ERROR(error = driver_bmp180_verify_calibration((uint16_t)cal->ac2));
    EXIT_ON_ERROR(error = driver_bmp180_verify_calibration((uint16_t)cal->ac3));
    EXIT_ON_ERROR(error = driver_bmp180_verify_calibration(cal->ac4));
    EXIT_ON_ERROR(error = driver_bmp180_verify_calibration(cal->ac5));
    EXIT_ON_ERROR(error = driver_bmp180_verify_calibration(cal->ac6));
    EXIT_ON_ERROR(error = driver_bmp180_verify_calibration((uint16_t)cal->b1));
    EXIT_ON_ERROR(error = driver_bmp180_verify_calibration((uint16_t)cal->b2));
    EXIT_ON_ERROR(error = driver_bmp180_verify_calibration((uint16_t)cal->mb));
    EXIT_ON_ERROR(error = driver_bmp180_verify_calibration((uint16_t)cal->mc));
    EXIT_ON_ERROR(error = driver_bmp180_verify_calibration((uint16_t)cal->md));

    bmp180->started = true;

exit:
    bmp180->busy = false;
    return error;
}

nrfx_err_t driver_bmp180_soft_reset(driver_bmp180_t *bmp180)
{
    nrfx_err_t error = NRFX_SUCCESS;
    VERIFY_OR_RETURN(!bmp180->busy, NRF_DRIVERS_ERROR_BUSY);
    bmp180->busy = true;
    error = driver_bmp180_write_ubyte(bmp180, BMP180_REGISTER_SOFT_RESET, BMP180_CMD_SOFT_RESET);
    bmp180->busy = false;
    return error;
}

nrfx_err_t driver_bmp180_set_mode(driver_bmp180_t *bmp180, bmp180_mode_t mode)
{
    VERIFY_OR_RETURN(!bmp180->busy, NRF_DRIVERS_ERROR_BUSY);
    bmp180->mode = mode;
    return NRFX_SUCCESS;
}

nrfx_err_t driver_bmp180_read_temp(driver_bmp180_t *bmp180, float *temp)
{
    nrfx_err_t error = NRFX_SUCCESS;
    int32_t temp_raw = 0;
    int32_t temp_true = 0;

    VERIFY_OR_RETURN(bmp180->started, NRFX_ERROR_INVALID_STATE);
    VERIFY_OR_RETURN(!bmp180->busy, NRF_DRIVERS_ERROR_BUSY);
    bmp180->busy = true;

    EXIT_ON_ERROR(error = driver_bmp180_read_temp_raw(bmp180, &temp_raw));
    temp_true = driver_bmp180_calc_true_temp(bmp180, temp_raw);
    *temp = ((float)temp_true) / 10;

exit:
    bmp180->busy = false;
    return error;
}

nrfx_err_t driver_bmp180_read_press(driver_bmp180_t *bmp180, float *temp, float *press)
{
    nrfx_err_t error = NRFX_SUCCESS;
    int32_t temp_raw = 0;
    int32_t press_raw = 0;
    bmp180_output_t output = {0};

    VERIFY_OR_RETURN(bmp180->started, NRFX_ERROR_INVALID_STATE);
    VERIFY_OR_RETURN(!bmp180->busy, NRF_DRIVERS_ERROR_BUSY);
    bmp180->busy = true;

    EXIT_ON_ERROR(error = driver_bmp180_read_temp_raw(bmp180, &temp_raw));
    EXIT_ON_ERROR(error = driver_bmp180_read_press_raw(bmp180, &press_raw));
    output = driver_bmp180_calc_true_press(bmp180, temp_raw, press_raw);
    *temp = ((float)output.temp) / 10.0;
    *press = (float)output.press;

exit:
    bmp180->busy = false;
    return error;
}

float driver_bmp180_calc_abs_alt(float press_sealvl, float press)
{
    float p_rel = powf((press / press_sealvl), (1.0 / 5.255));
    return 44330.0 * (1.0 - p_rel);
}

float driver_bmp180_calc_sealvl_press(float altitude, float press)
{
    float fract = 1.0 - (altitude / 44330.0);
    return press / powf(fract, 5.255);
}
