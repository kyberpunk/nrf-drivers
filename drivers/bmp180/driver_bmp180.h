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

#ifndef DRIVER_BMP180_H_
#define DRIVER_BMP180_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "nrf_drivers_common.h"
#include "nrfx_twim.h"

/**
 * @defgroup nrf_drivers BMP180
 * @{
 * @ingroup driver_bmp180
 * @brief   Driver for controlling BMP180 digital barometric pressure and temperature sensor.
 * TWIM driver is used for communication.
 */

/**
 * @brief BMP180 device TWI address (7 bits).
 */
#define BMP180_ADDRESS 0x77

/**
 * @brief Value of ID register is fixed to 0x55 and can be used to check whether
 * communication is functioning.
 */
#define BMP180_DEFAULT_CHIP_ID 0x55

/**
 * @brief Average value of sea level pressure in Pa.
 */
#define BMP180_AVERAGE_SEALVL_PRESS 101325

/**
 * @brief Addresses of BMP180 registers.
 */
typedef enum
{
    BMP180_REGISTER_CAL_AC1_MSB = 0xAA, //< Calibration register, read only
    BMP180_REGISTER_CAL_AC1_LSB = 0xAB, //< Calibration register, read only
    BMP180_REGISTER_CAL_AC2_MSB = 0xAC, //< Calibration register, read only
    BMP180_REGISTER_CAL_AC2_LSB = 0xAD, //< Calibration register, read only
    BMP180_REGISTER_CAL_AC3_MSB = 0xAE, //< Calibration register, read only
    BMP180_REGISTER_CAL_AC3_LSB = 0xAF, //< Calibration register, read only
    BMP180_REGISTER_CAL_AC4_MSB = 0xB0, //< Calibration register, read only
    BMP180_REGISTER_CAL_AC4_LSB = 0xB1, //< Calibration register, read only
    BMP180_REGISTER_CAL_AC5_MSB = 0xB2, //< Calibration register, read only
    BMP180_REGISTER_CAL_AC5_LSB = 0xB3, //< Calibration register, read only
    BMP180_REGISTER_CAL_AC6_MSB = 0xB4, //< Calibration register, read only
    BMP180_REGISTER_CAL_AC6_LSB = 0xB5, //< Calibration register, read only
    BMP180_REGISTER_CAL_B1_MSB = 0xB6, //< Calibration register, read only
    BMP180_REGISTER_CAL_B1_LSB = 0xB7, //< Calibration register, read only
    BMP180_REGISTER_CAL_B2_MSB = 0xB8, //< Calibration register, read only
    BMP180_REGISTER_CAL_B2_LSB = 0xB9, //< Calibration register, read only
    BMP180_REGISTER_CAL_MB_MSB = 0xBA, //< Calibration register, read only
    BMP180_REGISTER_CAL_MB_LSB = 0xBB, //< Calibration register, read only
    BMP180_REGISTER_CAL_MC_MSB = 0xBC, //< Calibration register, read only
    BMP180_REGISTER_CAL_MC_LSB = 0xBD, //< Calibration register, read only
    BMP180_REGISTER_CAL_MD_MSB = 0xBE, //< Calibration register, read only
    BMP180_REGISTER_CAL_MD_LSB = 0xBF, //< Calibration register, read only
    BMP180_REGISTER_ID = 0xD0, //< Fixed register, read only
    BMP180_REGISTER_SOFT_RESET = 0xE0, //< Control register, write only
    BMP180_REGISTER_CTRL_MEAS = 0xF4, //< Control register, read/write
    BMP180_REGISTER_OUT_MSB = 0xF6, //< Data register, read only
    BMP180_REGISTER_OUT_LSB = 0xF7, //< Data register, read only
    BMP180_REGISTER_OUT_XLSB = 0xF8 //< Data register, read only
} bmp180_register_t;

#define BMP180_OUT_XLSB_ADC_OUT_XLSB_MASK 0xF8
#define BMP180_CTRL_MEAS_MEASUREMENT_CONTROL_MASK 0x1F
#define BMP180_CTRL_MEAS_SCO_MASK 0x20
#define BMP180_CTRL_MEAS_OSS_MASK 0xC0

/**
 * @brief Control register commands for different internal oversampling_setting (oss).
 */
typedef enum
{
    BMP180_CMD_TEMP = 0x2E,
    BMP180_CMD_PRESS = 0x34,
} bmp180_ctrl_cmd_t;

/**
 * @brief BMP180 communication modes.
 */
typedef enum
{
    BMP180_MODE_ULTRA_LOW_POWER = 0,
    BMP180_MODE_STANDARD = 1,
    BMP180_MODE_HIGH_RESOLUTION = 2,
    BMP180_MODE_ULTRA_HIGH_RESOLUTION = 3
} bmp180_mode_t;

typedef struct
{
    int16_t ac1;
    int16_t ac2;
    int16_t ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;
    int16_t b1;
    int16_t b2;
    int16_t mb;
    int16_t mc;
    int16_t md;
} bmp180_cal_t;

/**
 * @brief HTU21D driver instance data.
 *
 * @note Attributes are for internal use only.
 */
typedef struct
{
    nrfx_twim_t *twi;
    uint8_t address;
    bool busy;
    bool twi_init;
    bmp180_cal_t calibrations;
    bool started;
    bmp180_mode_t mode;
} driver_bmp180_t;

/**
 * BMP180 driver instance initializer. Should be used for driver instance declaration.
 */
#define BMP180_INSTANCE(twi_par)           \
{                                          \
    .twi = twi_par,                        \
    .address = BMP180_ADDRESS,             \
    .busy = false,                         \
    .twi_init = false,                     \
	.calibrations = {0},                   \
	.started = false,                      \
	.mode = BMP180_MODE_STANDARD           \
}

/**
 * @brief Configuration of TWI interface used for communication with BMP180.
 *
 * @param sda_pin   TWI SDA pin number.
 * @param scl_pin   TWI SCL pin number.
 */
typedef struct
{
    uint32_t sda_pin;
    uint32_t scl_pin;
} bmp180_twi_config_t;

nrfx_err_t driver_bmp180_init(driver_bmp180_t *bmp180, bmp180_twi_config_t *twi_config);

void driver_bmp180_uninit(driver_bmp180_t *bmp180);

nrfx_err_t driver_bmp180_start(driver_bmp180_t *bmp180);

nrfx_err_t driver_bmp180_soft_reset(driver_bmp180_t *bmp180);

nrfx_err_t driver_bmp180_set_mode(driver_bmp180_t *bmp180, bmp180_mode_t mode);

nrfx_err_t driver_bmp180_read_temp(driver_bmp180_t *bmp180, float *temp);

nrfx_err_t driver_bmp180_read_press(driver_bmp180_t *bmp180, float *temp, float *press);

float driver_bmp180_calc_abs_alt(float press_sealvl, float press);

float driver_bmp180_calc_sealvl_press(float altitude, float press);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_BMP180_H_ */
