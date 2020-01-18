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
    BMP180_CMD_TEMP = 0x2E,  //< Read temperature command to be written to ctrl_meas register
    BMP180_CMD_PRESS = 0x34, //< Read pressure command to be written to ctrl_meas register
} bmp180_ctrl_cmd_t;

/**
 * @brief BMP180 communication modes.
 */
typedef enum
{
    BMP180_MODE_ULTRA_LOW_POWER = 0, //< Samples: 1, Conversion time: 4.5 ms, Current: 3 uA/sample, RMS Noise: 0.06 hPa
    BMP180_MODE_STANDARD = 1, //< Samples: 2, Conversion time: 7.5 ms, Current: 5 uA/sample, RMS Noise: 0.05 hPa
    BMP180_MODE_HIGH_RESOLUTION = 2, //< Samples: 4, Conversion time: 13.5 ms, Current: 7 uA/sample, RMS Noise: 0.04 hPa
    BMP180_MODE_ULTRA_HIGH_RESOLUTION = 3 //< Samples: 8, Conversion time: 25.5 ms, Current: 12 uA/sample, RMS Noise: 0.03 hPa
} bmp180_mode_t;

/**
 * @brief BMP180 calibration values.
 */
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
 * @brief BMP180 driver instance data.
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
 * @brief If set to 1 then calibrations value are not read out from the BMP180 device
 * and default values are used instead.
 */
#ifndef BMP180_USE_DEFAULT_CALIBRATIONS
#define BMP180_USE_DEFAULT_CALIBRATIONS 0
#endif

#define BMP180_DEFAULT_CALIBRATIONS         \
{                                           \
    .ac1 = 408,                             \
    .ac2 = -72,                             \
    .ac3 = -14383,                          \
    .ac4 = 32741,                           \
    .ac5 = 32757,                           \
    .ac6 = 23153,                           \
    .b1 = 6190,                             \
    .b2 = 4,                                \
    .mb = -32768,                           \
    .mc = -8711,                            \
    .md = 2868,                             \
}

/**
 * BMP180 driver instance initializer. Should be used for driver instance declaration.
 */
#define BMP180_INSTANCE(twi_par)                 \
{                                                \
    .twi = twi_par,                              \
    .address = BMP180_ADDRESS,                   \
    .busy = false,                               \
    .twi_init = false,                           \
	.calibrations = BMP180_DEFAULT_CALIBRATIONS, \
	.started = false,                            \
	.mode = BMP180_MODE_STANDARD                 \
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

/**
 * @brief Initialize BMP180 sensor driver and TWI interface.
 *
 * @note  Must be called before using the driver.
 * @note  Set config parameter to NULL if TWIM instance is already initialized in external code.
 * @note  TWI interface is initialized to 250 kbps speed by default.
 *
 * @param[in] bmp180  A pointer to BMP180 driver instance data.
 * @param[in] twi_config  A pointer to TWI configuration. If NULL then TWI instance will be used without initialization.
 *
 * @retval NRFX_SUCCESS               Driver was successfully initialized.
 * @retval NRFX_ERROR_INVALID_PARAM   Invalid configuration passed.
 */
nrfx_err_t driver_bmp180_init(driver_bmp180_t *bmp180, bmp180_twi_config_t *twi_config);

/**
 * @brief Uninitialize the driver and release hardware resources. TWIM driver is
 * uninitialized only when was initialized by this driver.
 *
 * @param[in] bmp180  A pointer to BMP180 driver instance data.
 */
void driver_bmp180_uninit(driver_bmp180_t *bmp180);

/**
 * @brief Start communication with BMP180 device. Calibration registers are downloaded from the device.
 *
 * @note Start must be called before first measurement after driver initialization.
 *
 * @param[in] bmp180  A pointer to BMP180 driver instance data.
 *
 * @retval NRFX_SUCCESS                Successfully started the communication.
 * @retval NRF_DRIVERS_ERROR_BUSY      The communication interface is busy.
 * @retval NRF_DRIVERS_ERROR_INV_DATA  Invalid data received.
 */
nrfx_err_t driver_bmp180_start(driver_bmp180_t *bmp180);

/**
 * @brief Perform software reset. This function will perform the same sequence as
 * power on reset.
 *
 * @note It takes some milliseconds before device starts responding after reset.
 *
 * @param[in] bmp180  A pointer to BMP180 driver instance data.
 *
 * @retval NRFX_SUCCESS                Reset was successfully performed.
 * @retval NRF_DRIVERS_ERROR_BUSY      The communication interface is busy.
 */
nrfx_err_t driver_bmp180_soft_reset(driver_bmp180_t *bmp180);

/**
 * @brief Set sampling accuracy mode which will be used for next measurement.
 *
 * @note Mode setting will affect measurement accuracy, duration and power consumption.
 * Read BMP180 datasheet for more information.
 *
 * @param[in] bmp180  A pointer to BMP180 driver instance data.
 *
 * @retval NRFX_SUCCESS      Mode was successfully set.
 */
nrfx_err_t driver_bmp180_set_mode(driver_bmp180_t *bmp180, bmp180_mode_t mode);

/**
 * @brief Get sensor temperature in Celsius degree with 0.1 accuracy.
 *
 * @note It takes 4.5 ms before temperature is read out.
 *
 * @param[in] bmp180  A pointer to BMP180 driver instance data.
 * @param[out] temp   A pointer to temperature value to be measured.
 *
 * @retval NRFX_SUCCESS              Temperature was successfully read.
 * @retval NRF_DRIVERS_ERROR_BUSY    The communication interface is busy.
 * @retval NRFX_ERROR_INVALID_STATE  Communication not started. Use driver_bmp180_start function.
 */
nrfx_err_t driver_bmp180_get_temp(driver_bmp180_t *bmp180, float *temp);

/**
 * @brief Get sensor temperature in Celsius degree with 0.1 accuracy and sensor pressure in Pa.
 *
 * @note Temperature is used for pressure value compensation.
 * @note It takes some time before temperature is read out depending on accuracy mode.
 *
 * @param[in] bmp180   A pointer to BMP180 driver instance data.
 * @param[out] temp    A pointer to temperature value to be measured.
 * @param[out] temp    A pointer to pressure value to be measured.
 *
 * @retval NRFX_SUCCESS              Temperature and pressure were successfully read.
 * @retval NRF_DRIVERS_ERROR_BUSY    The communication interface is busy.
 * @retval NRFX_ERROR_INVALID_STATE  Communication not started. Use driver_bmp180_start function.
 */
nrfx_err_t driver_bmp180_get_press(driver_bmp180_t *bmp180, float *temp, float *press);

/**
 * @brief Calculate estimated altitude in meters based on average sea level pressure.
 *
 * @param[in] press_sealvl   Average sea level pressure in Pa.
 * @param[in] press          Ambient pressure in Pa measured by BMP180 sensor.
 *
 * @return  Altitude value in meters.
 */
float driver_bmp180_calc_abs_alt(float press_sealvl, float press);

/**
 * @brief Calculate estimated sea level pressure in Pa.
 *
 * @param[in] altitude   Altitude in meters.
 * @param[in] press      Ambient pressure in Pa measured by BMP180 sensor.
 *
 * @return  Sea level pressure in Pa.
 */
float driver_bmp180_calc_sealvl_press(float altitude, float press);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_BMP180_H_ */
