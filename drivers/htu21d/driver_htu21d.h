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

#ifndef DRIVER_HTU21D_H_
#define DRIVER_HTU21D_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HTU21D_ADDRESS 0x40
#define HTU21D_DEFAULT_REGISTER_DATA 0x02

/**
 * @defgroup nrf_drivers HTU21D
 * @{
 * @ingroup driver_htu21s
 * @brief   Driver for controlling HTU21D module with I2C interface for measuring
 * relative humidity and temperature.
 */

/**
 * @brief Enumeration used for identification of HTU21D driver errors.
 */
typedef enum
{
    HTU21D_ERROR_NONE, ///< No error - OK result
    HTU21D_ERROR_FAIL, ///< General error type
    HTU21D_ERROR_CRC, ///< CRC verification failed
    HTU21D_ERROR_BUSY, ///< TWI peripheral is busy
    HTU21D_ERROR_TIMEOUT, ///< Device does not respond in defined time
    HTU21D_ERROR_ARGS ///< Invalid arguments passed
} htu21d_error_t;

/**
 * @brief Enumeration defines all HTU21D commands with numeric value.
 */
typedef enum
{
    HTU21D_TRIGGER_TEMP_MEASURE_HOLD = 0xE3, ///< Trigger temperature measurement with hold master, TWI bus is hold by slave while processing measurement
    HTU21D_TRIGGER_HUM_MEASURE_HOLD = 0xE5, ///< Trigger relative humidity measurement, TWI bus is hold by slave while processing measurement
    HTU21D_TRIGGER_TEMP_MEASURE_NOHOLD = 0xF3, ///< Trigger temperature measurement no hold, slave must be polled for result
    HTU21D_TRIGGER_HUM_MEASURE_NOHOLD = 0xF5, ///< Trigger relative humidity measurement no hold, slave must be polled for result
    HTU21D_WRITE_USER_REG = 0xE6, ///< Write data to the user register
    HTU21D_READ_USER_REG = 0xE7, ///< Read data from the user register
    HTU21D_SOFT_RESET = 0xFE ///< Software reset
} htu21d_command_t;

/**
 * @brief Enumeration defines possible resolution combinations for humidity
 * and temperature resolution configuration.
 */
typedef enum
{
    RESOLUTION_RH12_TEMP14 = 0x00, ///< RH 12 bits, Temp 14 bits
    RESOLUTION_RH8_TEMP12 = 0x01, ///< RH 8 bits, Temp 12 bits
    RESOLUTION_RH10_TEMP13 = 0x80, ///< RH 10 bits, Temp 13 bits
    RESOLUTION_RH11_TEMP11 = 0x81 ///< RH 11 bits, Temp 11 bits
} htu21d_resolution_t;

/**
 * @brief HTU21D configuration which is written into the user register.
 *
 * @param resolution          Measurement resolution setting.
 * @param end_of_battery      End of Battery alert/status is activated when the battery power falls below 2.25V.
 * @param heater_enabled      Connect or disconnect the input buffer.
 * @param disable_otp_reload  Pull configuration.
 */
typedef struct
{
    htu21d_resolution_t resolution;
    bool end_of_battery;
    bool heater_enabled;
    bool disable_otp_reload;
} htu21d_config_t;

/**
 * @brief Configuration of TWI interface used for communication with HTU21D.
 *
 * @param sda_pin   TWI SDA pin number.
 * @param scl_pin   TWI SCL pin number.
 */
typedef struct
{
    uint32_t sda_pin;
    uint32_t scl_pin;
} htu21d_twi_config_t;

/*
 * User register configuration bits masks.
 */
#define USER_REGISTER_RESOLUTION_MASK 0x81
#define USER_REGISTER_END_OF_BATTERY_MASK 0x40
#define USER_REGISTER_HEATER_ENABLED_MASK 0x04
#define USER_REGISTER_DISABLE_OTP_RELOAD_MASK 0x02
#define USER_REGISTER_RESERVED_MASK 0x38

/**
 * Default HTU21D user register configuration.
 */
#define HTU21D_DEFAULT_CONFIG              \
{                                          \
    .resolution = RESOLUTION_RH12_TEMP14,  \
    .end_of_battery = false,               \
    .heater_enabled = false,               \
    .disable_otp_reload = true             \
}

/**
 * @brief Initialize HTU21D sensor driver.
 *
 * @note  Must be called before using the driver.
 *
 * @note  The function waits 15 ms after initialization since it is required according to specification.
 *
 * @param[in] config  A pointer to TWI configuration.
 *
 * @retval HTU21D_ERROR_NONE   Driver was successfully initialized.
 * @retval HTU21D_ERROR_ARGS   Invalid configuration passed.
 * @retval HTU21D_ERROR_FAIL   Initialization failed.
 */
htu21d_error_t htu21d_driver_init(const htu21d_twi_config_t *config);

/**
 * @brief Uninitialize the driver and release hardware resources.
 */
void htu21d_driver_uninit(void);

/**
 * @brief Read HTU21D user register configuration fromt he sensor.
 *
 * @param[out] config  A pointer to HTU21D configuration to which will be the configuration data set.
 *
 * @retval HTU21D_ERROR_NONE   The user register configuration was successfully read out.
 * @retval HTU21D_ERROR_BUSY   The communication interface is busy.
 * @retval HTU21D_ERROR_FAIL   The user register readout failed.
 */
htu21d_error_t htu21d_driver_read_register(htu21d_config_t *config);

/**
 * @brief Write HTU21D user register configuration to the sensor.
 *
 * @param[out] config  A pointer to HTU21D configuration to be set.
 *
 * @retval HTU21D_ERROR_NONE   The user register configuration was successfully written.
 * @retval HTU21D_ERROR_BUSY   The communication interface is busy.
 * @retval HTU21D_ERROR_FAIL   The user register write failed.
 */
htu21d_error_t htu21d_driver_write_register(const htu21d_config_t *config);

/**
 * @brief Get sensor temperature in hold mode.
 *
 * @note  TWI bus is hold by slave during measurement.
 *
 * @param[out] value  A pointer to temperature value in Celsius degree.
 *
 * @retval HTU21D_ERROR_NONE   Temperature was successfully read from the sensor.
 * @retval HTU21D_ERROR_BUSY   The communication interface is busy.
 * @retval HTU21D_ERROR_CRC    CRC checksum verification failed.
 * @retval HTU21D_ERROR_FAIL   Value readout failed.
 */
htu21d_error_t htu21d_driver_get_temp_hold(float *value);

/**
 * @brief Get sensor temperature in no-hold polling mode.
 *
 * @param[out] value  A pointer to temperature value in Celsius degree.
 *
 * @retval HTU21D_ERROR_NONE     Temperature was successfully read from the sensor.
 * @retval HTU21D_ERROR_BUSY     The communication interface is busy.
 * @retval HTU21D_ERROR_CRC      CRC checksum verification failed.
 * @retval HTU21D_ERROR_TIMEOUT  Communication timeout.
 * @retval HTU21D_ERROR_FAIL     Value readout failed.
 */
htu21d_error_t htu21d_driver_get_temp_no_hold(float *value);

/**
 * @brief Get sensor relative humidity in hold mode.
 *
 * @note  TWI bus is hold by slave during measurement.
 *
 * @param[out] value  A pointer to relative humidity value in percents.
 *
 * @retval HTU21D_ERROR_NONE     Relative humidity was successfully read from the sensor.
 * @retval HTU21D_ERROR_BUSY     The communication interface is busy.
 * @retval HTU21D_ERROR_CRC      CRC checksum verification failed.
 * @retval HTU21D_ERROR_FAIL     Value readout failed.
 */
htu21d_error_t htu21d_driver_get_hum_hold(float *value);

/**
 * @brief Get sensor relative humidity in no-hold polling mode.
 *
 * @param[out] value  A pointer to relative humidity value in percents.
 *
 * @retval HTU21D_ERROR_NONE     Relative humidity was successfully read from the sensor.
 * @retval HTU21D_ERROR_BUSY     The communication interface is busy.
 * @retval HTU21D_ERROR_CRC      CRC checksum verification failed.
 * @retval HTU21D_ERROR_TIMEOUT  Communication timeout.
 * @retval HTU21D_ERROR_FAIL     Value readout failed.
 */
htu21d_error_t htu21d_driver_get_hum_no_hold(float *value);

/**
 * @brief Perform software reset of the HTU21D sensor.
 *
 * @note  It is recommended to perform software reset after initialization of the sensor
 *
 * @note  The user register register value is reset after software reset.
 *
 * @retval HTU21D_ERROR_NONE   The user register configuration was successfully written.
 * @retval HTU21D_ERROR_BUSY   The communication interface is busy.
 * @retval HTU21D_ERROR_FAIL   The user register write failed.
 */
htu21d_error_t htu21d_driver_soft_reset(void);

/**
 * @brief Calculate dew point value from ambient temperature and relative humidity measured by the sensor.
 *
 * @param[in] temp  Ambient temperature value in Celsius degree.
 * @param[in] hum   Ambient relative humidity in percents.
 *
 * @return  Dew point value in Celsius degree.
 */
float htu21d_driver_calc_dew_point(float temp, float hum);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_HTU21D_H_ */
