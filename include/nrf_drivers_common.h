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

#ifndef NRF_DRIVERS_COMMON_H_
#define NRF_DRIVERS_COMMON_H_

#include <nrfx.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nrf_drivers common
 * @{
 * @ingroup nrf_drivers_common
 * @brief   Shared definitions for NRF driver libraries.
 */

#define NRF_DRIVERS_ERROR_BASE (0x9000)

/* @defgroup NRF Drivers Error Codes
 * @{ */
#define NRF_DRIVERS_ERROR_CHECKSUM (NRF_DRIVERS_ERROR_BASE + 1)
#define NRF_DRIVERS_ERROR_TIMEOUT  (NRF_DRIVERS_ERROR_BASE + 2)
#define NRF_DRIVERS_ERROR_BUSY     (NRF_DRIVERS_ERROR_BASE + 3)
/** @} */

/* @defgroup Error Handling Macros
 * @{ */

/**
 * This macro check if the error code is success. If not then return is called with error code.
 * Should be used only in functions with error code return type.
 *
 * @param[in]  error     An error code to be evaluated against NRFX_SUCCESS.
 *
 */
#define RETURN_ON_ERROR(error)            \
    do                                    \
    {                                     \
        if ((error) != NRFX_SUCCESS)      \
        {                                 \
            return error;                 \
        }                                 \
    } while (false)

/**
 * This macro check if the error code is success. If not then goto exit label is called.
 *
 * @param[in]  error     An error code to be evaluated against NRFX_SUCCESS.
 *
 */
#define EXIT_ON_ERROR(error)              \
    do                                    \
    {                                     \
        if ((error) != NRFX_SUCCESS)      \
        {                                 \
            goto exit;                    \
        }                                 \
    } while (false)

/**
 * This macro check condition and return error if it is false. If condition result is true
 * then function execution continues. Should be used only in functions with error code return type.
 *
 * @param[in]  condition  Condition to be fulfilled.
 * @param[in]  error      An error code to be returned.
 *
 */
#define VERIFY_OR_RETURN(condition, error)   \
    do                                       \
    {                                        \
        if (!(condition))                    \
        {                                    \
            return error;                    \
        }                                    \
    } while (false)

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* NRF_DRIVERS_COMMON_H_ */
