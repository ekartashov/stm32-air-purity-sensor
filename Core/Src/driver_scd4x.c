/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      driver_scd4x.c
 * @brief     driver scd4x source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2023-09-25
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2023/09/25  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */
#include "driver_scd4x.h"
#include <stdint.h>
#include <stddef.h> // For NULL
#include <stdio.h> // For NULL

/**
 * @brief crc8 definition
 */
#define SCD4X_CRC8_POLYNOMIAL        0x31
#define SCD4X_CRC8_INIT              0xFF

// Forward declaration for IIC functions
static uint8_t a_scd4x_iic_read(scd4x_handle_t *handle, uint16_t reg, uint8_t *data, uint16_t len, uint16_t delay_ms);
static uint8_t a_scd4x_iic_write(scd4x_handle_t *handle, uint16_t reg, uint8_t *data, uint16_t len);

/**
 * @brief CRC8 generation function
 * @param data Pointer to data buffer
 * @param count Number of bytes to process
 * @return CRC8 value
 */
static uint8_t a_scd4x_generate_crc(uint8_t *data, uint8_t count) {
    uint8_t current_byte;
    uint8_t crc = SCD4X_CRC8_INIT;
    uint8_t crc_bit;

    for (current_byte = 0; current_byte < count; ++current_byte) {
        crc ^= data[current_byte];
        for (crc_bit = 0; crc_bit < 8; crc_bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ SCD4X_CRC8_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * Error handling helper functions
 */
static void scd4x_handle_error(scd4x_handle_t *handle, const char *error_msg, uint8_t error_code) {
    if (handle != 0 && handle->debug_print != 0) {
        handle->debug_print("scd4x: %s (error code: %u)\n", error_msg, error_code);
    }
}

static uint8_t scd4x_check_handle(scd4x_handle_t *handle) {
    if (handle == 0) {
        return 2; // handle is NULL
    }
    return 0;
}

static uint8_t scd4x_check_initialized(scd4x_handle_t *handle) {
    if (handle->inited != 1) {
        return 3; // handle is not initialized
    }
    return 0;
}

static uint8_t scd4x_handle_iic_operation(scd4x_handle_t *handle, uint8_t (*iic_func)(scd4x_handle_t *, uint16_t, uint8_t *, uint16_t), uint16_t command, const char *error_msg, uint8_t error_code) {
    uint8_t res;

    res = iic_func(handle, command, NULL, 0);
    if (res != 0) {
        scd4x_handle_error(handle, error_msg, error_code);
        return error_code;
    }

    return 0;
}

static uint8_t scd4x_handle_iic_operation_with_params(scd4x_handle_t *handle, uint8_t (*iic_func)(scd4x_handle_t *, uint16_t, uint8_t *, uint16_t), uint16_t command, uint8_t *params, uint16_t param_len, const char *error_msg, uint8_t error_code) {
    uint8_t res;

    res = iic_func(handle, command, params, param_len);
    if (res != 0) {
        scd4x_handle_error(handle, error_msg, error_code);
        return error_code;
    }

    return 0;
}

static uint8_t scd4x_check_crc(uint8_t *buf, uint8_t data_len, const char *error_msg, uint8_t error_code) {
    // compute CRC over the `data_len` data bytes starting at buf[0]
    uint8_t crc = a_scd4x_generate_crc(buf, data_len);

    // CRC byte is immediately after the data bytes
    if (buf[data_len] != crc) {
        scd4x_handle_error(0, error_msg, error_code);
        return error_code;
    }
    return 0;
}

static uint8_t scd4x_check_data_ready(uint16_t prev, const char *error_msg, uint8_t error_code) {
    if ((prev & 0x0FFF) == 0) {
        scd4x_handle_error(0, error_msg, error_code);
        return error_code;
    }
    return 0;
}

/**
 * @brief chip information definition
 */
#define CHIP_NAME                 "Sensirion SCD4X"        /**< chip name */
#define MANUFACTURER_NAME         "Sensirion"              /**< manufacturer name */
#define SUPPLY_VOLTAGE_MIN        2.40f                    /**< chip min supply voltage */
#define SUPPLY_VOLTAGE_MAX        5.50f                    /**< chip max supply voltage */
#define MAX_CURRENT               205.0f                   /**< chip max current */
#define TEMPERATURE_MIN           -10.0f                   /**< chip min operating temperature */
#define TEMPERATURE_MAX           60.0f                    /**< chip max operating temperature */
#define DRIVER_VERSION            1000                     /**< driver version */

/**
 * @brief chip address definition
 */
#define SCD4X_ADDRESS             (0x62)              /**< chip iic address */

/**
 * @brief chip command definition
 */
#define SCD4X_COMMAND_START_PERIODIC                                0x21B1U        /**< start periodic measurement command */
#define SCD4X_COMMAND_READ                                          0xEC05U        /**< read measurement command */
#define SCD4X_COMMAND_STOP_PERIODIC                                 0x3F86U        /**< stop periodic measurement command */
#define SCD4X_COMMAND_SET_TEMPERATURE_OFFSET                        0x241DU        /**< set temperature offset command */
#define SCD4X_COMMAND_GET_TEMPERATURE_OFFSET                        0x2318U        /**< get temperature offset command */
#define SCD4X_COMMAND_SET_SENSOR_ALTITUDE                           0x2427U        /**< set sensor altitude command */
#define SCD4X_COMMAND_GET_SENSOR_ALTITUDE                           0x2322U        /**< get sensor altitude command */
#define SCD4x_COMMAND_GET_SENSOR_VARIANT                            0x202F         /**< get senor variant */
#define SCD4X_COMMAND_SET_AMBIENT_PRESSURE                          0xE000U        /**< set ambient pressure command */
#define SCD4X_COMMAND_GET_AMBIENT_PRESSURE                          0xE000U        /**< get ambient pressure command */
#define SCD4X_COMMAND_PERFORM_FORCED_RECALIBRATION                  0x362FU        /**< perform forced recalibration command */
#define SCD4X_COMMAND_SET_AUTO_SELF_CALIBRATION                     0x2416U        /**< set automatic self calibration enabled command */
#define SCD4X_COMMAND_GET_AUTO_SELF_CALIBRATION                     0x2313U        /**< get automatic self calibration enabled command */
#define SCD4X_COMMAND_START_LOW_POWER_PERIODIC                      0x21ACU        /**< start low power periodic measurement command */
#define SCD4X_COMMAND_GET_DATA_READY_STATUS                         0xE4B8U        /**< get data ready status command */
#define SCD4X_COMMAND_PERSIST_SETTINGS                              0x3615U        /**< persist settings command */
#define SCD4X_COMMAND_GET_SERIAL_NUMBER                             0x3682U        /**< get serial number command */
#define SCD4X_COMMAND_PERFORM_SELF_TEST                             0x3639U        /**< perform self test command */
#define SCD4X_COMMAND_PERFORM_FACTORY_RESET                         0x3632U        /**< perform factory reset command */
#define SCD4X_COMMAND_REINIT                                        0x3646U        /**< reinit command */
#define SCD4X_COMMAND_MEASURE_SINGLE_SHOT                           0x219DU        /**< measure single shot command */
#define SCD4X_COMMAND_MEASURE_SINGLE_SHOT_RHT_ONLY                  0x2196U        /**< measure single shot rht only command */
#define SCD4X_COMMAND_POWER_DOWN                                    0x36E0U        /**< power down command */
#define SCD4X_COMMAND_WAKE_UP                                       0x36F6U        /**< wake up command */
#define SCD4X_COMMAND_SET_AUTO_SELF_CALIBRATION_INIT_PERIOD         0x2445U        /**< set automatic self calibration initial period command */
#define SCD4X_COMMAND_GET_AUTO_SELF_CALIBRATION_INIT_PERIOD         0x2340U        /**< get automatic self calibration initial period command */
#define SCD4X_COMMAND_SET_AUTO_SELF_CALIBRATION_STANDARD_PERIOD     0x244EU        /**< set automatic self calibration standard period command */
#define SCD4X_COMMAND_GET_AUTO_SELF_CALIBRATION_STANDARD_PERIOD     0x234BU        /**< get automatic self calibration standard period command */

/**
 * @brief crc8 definition
 */
#define SCD4X_CRC8_POLYNOMIAL        0x31
#define SCD4X_CRC8_INIT              0xFF

/**
 * @brief      read bytes with param
 * @param[in]  *handle pointer to an scd4x handle structure
 * @param[in]  reg iic register address
 * @param[in]  *data pointer to a data buffer
 * @param[in]  len data length
 * @param[in]  delay_ms delay time in ms
 * @param[out] *output pointer to an output buffer
 * @param[in]  output_len output length
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
static uint8_t a_scd4x_iic_read_with_param(scd4x_handle_t *handle, uint16_t reg, uint8_t *data, uint16_t len,
                                           uint16_t delay_ms, uint8_t *output, uint16_t output_len)
{
    uint8_t buf[16];
    uint16_t i;

    if ((len + 2) > 16)                                                            /* check length */
    {
        return 1;                                                                  /* return error */
    }
    memset(buf, 0, sizeof(uint8_t) * 16);                                          /* clear the buffer */
    buf[0] = (uint8_t)((reg >> 8) & 0xFF);                                         /* set MSB of reg */
    buf[1] = (uint8_t)(reg & 0xFF);                                                /* set LSB of reg */
    for (i = 0; i < len; i++)
    {
        buf[2 + i] = data[i];                                                      /* copy write data */
    }

    if (handle->iic_write_cmd(SCD4X_ADDRESS, (uint8_t *)buf, len + 2) != 0)        /* write iic command */
    {
        return 1;                                                                  /* write command */
    }
    handle->delay_ms(delay_ms);                                                    /* delay ms */
    if (handle->iic_read_cmd(SCD4X_ADDRESS, output, output_len) != 0)              /* read data */
    {
        return 1;                                                                  /* write command */
    }
    else
    {
        return 0;                                                                  /* success return 0 */
    }
}

/**
 * @brief      read bytes
 * @param[in]  *handle pointer to an scd4x handle structure
 * @param[in]  reg iic register address
 * @param[out] *data pointer to a data buffer
 * @param[in]  len data length
 * @param[in]  delay_ms delay time in ms
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
static uint8_t a_scd4x_iic_read(scd4x_handle_t *handle, uint16_t reg, uint8_t *data, uint16_t len, uint16_t delay_ms)
{
    uint8_t buf[2];

    memset(buf, 0, sizeof(uint8_t) * 2);                                     /* clear the buffer */
    buf[0] = (uint8_t)((reg >> 8) & 0xFF);                                   /* set reg MSB */
    buf[1] = (uint8_t)(reg & 0xFF);                                          /* set reg LSB */
    if (handle->iic_write_cmd(SCD4X_ADDRESS, (uint8_t *)buf, 2) != 0)        /* write command */
    {
        return 1;                                                            /* return error */
    }
    handle->delay_ms(delay_ms);                                              /* delay ms */
    if (handle->iic_read_cmd(SCD4X_ADDRESS, data, len) != 0)                 /* read data */
    {
        return 1;                                                            /* write command */
    }
    else
    {
        return 0;                                                            /* success return 0 */
    }
}

/**
 * @brief     write bytes
 * @param[in] *handle pointer to an scd4x handle structure
 * @param[in] reg iic register address
 * @param[in] *data pointer to a data buffer
 * @param[in] len data length
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
static uint8_t a_scd4x_iic_write(scd4x_handle_t *handle, uint16_t reg, uint8_t *data, uint16_t len)
{
    uint8_t buf[16];
    uint16_t i;

    if ((len + 2) > 16)                                                            /* check length */
    {
        return 1;                                                                  /* return error */
    }
    memset(buf, 0, sizeof(uint8_t) * 16);                                          /* clear the buffer */
    buf[0] = (uint8_t)((reg >> 8) & 0xFF);                                         /* set MSB of reg */
    buf[1] = (uint8_t)(reg & 0xFF);                                                /* set LSB of reg */
    for (i = 0; i < len; i++)
    {
        buf[2 + i] = data[i];                                                      /* copy write data */
    }

    if (handle->iic_write_cmd(SCD4X_ADDRESS, (uint8_t *)buf, len + 2) != 0)        /* write iic command */
    {
        return 1;                                                                  /* write command */
    }
    else
    {
        return 0;                                                                  /* success return 0 */
    }
}

/**
 * @brief     set type
 * @param[in] *handle pointer to an scd4x handle structure
 * @param[in] type chip type
 * @return    status code
 *            - 0 success
 *            - 2 handle is NULL
 * @note      none
 */
uint8_t scd4x_set_type(scd4x_handle_t *handle, scd4x_t type)
{
    uint8_t res;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    handle->type = type;        /* set type */

    return 0;                   /* success return 0 */
}

/**
 * @brief      get type
 * @param[in]  *handle pointer to an scd4x handle structure
 * @param[out] *type pointer to a chip type buffer
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 * @note       none
 */
uint8_t scd4x_get_type(scd4x_handle_t *handle, scd4x_t *type)
{
    uint8_t res;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    *type = (scd4x_t)(handle->type);        /* get type */

    return 0;                               /* success return 0 */
}

/**
 * @brief     start periodic measurement
 * @param[in] *handle pointer to an scd4x handle structure
 * @return    status code
 *            - 0 success
 *            - 1 start periodic measurement failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t scd4x_start_periodic_measurement(scd4x_handle_t *handle)
{
    return scd4x_handle_iic_operation(handle, a_scd4x_iic_write, SCD4X_COMMAND_START_PERIODIC,
                                       "start periodic measurement failed", 1);
}

/**
 * @brief      read data
 * @param[in]  *handle pointer to an scd4x handle structure
 * @param[out] *co2_raw pointer to a co2 raw buffer
 * @param[out] *co2_ppm pointer to a co2 ppm buffer
 * @param[out] *temperature_raw pointer to a temperature raw buffer
 * @param[out] *temperature_s pointer to a temperature buffer
 * @param[out] *humidity_raw pointer to a humidity raw buffer
 * @param[out] *humidity_s pointer to a humidity buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 *             - 4 crc error
 *             - 5 data is not ready
 * @note       none
 */
uint8_t scd4x_read(scd4x_handle_t *handle, uint16_t *co2_raw, uint16_t *co2_ppm,
                   uint16_t *temperature_raw, float *temperature_s,
                   uint16_t *humidity_raw, float *humidity_s)
{
    uint8_t res;
    uint8_t buf[9];
    uint16_t prev;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    res = a_scd4x_iic_read(handle, SCD4X_COMMAND_GET_DATA_READY_STATUS, buf, 3, 1);       /* read config */
    if (res != 0) {
        scd4x_handle_error(handle, "get data ready status failed", 1);
        return 1;
    }

    res = scd4x_check_crc(buf, 2, "crc error", 4);
    if (res != 0) {
        return res;
    }

    prev = (uint16_t)(((uint16_t)buf[0]) << 8) | buf[1];                                  /* set prev */
    res = scd4x_check_data_ready(prev, "data is not ready", 5);
    if (res != 0) {
        return res;
    }

    res = a_scd4x_iic_read(handle, SCD4X_COMMAND_READ, buf, 9, 1);                        /* read data */
    if (res != 0) {
        scd4x_handle_error(handle, "read failed", 1);
        return 1;
    }

    res = scd4x_check_crc(buf, 2, "crc error", 4);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_crc(&buf[3], 2, "crc error", 4);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_crc(&buf[6], 2, "crc error", 4);
    if (res != 0) {
        return res;
    }

    *co2_raw = (uint16_t)(((uint16_t)buf[0]) << 8) | buf[1];                              /* set co2 raw */
    *temperature_raw = (uint16_t)(((uint16_t)buf[3]) << 8) | buf[4];                      /* set temperature raw */
    *humidity_raw = (uint16_t)(((uint16_t)buf[6]) << 8) | buf[7];                         /* set humidity raw */
    *co2_ppm = *co2_raw;                                                                  /* set co2 ppm */
    *temperature_s = -45.0f + 175.0f * (float)(*temperature_raw) / 65535.0f;              /* set temperature */
    *humidity_s = 100.0f * (float)(*humidity_raw) / 65535.0f;                             /* set humidity */

    return 0;                                                                             /* success return 0 */
}

/**
 * @brief     stop periodic measurement
 * @param[in] *handle pointer to an scd4x handle structure
 * @return    status code
 *            - 0 success
 *            - 1 stop periodic measurement failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t scd4x_stop_periodic_measurement(scd4x_handle_t *handle)
{
    return scd4x_handle_iic_operation(handle, a_scd4x_iic_write, SCD4X_COMMAND_STOP_PERIODIC,
                                       "stop periodic measurement failed", 1);
}

/**
 * @brief     set temperature offset
 * @param[in] *handle pointer to an scd4x handle structure
 * @param[in] offset temperature offset
 * @return    status code
 *            - 0 success
 *            - 1 set temperature offset failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t scd4x_set_temperature_offset(scd4x_handle_t *handle, uint16_t offset)
{
    uint8_t res;
    uint8_t buf[3];

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    buf[0] = (offset >> 8) & 0xFF;                                                        /* msb */
    buf[1] = (offset >> 0) & 0xFF;                                                        /* lsb */
    buf[2] = a_scd4x_generate_crc(&buf[0], 2);                                            /* set crc */
    res = scd4x_handle_iic_operation_with_params(handle, a_scd4x_iic_write, SCD4X_COMMAND_SET_TEMPERATURE_OFFSET, buf, 3,
                                                 "set temperature offset failed", 1);
    if (res != 0) {
        return res;
    }
    handle->delay_ms(1);                                                                  /* delay 1ms */

    return 0;                                                                             /* success return 0 */
}

/**
 * @brief      get temperature offset
 * @param[in]  *handle pointer to an scd4x handle structure
 * @param[out] *offset pointer to a temperature offset buffer
 * @return     status code
 *             - 0 success
 *             - 1 get temperature offset failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 *             - 4 crc error
 * @note       none
 */
uint8_t scd4x_get_temperature_offset(scd4x_handle_t *handle, uint16_t *offset)
{
    uint8_t res;
    uint8_t buf[3];

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    res = a_scd4x_iic_read(handle, SCD4X_COMMAND_GET_TEMPERATURE_OFFSET, buf, 3, 1);        /* read config */
    if (res != 0) {
        scd4x_handle_error(handle, "get temperature offset failed", 1);
        return 1;
    }

    res = scd4x_check_crc(buf, 2, "crc error", 4);
    if (res != 0) {
        return res;
    }

    *offset = (uint16_t)(((uint16_t)buf[0]) << 8) | buf[1];                                 /* set offset */

    return 0;                                                                               /* success return 0 */
}

/**
 * @brief      convert the temperature offset to the register raw data
 * @param[in]  *handle pointer to an scd4x handle structure
 * @param[in]  degrees set degrees
 * @param[out] *reg pointer to a register raw buffer
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t scd4x_temperature_offset_convert_to_register(scd4x_handle_t *handle, float degrees, uint16_t *reg)
{
    uint8_t res;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    *reg = (uint16_t)(degrees * (65535.0f / 175.0f));        /* convert real data to register data */

    return 0;                                                /* success return 0 */
}

/**
 * @brief      convert the register raw data to the temperature offset
 * @param[in]  *handle pointer to an scd4x handle structure
 * @param[in]  reg register raw data
 * @param[out] *degrees pointer to a temperature offset buffer
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t scd4x_temperature_offset_convert_to_data(scd4x_handle_t *handle, uint16_t reg, float *degrees)
{
    uint8_t res;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    *degrees = (float)(reg) * 175.0f / 65535.0f;        /* convert raw data to real data */

    return 0;                                           /* success return 0 */
}

/**
  * @brief     set sensor altitude
  * @param[in] *handle pointer to an scd4x handle structure
  * @param[in] altitude set altitude
  * @return    status code
  *            - 0 success
  *            - 1 set sensor altitude failed
  *            - 2 handle is NULL
  *            - 3 handle is not initialized
  * @note      none
  */
uint8_t scd4x_set_sensor_altitude(scd4x_handle_t *handle, uint16_t altitude)
{
    uint8_t res;
    uint8_t buf[3];

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    buf[0] = (altitude >> 8) & 0xFF;                                                    /* msb */
    buf[1] = (altitude >> 0) & 0xFF;                                                    /* lsb */
    buf[2] = a_scd4x_generate_crc(&buf[0], 2);                                          /* set crc */
    res = scd4x_handle_iic_operation_with_params(handle, a_scd4x_iic_write, SCD4X_COMMAND_SET_SENSOR_ALTITUDE, buf, 3,
                                                  "set sensor altitude failed", 1);
    if (res != 0) {
        return res;
    }
    handle->delay_ms(1);                                                                /* delay 1ms */

    return 0;                                                                           /* success return 0 */
}

/**
  * @brief      get sensor altitude
  * @param[in]  *handle pointer to an scd4x handle structure
  * @param[out] *altitude pointer to an altitude buffer
  * @return     status code
  *             - 0 success
  *             - 1 get sensor altitude failed
  *             - 2 handle is NULL
  *             - 3 handle is not initialized
  *             - 4 crc error
  * @note       none
  */
uint8_t scd4x_get_sensor_altitude(scd4x_handle_t *handle, uint16_t *altitude)
{
    uint8_t res;
    uint8_t buf[3];

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    res = a_scd4x_iic_read(handle, SCD4X_COMMAND_GET_SENSOR_ALTITUDE, buf, 3, 1);        /* read config */
    if (res != 0) {
        scd4x_handle_error(handle, "get sensor altitude failed", 1);
        return 1;
    }

    res = scd4x_check_crc(buf, 2, "crc error", 4);
    if (res != 0) {
        return res;
    }

    *altitude = (uint16_t)(((uint16_t)buf[0]) << 8) | buf[1];                            /* set offset */

    return 0;                                                                            /* success return 0 */
}

/**
  * @brief      get sensor variant
  * @param[in]  *handle pointer to an scd4x handle structure
  * @param[out] *sensor variant pointer to an altitude buffer
  * @return     status code
  *             - 0 success
  *             - 1 get sensor variant failed
  *             - 2 handle is NULL
  *             - 3 handle is not initialized
  *             - 4 crc error
  * @note       none
  */
uint8_t scd4x_get_sensor_variant(scd4x_handle_t *handle, uint16_t *variant) {
    uint8_t res;
    uint8_t buf[3];

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    res = a_scd4x_iic_read(handle, SCD4x_COMMAND_GET_SENSOR_VARIANT, buf, 3, 1);        /* read config */
    if (res != 0) {
        scd4x_handle_error(handle, "get sensor variant failed", 1);
        return 1;
    }

    res = scd4x_check_crc(buf, 2, "crc error", 4);
    if (res != 0) {
        return res;
    }

    *variant = (uint16_t)(((uint16_t)buf[0]) << 8) | buf[1];                            /* set offset */

    return 0;                                                                            /* success return 0 */
}

/**
  * @brief      convert the sensor altitude to the register raw data
  * @param[in]  *handle pointer to an scd4x handle structure
  * @param[in]  m set m
  * @param[out] *reg pointer to a register raw buffer
  * @return     status code
  *             - 0 success
  *             - 2 handle is NULL
  *             - 3 handle is not initialized
  * @note       none
  */
uint8_t scd4x_sensor_altitude_convert_to_register(scd4x_handle_t *handle, float m, uint16_t *reg)
{
    uint8_t res;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    *reg = (uint16_t)(m);           /* convert real data to register data */

    return 0;                       /* success return 0 */
}

/**
  * @brief      convert the register raw data to the sensor altitude
  * @param[in]  *handle pointer to an scd4x handle structure
  * @param[in]  reg register raw data
  * @param[out] *m pointer to an m buffer
  * @return     status code
  *             - 0 success
  *             - 2 handle is NULL
  *             - 3 handle is not initialized
  * @note       none
  */
uint8_t scd4x_sensor_altitude_convert_to_data(scd4x_handle_t *handle, uint16_t reg, float *m)
{
    uint8_t res;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    *m = (float)(reg);              /* convert raw data to real data */

    return 0;                       /* success return 0 */
}

/**
  * @brief     set ambient pressure
  * @param[in] *handle pointer to an scd4x handle structure
  * @param[in] pressure set pressure
  * @return    status code
  *            - 0 success
  *            - 1 set ambient pressure failed
  *            - 2 handle is NULL
  *            - 3 handle is not initialized
  * @note      none
  */
uint8_t scd4x_set_ambient_pressure(scd4x_handle_t *handle, uint16_t pressure)
{
    uint8_t res;
    uint8_t buf[3];

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    buf[0] = (pressure >> 8) & 0xFF;                                                    /* msb */
    buf[1] = (pressure >> 0) & 0xFF;                                                    /* lsb */
    buf[2] = a_scd4x_generate_crc(&buf[0], 2);                                          /* set crc */
    res = scd4x_handle_iic_operation_with_params(handle, a_scd4x_iic_write, SCD4X_COMMAND_SET_AMBIENT_PRESSURE, buf, 3,
                                                  "set ambient pressure failed", 1);
    if (res != 0) {
        return res;
    }
    handle->delay_ms(1);                                                                /* delay 1ms */

    return 0;                                                                           /* success return 0 */
}

/**
  * @brief      get ambient pressure
  * @param[in]  *handle pointer to an scd4x handle structure
  * @param[out] *pressure pointer to a pressure buffer
  * @return     status code
  *             - 0 success
  *             - 1 get ambient pressure failed
  *             - 2 handle is NULL
  *             - 3 handle is not initialized
  *             - 4 crc error
  * @note       none
  */
uint8_t scd4x_get_ambient_pressure(scd4x_handle_t *handle, uint16_t *pressure)
{
    uint8_t res;
    uint8_t buf[3];

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    res = a_scd4x_iic_read(handle, SCD4X_COMMAND_GET_AMBIENT_PRESSURE, buf, 3, 1);       /* read config */
    if (res != 0) {
        scd4x_handle_error(handle, "get ambient pressure failed", 1);
        return 1;
    }

    res = scd4x_check_crc(buf, 2, "crc error", 4);
    if (res != 0) {
        return res;
    }

    *pressure = (uint16_t)(((uint16_t)buf[0]) << 8) | buf[1];                            /* set offset */

    return 0;                                                                            /* success return 0 */
}

/**
  * @brief      convert the ambient pressure to the register raw data
  * @param[in]  *handle pointer to an scd4x handle structure
  * @param[in]  pa set pa
  * @param[out] *reg pointer to a register raw buffer
  * @return     status code
  *             - 0 success
  *             - 2 handle is NULL
  *             - 3 handle is not initialized
  * @note       none
  */
uint8_t scd4x_ambient_pressure_convert_to_register(scd4x_handle_t *handle, float pa, uint16_t *reg)
{
    uint8_t res;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    *reg = (uint16_t)(pa / 100.0f);           /* convert real data to register data */

    return 0;                                 /* success return 0 */
}

/**
  * @brief      convert the register raw data to the ambient pressure
  * @param[in]  *handle pointer to an scd4x handle structure
  * @param[in]  reg register raw data
  * @param[out] *pa pointer to a pa buffer
  * @return     status code
  *             - 0 success
  *             - 2 handle is NULL
  *             - 3 handle is not initialized
  * @note       none
  */
uint8_t scd4x_ambient_pressure_convert_to_data(scd4x_handle_t *handle, uint16_t reg, float *pa)
{
    uint8_t res;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    *pa = (float)(reg * 100.0f);        /* convert raw data to real data */

    return 0;                           /* success return 0 */
}

/**
  * @brief      perform forced recalibration
  * @param[in]  *handle pointer to an scd4x handle structure
  * @param[in]  co2_raw co2 raw data
  * @param[out] *frc pointer to a frc buffer
  * @return     status code
  *             - 0 success
  *             - 1 perform forced recalibration failed
  *             - 2 handle is NULL
  *             - 3 handle is not initialized
  *             - 4 crc error
  * @note       none
  */
uint8_t scd4x_perform_forced_recalibration(scd4x_handle_t *handle, uint16_t co2_raw, uint16_t *frc)
{
    uint8_t res;
    uint8_t in_buf[3];
    uint8_t out_buf[3];

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    in_buf[0] = (co2_raw >> 8) & 0xFF;                                                           /* set msb */
    in_buf[1] = (co2_raw >> 0) & 0xFF;                                                           /* set lsb */
    in_buf[2] = a_scd4x_generate_crc(&in_buf[0], 2);                                             /* set crc */
    res = a_scd4x_iic_read_with_param(handle, SCD4X_COMMAND_PERFORM_FORCED_RECALIBRATION,
                                       in_buf, 3, 400, out_buf, 3);                               /* read config */
    if (res != 0) {
        scd4x_handle_error(handle, "perform forced recalibration failed", 1);
        return 1;
    }

    res = scd4x_check_crc(out_buf, 2, "crc error", 4);
    if (res != 0) {
        return res;
    }

    *frc = (uint16_t)(((uint16_t)out_buf[0]) << 8) | out_buf[1];                                 /* set frc */

    return 0;                                                                                    /* success return 0 */
}

/**
  * @brief      convert the co2 to the register raw data
  * @param[in]  *handle pointer to an scd4x handle structure
  * @param[in]  ppm set ppm
  * @param[out] *reg pointer to a register raw buffer
  * @return     status code
  *             - 0 success
  *             - 2 handle is NULL
  *             - 3 handle is not initialized
  * @note       none
  */
uint8_t scd4x_co2_convert_to_register(scd4x_handle_t *handle, float ppm, uint16_t *reg)
{
    uint8_t res;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    *reg = (uint16_t)(ppm);         /* convert real data to register data */

    return 0;                       /* success return 0 */
}

/**
  * @brief      convert the register raw data to the co2
  * @param[in]  *handle pointer to an scd4x handle structure
  * @param[in]  reg register raw data
  * @param[out] *ppm pointer to a ppm buffer
  * @return     status code
  *             - 0 success
  *             - 2 handle is NULL
  *             - 3 handle is not initialized
  * @note       none
  */
uint8_t scd4x_co2_convert_to_data(scd4x_handle_t *handle, uint16_t reg, float *ppm)
{
    uint8_t res;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    *ppm = (float)(reg);            /* convert raw data to real data */

    return 0;                       /* success return 0 */
}

/**
  * @brief     enable or disable automatic self calibration
  * @param[in] *handle pointer to an scd4x handle structure
  * @param[in] enable bool value
  * @return    status code
  *            - 0 success
  *            - 1 set automatic self calibration failed
  *            - 2 handle is NULL
  *            - 3 handle is not initialized
  * @note      none
  */
uint8_t scd4x_set_automatic_self_calibration(scd4x_handle_t *handle, scd4x_bool_t enable)
{
    uint8_t res;
    uint8_t buf[3];
    uint16_t prev;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    prev = enable;                                                                           /* set bool */
    buf[0] = (prev >> 8) & 0xFF;                                                             /* msb */
    buf[1] = (prev >> 0) & 0xFF;                                                             /* lsb */
    buf[2] = a_scd4x_generate_crc(&buf[0], 2);                                               /* set crc */
    res = scd4x_handle_iic_operation_with_params(handle, a_scd4x_iic_write, SCD4X_COMMAND_SET_AUTO_SELF_CALIBRATION, buf, 3,
                                                  "set automatic self calibration failed", 1);
    if (res != 0) {
        return res;
    }
    handle->delay_ms(1);                                                                     /* delay 1ms */

    return 0;                                                                                /* success return 0 */
}

/**
  * @brief      get automatic self calibration status
  * @param[in]  *handle pointer to an scd4x handle structure
  * @param[out] *enable pointer to a bool buffer
  * @return     status code
  *             - 0 success
  *             - 1 get automatic self calibration failed
  *             - 2 handle is NULL
  *             - 3 handle is not initialized
  *             - 4 crc error
  * @note       none
  */
uint8_t scd4x_get_automatic_self_calibration(scd4x_handle_t *handle, scd4x_bool_t *enable)
{
    uint8_t res;
    uint8_t buf[3];
    uint16_t prev;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    res = a_scd4x_iic_read(handle, SCD4X_COMMAND_GET_AUTO_SELF_CALIBRATION, buf, 3, 1);       /* read config */
    if (res != 0) {
        scd4x_handle_error(handle, "get automatic self calibration failed", 1);
        return 1;
    }

    res = scd4x_check_crc(buf, 2, "crc error", 4);
    if (res != 0) {
        return res;
    }

    prev = (uint16_t)(((uint16_t)buf[0]) << 8) | buf[1];                                      /* set prev */
    *enable = (scd4x_bool_t)((prev >> 0) & 0x01);                                             /* get bool */

    return 0;                                                                                 /* success return 0 */
}

/**
  * @brief     start low power periodic measurement
  * @param[in] *handle pointer to an scd4x handle structure
  * @return    status code
  *            - 0 success
  *            - 1 start low power periodic measurement failed
  *            - 2 handle is NULL
  *            - 3 handle is not initialized
  * @note      none
  */
uint8_t scd4x_start_low_power_periodic_measurement(scd4x_handle_t *handle)
{
    uint8_t res;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_handle_iic_operation(handle, a_scd4x_iic_write, SCD4X_COMMAND_START_LOW_POWER_PERIODIC,
                                    "start low power periodic measurement failed", 1);
    if (res != 0) {
        return res;
    }

    return 0;                                                                                /* success return 0 */
}

/**
  * @brief      get data ready status
  * @param[in]  *handle pointer to an scd4x handle structure
  * @param[out] *enable pointer to a bool buffer
  * @return     status code
  *             - 0 success
  *             - 1 get data ready status failed
  *             - 2 handle is NULL
  *             - 3 handle is not initialized
  *             - 4 crc error
  * @note       none
  */
uint8_t scd4x_get_data_ready_status(scd4x_handle_t *handle, scd4x_bool_t *enable)
{
    uint8_t res;
    uint8_t buf[3];
    uint16_t prev;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    res = a_scd4x_iic_read(handle, SCD4X_COMMAND_GET_DATA_READY_STATUS, buf, 3, 1);       /* read config */
    if (res != 0) {
        scd4x_handle_error(handle, "get data ready status failed", 1);
        return 1;
    }

    res = scd4x_check_crc(buf, 2, "crc error", 4);
    if (res != 0) {
        return res;
    }

    prev = (uint16_t)(((uint16_t)buf[0]) << 8) | buf[1];                                  /* set prev */
    if ((prev & 0x0FFF) != 0)                                                             /* check data */
    {
        *enable = SCD4X_BOOL_TRUE;                                                        /* ready */
    }
    else
    {
        *enable = SCD4X_BOOL_FALSE;                                                       /* not ready */
    }

    return 0;                                                                             /* success return 0 */
}

/**
  * @brief     persist settings
  * @param[in] *handle pointer to an scd4x handle structure
  * @return    status code
  *            - 0 success
  *            - 1 persist settings failed
  *            - 2 handle is NULL
  *            - 3 handle is not initialized
  * @note      none
  */
uint8_t scd4x_persist_settings(scd4x_handle_t *handle)
{
    uint8_t res;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_handle_iic_operation(handle, a_scd4x_iic_write, SCD4X_COMMAND_PERSIST_SETTINGS,
                                    "persist settings failed", 1);
    if (res != 0) {
        return res;
    }
    handle->delay_ms(800);                                                           /* delay 800ms */

    return 0;                                                                        /* success return 0 */
}

/**
  * @brief      get serial number
  * @param[in]  *handle pointer to an scd4x handle structure
  * @param[out] *number pointer to a number buffer
  * @return     status code
  *             - 0 success
  *             - 1 get serial number failed
  *             - 2 handle is NULL
  *             - 3 handle is not initialized
  *             - 4 crc error
  * @note       none
  */
uint8_t scd4x_get_serial_number(scd4x_handle_t *handle, uint16_t number[3])
{
    uint8_t res;
    uint8_t buf[9];

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    res = a_scd4x_iic_read(handle, SCD4X_COMMAND_GET_SERIAL_NUMBER, buf, 9, 1);       /* read config */
    if (res != 0) {
        scd4x_handle_error(handle, "get serial number failed", 1);
        return 1;
    }

    res = scd4x_check_crc(buf, 2, "crc error", 4);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_crc(&buf[3], 2, "crc error", 4);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_crc(&buf[6], 2, "crc error", 4);
    if (res != 0) {
        return res;
    }

    number[0] = (uint16_t)(((uint16_t)buf[0]) << 8) | buf[1];                         /* set number0 */
    number[1] = (uint16_t)(((uint16_t)buf[3]) << 8) | buf[4];                         /* set number1 */
    number[2] = (uint16_t)(((uint16_t)buf[6]) << 8) | buf[7];                         /* set number2 */

    return 0;                                                                         /* success return 0 */
}

/**
  * @brief      perform self test
  * @param[in]  *handle pointer to an scd4x handle structure
  * @param[out] *malfunction_detected pointer to a bool value buffer
  * @return     status code
  *             - 0 success
  *             - 1 perform self test failed
  *             - 2 handle is NULL
  *             - 3 handle is not initialized
  *             - 4 crc error
  * @note       none
  */
uint8_t scd4x_perform_self_test(scd4x_handle_t *handle, scd4x_bool_t *malfunction_detected)
{
    uint8_t res;
    uint8_t buf[3];
    uint16_t prev;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    res = a_scd4x_iic_read(handle, SCD4X_COMMAND_PERFORM_SELF_TEST, buf, 3, 10000);       /* read config */
    if (res != 0) {
        scd4x_handle_error(handle, "perform self test failed", 1);
        return 1;
    }

    res = scd4x_check_crc(buf, 2, "crc error", 4);
    if (res != 0) {
        return res;
    }

    prev = (uint16_t)(((uint16_t)buf[0]) << 8) | buf[1];                                  /* set prev */
    if (prev != 0)                                                                        /* check data */
    {
        *malfunction_detected = SCD4X_BOOL_TRUE;                                          /* true */
    }
    else
    {
        *malfunction_detected = SCD4X_BOOL_FALSE;                                         /* false */
    }

    return 0;                                                                             /* success return 0 */
}

/**
  * @brief     perform factory reset
  * @param[in] *handle pointer to an scd4x handle structure
  * @return    status code
  *            - 0 success
  *            - 1 perform factory reset failed
  *            - 2 handle is NULL
  *            - 3 handle is not initialized
  * @note      none
  */
uint8_t scd4x_perform_factory_reset(scd4x_handle_t *handle)
{
    uint8_t res;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_handle_iic_operation(handle, a_scd4x_iic_write, SCD4X_COMMAND_PERFORM_FACTORY_RESET,
                                    "perform factory reset failed", 1);
    if (res != 0) {
        return res;
    }
    handle->delay_ms(1200);                                                               /* delay 1200ms */

    return 0;                                                                             /* success return 0 */
}

/**
  * @brief     reinit
  * @param[in] *handle pointer to an scd4x handle structure
  * @return    status code
  *            - 0 success
  *            - 1 reinit failed
  *            - 2 handle is NULL
  *            - 3 handle is not initialized
  * @note      none
  */
uint8_t scd4x_reinit(scd4x_handle_t *handle)
{
    uint8_t res;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_handle_iic_operation(handle, a_scd4x_iic_write, SCD4X_COMMAND_REINIT,
                                    "reinit failed", 1);
    if (res != 0) {
        return res;
    }
    handle->delay_ms(30);                                                  /* delay 30ms */

    return 0;                                                              /* success return 0 */
}

/**
  * @brief     measure single shot
  * @param[in] *handle pointer to an scd4x handle structure
  * @return    status code
  *            - 0 success
  *            - 1 measure single shot failed
  *            - 2 handle is NULL
  *            - 3 handle is not initialized
  *            - 4 only scd41 and scd43 has this function
  * @note      none
  */
uint8_t scd4x_measure_single_shot(scd4x_handle_t *handle)
{
    uint8_t res;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    if (handle->type == SCD40)                                                          /* check type */
    {
        scd4x_handle_error(handle, "only scd41 and scd43 has this function", 4);
        return 4;                                                                       /* return error */
    }

    res = scd4x_handle_iic_operation(handle, a_scd4x_iic_write, SCD4X_COMMAND_MEASURE_SINGLE_SHOT,
                                    "measure single shot failed", 1);
    if (res != 0) {
        return res;
    }
    handle->delay_ms(5000);                                                             /* delay 5000ms */

    return 0;                                                                           /* success return 0 */
}

/**
  * @brief     measure single shot rht only
  * @param[in] *handle pointer to an scd4x handle structure
  * @return    status code
  *            - 0 success
  *            - 1 measure single shot rht only failed
  *            - 2 handle is NULL
  *            - 3 handle is not initialized
  *            - 4 only scd41 and scd43 has this function
  * @note      none
  */
uint8_t scd4x_measure_single_shot_rht_only(scd4x_handle_t *handle)
{
    uint8_t res;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    if (handle->type == SCD40)                                                                   /* check type */
    {
        scd4x_handle_error(handle, "only scd41 and scd43 has this function", 4);
        return 4;                                                                                /* return error */
    }

    res = scd4x_handle_iic_operation(handle, a_scd4x_iic_write, SCD4X_COMMAND_MEASURE_SINGLE_SHOT_RHT_ONLY,
                                    "measure single shot rht only failed", 1);
    if (res != 0) {
        return res;
    }
    handle->delay_ms(50);                                                                        /* delay 50ms */

    return 0;                                                                                    /* success return 0 */
}

/**
  * @brief     power down
  * @param[in] *handle pointer to an scd4x handle structure
  * @return    status code
  *            - 0 success
  *            - 1 power down failed
  *            - 2 handle is NULL
  *            - 3 handle is not initialized
  *            - 4 only scd41 and scd43 has this function
  * @note      none
  */
uint8_t scd4x_power_down(scd4x_handle_t *handle)
{
    uint8_t res;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    if (handle->type == SCD40)                                                          /* check type */
    {
        scd4x_handle_error(handle, "only scd41 and scd43 has this function", 4);
        return 4;                                                                       /* return error */
    }

    res = scd4x_handle_iic_operation(handle, a_scd4x_iic_write, SCD4X_COMMAND_POWER_DOWN,
                                    "power down failed", 1);
    if (res != 0) {
        return res;
    }
    handle->delay_ms(1);                                                                /* delay 1ms */

    return 0;                                                                           /* success return 0 */
}

/**
  * @brief     wake up
  * @param[in] *handle pointer to an scd4x handle structure
  * @return    status code
  *            - 0 success
  *            - 1 wake up failed
  *            - 2 handle is NULL
  *            - 3 handle is not initialized
  *            - 4 only scd41 and scd43 has this function
  * @note      none
  */
uint8_t scd4x_wake_up(scd4x_handle_t *handle)
{
    uint8_t res;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    if (handle->type == SCD40)                                                          /* check type */
    {
        scd4x_handle_error(handle, "only scd41 and scd43 has this function", 4);
        return 4;                                                                       /* return error */
    }

    res = scd4x_handle_iic_operation(handle, a_scd4x_iic_write, SCD4X_COMMAND_WAKE_UP,
                                    "wake up failed", 1);
    if (res != 0) {
        return res;
    }
    handle->delay_ms(30);                                                               /* delay 30ms */

    return 0;                                                                           /* success return 0 */
}

/**
  * @brief     set automatic self calibration initial period
  * @param[in] *handle pointer to an scd4x handle structure
  * @param[in] hour set hour
  * @return    status code
  *            - 0 success
  *            - 1 set automatic self calibration initial period failed
  *            - 2 handle is NULL
  *            - 3 handle is not initialized
  *            - 4 only scd41 and scd43 has this function
  *            - 5 hour is not integer multiples of 4
  * @note      none
  */
uint8_t scd4x_set_automatic_self_calibration_initial_period(scd4x_handle_t *handle, uint16_t hour)
{
    uint8_t res;
    uint8_t buf[3];

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    if (handle->type == SCD40)                                                                           /* check type */
    {
        scd4x_handle_error(handle, "only scd41 and scd43 has this function", 4);
        return 4;                                                                                        /* return error */
    }

    if ((hour % 4) != 0)                                                                                 /* check hour */
    {
        scd4x_handle_error(handle, "hour is not integer multiples of 4", 5);
        return 5;                                                                                        /* return error */
    }

    buf[0] = (hour >> 8) & 0xFF;                                                                         /* msb */
    buf[1] = (hour >> 0) & 0xFF;                                                                         /* lsb */
    buf[2] = a_scd4x_generate_crc(&buf[0], 2);                                                           /* crc */
    res = scd4x_handle_iic_operation_with_params(handle, a_scd4x_iic_write, SCD4X_COMMAND_SET_AUTO_SELF_CALIBRATION_INIT_PERIOD, buf, 3,
                                                  "set automatic self calibration initial period failed", 1);
    if (res != 0) {
        return res;
    }
    handle->delay_ms(1);                                                                                 /* delay 1ms */

    return 0;                                                                                            /* success return 0 */
}

/**
  * @brief      get automatic self calibration initial period
  * @param[in]  *handle pointer to an scd4x handle structure
  * @param[out] *hour pointer to an hour buffer
  * @return     status code
  *             - 0 success
  *             - 1 get automatic self calibration initial period failed
  *             - 2 handle is NULL
  *             - 3 handle is not initialized
  *             - 4 only scd41 and scd43 has this function
  *             - 5 crc error
  * @note       none
  */
uint8_t scd4x_get_automatic_self_calibration_initial_period(scd4x_handle_t *handle, uint16_t *hour)
{
    uint8_t res;
    uint8_t buf[3];

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    if (handle->type == SCD40)                                                                             /* check type */
    {
        scd4x_handle_error(handle, "only scd41 and scd43 has this function", 4);
        return 4;                                                                                          /* return error */
    }

    res = a_scd4x_iic_read(handle, SCD4X_COMMAND_GET_AUTO_SELF_CALIBRATION_INIT_PERIOD, buf, 3, 1);        /* read config */
    if (res != 0) {
        scd4x_handle_error(handle, "get automatic self calibration initial period failed", 1);
        return 1;
    }

    res = scd4x_check_crc(buf, 2, "crc error", 5);
    if (res != 0) {
        return res;
    }

    *hour = (uint16_t)(((uint16_t)buf[0]) << 8) | buf[1];                                                  /* get hour */

    return 0;                                                                                              /* success return 0 */
}

/**
  * @brief     set automatic self calibration standard period
  * @param[in] *handle pointer to an scd4x handle structure
  * @param[in] hour set hour
  * @return    status code
  *            - 0 success
  *            - 1 set automatic self calibration standard period failed
  *            - 2 handle is NULL
  *            - 3 handle is not initialized
  *            - 4 only scd41 and scd43 has this function
  *            - 5 hour is not integer multiples of 4
  * @note      none
  */
uint8_t scd4x_set_automatic_self_calibration_standard_period(scd4x_handle_t *handle, uint16_t hour)
{
    uint8_t res;
    uint8_t buf[3];

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    if (handle->type == SCD40)                                                                           /* check type */
    {
        scd4x_handle_error(handle, "only scd41 and scd43 has this function", 4);
        return 4;                                                                                        /* return error */
    }

    if ((hour % 4) != 0)                                                                                 /* check hour */
    {
        scd4x_handle_error(handle, "hour is not integer multiples of 4", 5);
        return 5;                                                                                        /* return error */
    }

    buf[0] = (hour >> 8) & 0xFF;                                                                         /* msb */
    buf[1] = (hour >> 0) & 0xFF;                                                                         /* lsb */
    buf[2] = a_scd4x_generate_crc(&buf[0], 2);                                                           /* crc */
    res = scd4x_handle_iic_operation_with_params(handle, a_scd4x_iic_write, SCD4X_COMMAND_SET_AUTO_SELF_CALIBRATION_STANDARD_PERIOD, buf, 3,
                                                  "set automatic self calibration standard period failed", 1);
    if (res != 0) {
        return res;
    }
    handle->delay_ms(1);                                                                                 /* delay 1ms */

    return 0;                                                                                            /* success return 0 */
}

/**
  * @brief      get automatic self calibration standard period
  * @param[in]  *handle pointer to an scd4x handle structure
  * @param[out] *hour pointer to an hour buffer
  * @return     status code
  *             - 0 success
  *             - 1 get automatic self calibration standard period failed
  *             - 2 handle is NULL
  *             - 3 handle is not initialized
  *             - 4 only scd41 and scd43 has this function
  *             - 5 crc error
  * @note       none
  */
uint8_t scd4x_get_automatic_self_calibration_standard_period(scd4x_handle_t *handle, uint16_t *hour)
{
    uint8_t res;
    uint8_t buf[3];

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    if (handle->type == SCD40)                                                                             /* check type */
    {
        scd4x_handle_error(handle, "only scd41 and scd43 has this function", 4);
        return 4;                                                                                          /* return error */
    }

    res = a_scd4x_iic_read(handle, SCD4X_COMMAND_GET_AUTO_SELF_CALIBRATION_STANDARD_PERIOD, buf, 3, 1);    /* read config */
    if (res != 0) {
        scd4x_handle_error(handle, "get automatic self calibration standard period failed", 1);
        return 1;
    }

    res = scd4x_check_crc(buf, 2, "crc error", 5);
    if (res != 0) {
        return res;
    }

    *hour = (uint16_t)(((uint16_t)buf[0]) << 8) | buf[1];                                                  /* get hour */

    return 0;                                                                                              /* success return 0 */
}

/**
  * @brief     initialize the chip
  * @param[in] *handle pointer to an scd4x handle structure
  * @return    status code
  *            - 0 success
  *            - 1 iic initialization failed
  *            - 2 handle is NULL
  *            - 3 linked functions is NULL
  * @note      none
  */
uint8_t scd4x_init(scd4x_handle_t *handle)
{
    uint8_t res;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    if (handle->debug_print == NULL)                                         /* check debug_print */
    {
        return 3;                                                            /* return error */
    }

    if (handle->iic_init == NULL)                                            /* check iic_init */
    {
        scd4x_handle_error(handle, "iic_init is null", 3);
        return 3;                                                            /* return error */
    }
    if (handle->iic_deinit == NULL)                                          /* check iic_deinit */
    {
        scd4x_handle_error(handle, "iic_deinit is null", 3);
        return 3;                                                            /* return error */
    }
    if (handle->iic_write_cmd == NULL)                                       /* check iic_write_cmd */
    {
        scd4x_handle_error(handle, "iic_write_cmd is null", 3);
        return 3;                                                            /* return error */
    }
    if (handle->iic_read_cmd == NULL)                                        /* check iic_read_cmd */
    {
        scd4x_handle_error(handle, "iic_read_cmd is null", 3);
        return 3;                                                            /* return error */
    }
    if (handle->delay_ms == NULL)                                            /* check delay_ms */
    {
        scd4x_handle_error(handle, "delay_ms is null", 3);
        return 3;                                                            /* return error */
    }

    if (handle->iic_init() != 0)                                             /* iic init */
    {
        scd4x_handle_error(handle, "iic init failed", 1);
        return 1;                                                            /* return error */
    }
    handle->inited = 1;                                                      /* flag finish initialization */

    return 0;                                                                /* success return 0 */
}

/**
  * @brief     close the chip
  * @param[in] *handle pointer to an scd4x handle structure
  * @return    status code
  *            - 0 success
  *            - 1 iic deinit failed
  *            - 2 handle is NULL
  *            - 3 handle is not initialized
  *            - 4 stop periodic measurement failed
  * @note      none
  */
uint8_t scd4x_deinit(scd4x_handle_t *handle)
{
    uint8_t res;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_handle_iic_operation(handle, a_scd4x_iic_write, SCD4X_COMMAND_STOP_PERIODIC,
                                    "stop periodic measurement failed", 4);
    if (res != 0) {
        return res;
    }

    if (handle->iic_deinit() != 0)                                                /* iic deinit */
    {
        scd4x_handle_error(handle, "iic close failed", 3);
        return 3;                                                                 /* return error */
    }
    handle->inited = 0;                                                           /* flag close initialization */

    return 0;                                                                     /* success return 0 */
}

/**
  * @brief     set the chip register
  * @param[in] *handle pointer to an scd4x handle structure
  * @param[in] reg iic register address
  * @param[in] *buf pointer to a data buffer
  * @param[in] len data buffer length
  * @return    status code
  *            - 0 success
  *            - 1 write failed
  *            - 2 handle is NULL
  *            - 3 handle is not initialized
  * @note      none
  */
uint8_t scd4x_set_reg(scd4x_handle_t *handle, uint16_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t res;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    return a_scd4x_iic_write(handle, reg, buf, len);          /* write data */
}

/**
  * @brief      get the chip register
  * @param[in]  *handle pointer to an scd4x handle structure
  * @param[in]  reg iic register address
  * @param[out] *buf pointer to a data buffer
  * @param[in]  len data buffer length
  * @param[in]  delay_ms delay time in ms
  * @return     status code
  *             - 0 success
  *             - 1 read failed
  *             - 2 handle is NULL
  *             - 3 handle is not initialized
  * @note       none
  */
uint8_t scd4x_get_reg(scd4x_handle_t *handle, uint16_t reg, uint8_t *buf, uint16_t len, uint16_t delay_ms)
{
    uint8_t res;

    res = scd4x_check_handle(handle);
    if (res != 0) {
        return res;
    }

    res = scd4x_check_initialized(handle);
    if (res != 0) {
        return res;
    }

    return a_scd4x_iic_read(handle, reg, buf, len, delay_ms);      /* read data */
}

/**
  * @brief      get chip information
  * @param[out] *info pointer to an scd4x info structure
  * @return     status code
  *             - 0 success
  *             - 2 handle is NULL
  * @note       none
  */
uint8_t scd4x_info(scd4x_info_t *info)
{
    uint8_t res;

    res = scd4x_check_handle((scd4x_handle_t *)info);
    if (res != 0) {
        return res;
    }

    memset(info, 0, sizeof(scd4x_info_t));                          /* initialize scd4x info structure */
    strncpy(info->chip_name, CHIP_NAME, 32);                        /* copy chip name */
    strncpy(info->manufacturer_name, MANUFACTURER_NAME, 32);        /* copy manufacturer name */
    strncpy(info->interface, "IIC", 8);                             /* copy interface name */
    info->supply_voltage_min_v = SUPPLY_VOLTAGE_MIN;                /* set minimal supply voltage */
    info->supply_voltage_max_v = SUPPLY_VOLTAGE_MAX;                /* set maximum supply voltage */
    info->max_current_ma = MAX_CURRENT;                             /* set maximum current */
    info->temperature_max = TEMPERATURE_MAX;                        /* set minimal temperature */
    info->temperature_min = TEMPERATURE_MIN;                        /* set maximum temperature */
    info->driver_version = DRIVER_VERSION;                          /* set driver version */

    return 0;                                                       /* success return 0 */
}