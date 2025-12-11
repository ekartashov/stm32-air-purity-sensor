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
 * @file      driver_sen5x.c
 * @brief     driver sen5x source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2025-06-15
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2025/06/15  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_sen5x.h"

/**
 * @brief chip information definition
 */
#define CHIP_NAME                 "Sensirion SEN5X"        /**< chip name */
#define MANUFACTURER_NAME         "Sensirion"              /**< manufacturer name */
#define SUPPLY_VOLTAGE_MIN        4.50f                    /**< chip min supply voltage */
#define SUPPLY_VOLTAGE_MAX        5.50f                    /**< chip max supply voltage */
#define MAX_CURRENT               100.0f                   /**< chip max current */
#define TEMPERATURE_MIN           -10.0f                   /**< chip min operating temperature */
#define TEMPERATURE_MAX           60.0f                    /**< chip max operating temperature */
#define DRIVER_VERSION            1000                     /**< driver version */

/**
 * @brief chip address definition
 */
#define SEN5X_ADDRESS             (0x69)              /**< chip iic address */

/**
 * @brief chip command definition
 */
#define SEN5X_IIC_COMMAND_START_MEASUREMENT                        0x0021U        /**< start measurement command */
#define SEN5X_IIC_COMMAND_START_MEASUREMENT_IN_RHT_GAS_ONLY        0x0037U        /**< start measurement in rht/gas-only measurement mode command */
#define SEN5X_IIC_COMMAND_STOP_MEASUREMENT                         0x0104U        /**< stop measurement command */
#define SEN5X_IIC_COMMAND_READ_DATA_READY_FLAG                     0x0202U        /**< read data ready flag command */
#define SEN5X_IIC_COMMAND_READ_MEASURED_VALUES                     0x03C4U        /**< read measured values command */
#define SEN5X_IIC_COMMAND_READ_WRITE_TEMPERATURE_COMPENSATION      0x60B2U        /**< read/write temperature compensation parameters command */
#define SEN5X_IIC_COMMAND_READ_WRITE_WARM_START_PARAMS             0x60C6U        /**< read/write warm start parameters command */
#define SEN5X_IIC_COMMAND_READ_WRITE_VOC_ALG_TUNING_PARAMS         0x60D0U        /**< read/write voc algorithm tuning parameters command */
#define SEN5X_IIC_COMMAND_READ_WRITE_NOX_ALG_TUNING_PARAMS         0x60E1U        /**< read/write nox algorithm tuning parameters command */
#define SEN5X_IIC_COMMAND_READ_WRITE_RHT_ACC_MODE                  0x60F7U        /**< read/write data and parameters command */
#define SEN5X_IIC_COMMAND_READ_WRITE_VOC_ALG_STATE                 0x6181U        /**< read/write voc algorithm state command */
#define SEN5X_IIC_COMMAND_START_FAN_CLEANING                       0x5607U        /**< start fan cleaning command */
#define SEN5X_IIC_COMMAND_READ_WRITE_AUTO_CLEANING_INTERVAL        0x8004U        /**< read/write auto cleaning interval command */
#define SEN5X_IIC_COMMAND_READ_PRODUCT_NAME                        0xD014U        /**< read product name command */
#define SEN5X_IIC_COMMAND_READ_SERIAL_NUMBER                       0xD033U        /**< read serial number command */
#define SEN5X_IIC_COMMAND_READ_VERSION                             0xD100U        /**< read version command */
#define SEN5X_IIC_COMMAND_READ_DEVICE_STATUS_REG                   0xD206U        /**< read device status register command */
#define SEN5X_IIC_COMMAND_CLEAR_DEVICE_STATUS_REG                  0xD210U        /**< clear device status register command */
#define SEN5X_IIC_COMMAND_RESET                                    0xD304U        /**< reset command */
#define SEN5X_IIC_COMMAND_READ_MEASURED_PM_VALUES                  0x0413U        /**< read measured pm values command */
#define SEN5X_IIC_COMMAND_READ_MEASURED_RAW_VALUES                 0x03D2U        /**< read measured raw values command */

/**
 * @brief     generate the crc
 * @param[in] *handle pointer to a sen5x handle structure
 * @param[in] *data pointer to a data buffer
 * @param[in] count data length
 * @return    crc
 * @note      none
 */
static uint8_t a_sen5x_crc(sen5x_handle_t *handle, uint8_t* data, uint8_t count) 
{
    uint8_t current_byte;
    uint8_t crc = 0xFF;
    uint8_t crc_bit;

    for (current_byte = 0; current_byte < count; ++current_byte)        /* calculate crc */
    {
        crc ^= (data[current_byte]);                                    /* xor data */
        for (crc_bit = 8; crc_bit > 0; --crc_bit)                       /* 8 bit */
        {
            if ((crc & 0x80) != 0)                                      /* if 7th bit is 1 */
            {
                crc = (crc << 1) ^ 0x31;                                /* xor */
            }
            else
            {
                crc = crc << 1;                                         /* left shift 1 */
            }
        }
    }
    
    return crc;                                                         /* return crc */
}

/**
 * @brief      read bytes
 * @param[in]  *handle pointer to a sen5x handle structure
 * @param[in]  addr iic device address
 * @param[in]  reg iic register address
 * @param[out] *data pointer to a data buffer
 * @param[in]  len data length
 * @param[in]  delay_ms delay time in ms
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
static uint8_t a_sen5x_iic_read(sen5x_handle_t *handle, uint8_t addr, uint16_t reg, uint8_t *data, uint16_t len, uint16_t delay_ms)
{
    uint8_t buf[2];
    
    buf[0] = (reg >> 8) & 0xFF;                                  /* set msb */
    buf[1] = (reg >> 0) & 0xFF;                                  /* set lsb */
    if (handle->iic_write_cmd(addr, (uint8_t *)buf, 2) != 0)     /* write data */
    {
        return 1;                                                /* return error */
    }
    handle->delay_ms(delay_ms);                                  /* delay ms */
    if (handle->iic_read_cmd(addr, (uint8_t *)data, len) != 0)   /* read data */
    {
        return 1;                                                /* return error */
    }
    
    return 0;                                                    /* success return 0 */
}

/**
 * @brief     write bytes
 * @param[in] *handle pointer to a sen5x handle structure
 * @param[in] addr iic device address
 * @param[in] reg iic register address
 * @param[in] *data pointer to a data buffer
 * @param[in] len data length
 * @param[in] delay_ms delay time in ms
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
static uint8_t a_sen5x_iic_write(sen5x_handle_t *handle, uint8_t addr, uint16_t reg, uint8_t *data, uint16_t len, uint16_t delay_ms)
{
    uint8_t buf[32];
    
    if (len > 30)                                                    /* check length */
    {
        return 1;                                                    /* return error */
    }
    buf[0] = (reg >> 8) & 0xFF;                                      /* set msb */
    buf[1] = (reg >> 0) & 0xFF;                                      /* set lsb */
    memcpy((uint8_t *)&buf[2], data, len);                           /* copy data */
    if (handle->iic_write_cmd(addr, (uint8_t *)buf, len + 2) != 0)   /* write data */
    {
        return 1;                                                    /* return error */
    }
    handle->delay_ms(delay_ms);                                      /* delay ms */
    
    return 0;                                                        /* success return 0 */
}

/**
 * @brief     start the measurement
 * @param[in] *handle pointer to a sen5x handle structure
 * @return    status code
 *            - 0 success
 *            - 1 start measurement failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t sen5x_start_measurement(sen5x_handle_t *handle)
{
    uint8_t res;
    
    if (handle == NULL)                                                                                     /* check handle */
    {
        return 2;                                                                                           /* return error */
    }
    if (handle->inited != 1)                                                                                /* check handle initialization */
    {
        return 3;                                                                                           /* return error */
    }
    
    res = a_sen5x_iic_write(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_START_MEASUREMENT, NULL, 0, 50);       /* start measurement command */
    if (res != 0)                                                                                           /* check result */
    {
        handle->debug_print("sen5x: start measurement failed.\n");                                          /* start measurement failed */
       
        return 1;                                                                                           /* return error */
    }
    
    return 0;                                                                                               /* success return 0 */
}

/**
 * @brief     start the measurement without pm
 * @param[in] *handle pointer to a sen5x handle structure
 * @return    status code
 *            - 0 success
 *            - 1 start measurement failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 sen50 doesn't support this function
 * @note      sen50 doesn't support this function
 */
uint8_t sen5x_start_measurement_without_pm(sen5x_handle_t *handle)
{
    uint8_t res;
    
    if (handle == NULL)                                                                               /* check handle */
    {
        return 2;                                                                                     /* return error */
    }
    if (handle->inited != 1)                                                                          /* check handle initialization */
    {
        return 3;                                                                                     /* return error */
    }
    if (handle->type == (uint8_t)(SEN50))                                                             /* check result */
    {
        handle->debug_print("sen5x: sen50 doesn't support this function.\n");                         /* sen50 doesn't support this function */
       
        return 4;                                                                                     /* return error */
    }
    
    res = a_sen5x_iic_write(handle, SEN5X_ADDRESS,
                            SEN5X_IIC_COMMAND_START_MEASUREMENT_IN_RHT_GAS_ONLY, NULL, 0, 50);        /* start measurement command */
    if (res != 0)                                                                                     /* check result */
    {
        handle->debug_print("sen5x: start measurement without pm failed.\n");                         /* start measurement failed */
       
        return 1;                                                                                     /* return error */
    }
    
    return 0;                                                                                         /* success return 0 */
}

/**
 * @brief     stop the measurement
 * @param[in] *handle pointer to a sen5x handle structure
 * @return    status code
 *            - 0 success
 *            - 1 stop measurement failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t sen5x_stop_measurement(sen5x_handle_t *handle)
{
    uint8_t res;
    
    if (handle == NULL)                                                                                    /* check handle */
    {
        return 2;                                                                                          /* return error */
    }
    if (handle->inited != 1)                                                                               /* check handle initialization */
    {
        return 3;                                                                                          /* return error */
    }
    
    res = a_sen5x_iic_write(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_STOP_MEASUREMENT, NULL, 0, 200);      /* stop measurement command */
    if (res != 0)                                                                                          /* check result */
    {
        handle->debug_print("sen5x: stop measurement failed.\n");                                          /* stop measurement failed */
       
        return 1;                                                                                          /* return error */
    }
        
    return 0;                                                                                              /* success return 0 */
}

/**
 * @brief      read the data flag
 * @param[in]  *handle pointer to a sen5x handle structure
 * @param[out] *flag pointer to a data ready flag buffer
 * @return     status code
 *             - 0 success
 *             - 1 read data flag failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t sen5x_read_data_flag(sen5x_handle_t *handle, sen5x_data_ready_flag_t *flag)
{
    uint8_t res;
    uint8_t buf[3];
    
    if (handle == NULL)                                                                                                  /* check handle */
    {
        return 2;                                                                                                        /* return error */
    }
    if (handle->inited != 1)                                                                                             /* check handle initialization */
    {
        return 3;                                                                                                        /* return error */
    }
    
    memset(buf, 0, sizeof(uint8_t) * 3);                                                                                 /* clear the buffer */
    res = a_sen5x_iic_read(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_DATA_READY_FLAG, (uint8_t *)buf, 3, 20);        /* read data ready flag command */
    if (res != 0)                                                                                                        /* check result */
    {
        handle->debug_print("sen5x: read data ready flag failed.\n");                                                    /* read data ready flag failed */
       
        return 1;                                                                                                        /* return error */
    }
    if (buf[2] != a_sen5x_crc(handle, (uint8_t *)buf, 2))                                                                /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                                               /* crc check failed */
       
        return 1;                                                                                                        /* return error */
    }
    *flag = (sen5x_data_ready_flag_t)(buf[1] & 0x01);                                                                    /* get the data ready flag */
        
    return 0;                                                                                                            /* success return 0 */
}

/**
 * @brief      read the result
 * @param[in]  *handle pointer to a sen5x handle structure
 * @param[out] *output pointer to a data structure
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 *             - 4 only sen55 can use this function
 * @note       only sen55 can use this function
 */
uint8_t sen55_read(sen5x_handle_t *handle, sen55_data_t *output)
{
    uint8_t res;
    uint8_t i;
    uint8_t check[3];
    uint8_t buf[24];
    
    if ((handle == NULL) || (output == NULL))                                                                               /* check handle */
    {
        return 2;                                                                                                           /* return error */
    }
    if (handle->inited != 1)                                                                                                /* check handle initialization */
    {
        return 3;                                                                                                           /* return error */
    }
    if (handle->type != (uint8_t)(SEN55))                                                                                   /* check type */
    {
        handle->debug_print("sen5x: only sen55 can use this function.\n");                                                  /* only sen55 can use this function */
       
        return 4;                                                                                                           /* return error */
    }
    
    memset(check, 0, sizeof(uint8_t) * 3);                                                                                  /* clear the buffer */
    res = a_sen5x_iic_read(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_DATA_READY_FLAG, (uint8_t *)check, 3, 20);         /* read data ready flag command */
    if (res != 0)                                                                                                           /* check result */
    {
        handle->debug_print("sen5x: read data ready flag failed.\n");                                                       /* read data ready flag failed */
       
        return 1;                                                                                                           /* return error */
    }
    if (check[2] != a_sen5x_crc(handle, (uint8_t *)check, 2))                                                               /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                                                  /* crc check failed */
       
        return 1;                                                                                                           /* return error */
    }
    if ((check[1] & 0x01) == 0)                                                                                             /* check flag */
    {
        handle->debug_print("sen5x: data not ready.\n");                                                                    /* data not ready */
       
        return 1;                                                                                                           /* return error */
    }
    
    memset(buf, 0, sizeof(uint8_t) * 24);                                                                                   /* clear the buffer */
    res = a_sen5x_iic_read(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_MEASURED_VALUES, (uint8_t *)buf, 24, 20);          /* read measured values command */
    if (res != 0)                                                                                                           /* check result */
    {
        handle->debug_print("sen5x: read measured values failed.\n");                                                       /* read measured values failed */
       
        return 1;                                                                                                           /* return error */
    }
    for (i = 0; i < 8; i++)                                                                                                 /* check crc */
    {
        if (buf[i * 3 + 2] != a_sen5x_crc(handle, (uint8_t *)&buf[i * 3], 2))                                               /* check crc */
        {
            handle->debug_print("sen5x: crc is error.\n");                                                                  /* crc is error */
           
            return 1;                                                                                                       /* return error */
        }
    }
    output->pm1p0_raw = (uint16_t)(((uint16_t)(buf[0]) << 8) | ((uint16_t)(buf[1]) << 0));                                  /* copy pm1.0 */
    output->pm2p5_raw = (uint16_t)(((uint16_t)(buf[3]) << 8) | ((uint16_t)(buf[4]) << 0));                                  /* copy pm2.5 */
    output->pm4p0_raw = (uint16_t)(((uint16_t)(buf[6]) << 8) | ((uint16_t)(buf[7]) << 0));                                  /* copy pm4.0 */
    output->pm10_raw = (uint16_t)(((uint16_t)(buf[9]) << 8) | ((uint16_t)(buf[10]) << 0));                                  /* copy pm10 */
    output->compensated_ambient_humidity_raw = (int16_t)(((uint16_t)(buf[12]) << 8) | ((uint16_t)(buf[13]) << 0));          /* humidity raw */
    output->compensated_ambient_temperature_raw = (int16_t)(((uint16_t)(buf[15]) << 8) | ((uint16_t)(buf[16]) << 0));       /* temperature raw */
    output->voc_raw = (int16_t)(((uint16_t)(buf[18]) << 8) | ((uint16_t)(buf[19]) << 0));                                   /* voc raw */
    output->nox_raw = (int16_t)(((uint16_t)(buf[21]) << 8) | ((uint16_t)(buf[22]) << 0));                                   /* nox raw */
    if ((output->pm1p0_raw == 0xFFFFU) || (output->pm2p5_raw == 0xFFFFU) || 
        (output->pm4p0_raw == 0xFFFFU) || (output->pm10_raw == 0xFFFFU))                                                    /* check invalid */
    {
        output->pm1p0_ug_m3 = FP_NAN;                                                                                       /* set nan */
        output->pm2p5_ug_m3 = FP_NAN;                                                                                       /* set nan */
        output->pm4p0_ug_m3 = FP_NAN;                                                                                       /* set nan */
        output->pm10_ug_m3 = FP_NAN;                                                                                        /* set nan */
        output->pm_valid = 0;                                                                                               /* set invalid */
    }
    else
    {
        output->pm1p0_ug_m3 = (float)(output->pm1p0_raw) / 10.0f;                                                           /* convert pm1.0 */
        output->pm2p5_ug_m3 = (float)(output->pm2p5_raw) / 10.0f;                                                           /* convert pm2.5 */
        output->pm4p0_ug_m3 = (float)(output->pm4p0_raw) / 10.0f;                                                           /* convert pm4.0 */
        output->pm10_ug_m3 = (float)(output->pm10_raw) / 10.0f;                                                             /* convert pm10 */
        output->pm_valid = 1;                                                                                               /* set valid */
    }
    if (output->compensated_ambient_humidity_raw == 0x7FFF)                                                                 /* check humidity raw */
    {
        output->compensated_ambient_humidity_percentage = FP_NAN;                                                           /* set nan */
    }
    else                                                                                                                    /* convert */
    {
        output->compensated_ambient_humidity_percentage = (float)(output->compensated_ambient_humidity_raw) / 100.0f;       /* convert humidity */
    }
    if (output->compensated_ambient_temperature_raw == 0x7FFF)                                                              /* check temperature raw */
    {
        output->compensated_ambient_temperature_degree = FP_NAN;                                                            /* set nan */
    }
    else
    {
        output->compensated_ambient_temperature_degree = (float)(output->compensated_ambient_temperature_raw) / 200.0f;     /* convert temperature */
    }
    if (output->voc_raw == 0x7FFF)                                                                                          /* check voc raw */
    {
        output->voc_index = FP_NAN;                                                                                         /* set nan */
    }
    else
    {
        output->voc_index = (float)(output->voc_raw) / 10.0f;                                                               /* convert voc */
    }
    if (output->nox_raw == 0x7FFF)                                                                                          /* check nox raw */
    {
        output->nox_index = FP_NAN;                                                                                         /* set nan */
    }
    else
    {
        output->nox_index = (float)(output->nox_raw) / 10.0f;                                                               /* convert nox */
    }
    
    return 0;                                                                                                               /* success return 0 */
}

/**
 * @brief      read the result
 * @param[in]  *handle pointer to a sen5x handle structure
 * @param[out] *output pointer to a data structure
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 *             - 4 only sen54 can use this function
 * @note       only sen54 can use this function
 */
uint8_t sen54_read(sen5x_handle_t *handle, sen54_data_t *output)
{
    uint8_t res;
    uint8_t i;
    uint8_t check[3];
    uint8_t buf[24];
    
    if ((handle == NULL) || (output == NULL))                                                                               /* check handle */
    {
        return 2;                                                                                                           /* return error */
    }
    if (handle->inited != 1)                                                                                                /* check handle initialization */
    {
        return 3;                                                                                                           /* return error */
    }
    if (handle->type != (uint8_t)(SEN54))                                                                                   /* check type */
    {
        handle->debug_print("sen5x: only sen54 can use this function.\n");                                                  /* only sen54 can use this function */
       
        return 4;                                                                                                           /* return error */
    }
    
    memset(check, 0, sizeof(uint8_t) * 3);                                                                                  /* clear the buffer */
    res = a_sen5x_iic_read(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_DATA_READY_FLAG, (uint8_t *)check, 3, 20);         /* read data ready flag command */
    if (res != 0)                                                                                                           /* check result */
    {
        handle->debug_print("sen5x: read data ready flag failed.\n");                                                       /* read data ready flag failed */
       
        return 1;                                                                                                           /* return error */
    }
    if (check[2] != a_sen5x_crc(handle, (uint8_t *)check, 2))                                                               /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                                                  /* crc check failed */
       
        return 1;                                                                                                           /* return error */
    }
    if ((check[1] & 0x01) == 0)                                                                                             /* check flag */
    {
        handle->debug_print("sen5x: data not ready.\n");                                                                    /* data not ready */
       
        return 1;                                                                                                           /* return error */
    }
    
    memset(buf, 0, sizeof(uint8_t) * 24);                                                                                   /* clear the buffer */
    res = a_sen5x_iic_read(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_MEASURED_VALUES, (uint8_t *)buf, 24, 20);          /* read measured values command */
    if (res != 0)                                                                                                           /* check result */
    {
        handle->debug_print("sen5x: read measured values failed.\n");                                                       /* read measured values failed */
       
        return 1;                                                                                                           /* return error */
    }
    for (i = 0; i < 8; i++)                                                                                                 /* check crc */
    {
        if (buf[i * 3 + 2] != a_sen5x_crc(handle, (uint8_t *)&buf[i * 3], 2))                                               /* check crc */
        {
            handle->debug_print("sen5x: crc is error.\n");                                                                  /* crc is error */
           
            return 1;                                                                                                       /* return error */
        }
    }
    output->pm1p0_raw = (uint16_t)(((uint16_t)(buf[0]) << 8) | ((uint16_t)(buf[1]) << 0));                                  /* copy pm1.0 */
    output->pm2p5_raw = (uint16_t)(((uint16_t)(buf[3]) << 8) | ((uint16_t)(buf[4]) << 0));                                  /* copy pm2.5 */
    output->pm4p0_raw = (uint16_t)(((uint16_t)(buf[6]) << 8) | ((uint16_t)(buf[7]) << 0));                                  /* copy pm4.0 */
    output->pm10_raw = (uint16_t)(((uint16_t)(buf[9]) << 8) | ((uint16_t)(buf[10]) << 0));                                  /* copy pm10 */
    output->compensated_ambient_humidity_raw = (int16_t)(((uint16_t)(buf[12]) << 8) | ((uint16_t)(buf[13]) << 0));          /* humidity raw */
    output->compensated_ambient_temperature_raw = (int16_t)(((uint16_t)(buf[15]) << 8) | ((uint16_t)(buf[16]) << 0));       /* temperature raw */
    output->voc_raw = (int16_t)(((uint16_t)(buf[18]) << 8) | ((uint16_t)(buf[19]) << 0));                                   /* voc raw */
    if ((output->pm1p0_raw == 0xFFFFU) || (output->pm2p5_raw == 0xFFFFU) || 
        (output->pm4p0_raw == 0xFFFFU) || (output->pm10_raw == 0xFFFFU))                                                    /* check invalid */
    {
        output->pm1p0_ug_m3 = FP_NAN;                                                                                       /* set nan */
        output->pm2p5_ug_m3 = FP_NAN;                                                                                       /* set nan */
        output->pm4p0_ug_m3 = FP_NAN;                                                                                       /* set nan */
        output->pm10_ug_m3 = FP_NAN;                                                                                        /* set nan */
        output->pm_valid = 0;                                                                                               /* set invalid */
    }
    else
    {
        output->pm1p0_ug_m3 = (float)(output->pm1p0_raw) / 10.0f;                                                           /* convert pm1.0 */
        output->pm2p5_ug_m3 = (float)(output->pm2p5_raw) / 10.0f;                                                           /* convert pm2.5 */
        output->pm4p0_ug_m3 = (float)(output->pm4p0_raw) / 10.0f;                                                           /* convert pm4.0 */
        output->pm10_ug_m3 = (float)(output->pm10_raw) / 10.0f;                                                             /* convert pm10 */
        output->pm_valid = 1;                                                                                               /* set valid */
    }
    if (output->compensated_ambient_humidity_raw == 0x7FFF)                                                                 /* check humidity raw */
    {
        output->compensated_ambient_humidity_percentage = FP_NAN;                                                           /* set nan */
    }
    else                                                                                                                    /* convert */
    {
        output->compensated_ambient_humidity_percentage = (float)(output->compensated_ambient_humidity_raw) / 100.0f;       /* convert humidity */
    }
    if (output->compensated_ambient_temperature_raw == 0x7FFF)                                                              /* check temperature raw */
    {
        output->compensated_ambient_temperature_degree = FP_NAN;                                                            /* set nan */
    }
    else
    {
        output->compensated_ambient_temperature_degree = (float)(output->compensated_ambient_temperature_raw) / 200.0f;     /* convert temperature */
    }
    if (output->voc_raw == 0x7FFF)                                                                                          /* check voc raw */
    {
        output->voc_index = FP_NAN;                                                                                         /* set nan */
    }
    else
    {
        output->voc_index = (float)(output->voc_raw) / 10.0f;                                                               /* convert voc */
    }
    
    return 0;                                                                                                               /* success return 0 */
}

/**
 * @brief      read the result
 * @param[in]  *handle pointer to a sen5x handle structure
 * @param[out] *output pointer to a data structure
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 *             - 4 only sen50 can use this function
 * @note       only sen50 can use this function
 */
uint8_t sen50_read(sen5x_handle_t *handle, sen50_data_t *output)
{
    uint8_t res;
    uint8_t i;
    uint8_t check[3];
    uint8_t buf[24];
    
    if ((handle == NULL) || (output == NULL))                                                                               /* check handle */
    {
        return 2;                                                                                                           /* return error */
    }
    if (handle->inited != 1)                                                                                                /* check handle initialization */
    {
        return 3;                                                                                                           /* return error */
    }
    if (handle->type != (uint8_t)(SEN50))                                                                                   /* check type */
    {
        handle->debug_print("sen5x: only sen50 can use this function.\n");                                                  /* only sen50 can use this function */
       
        return 4;                                                                                                           /* return error */
    }
    
    memset(check, 0, sizeof(uint8_t) * 3);                                                                                  /* clear the buffer */
    res = a_sen5x_iic_read(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_DATA_READY_FLAG, (uint8_t *)check, 3, 20);         /* read data ready flag command */
    if (res != 0)                                                                                                           /* check result */
    {
        handle->debug_print("sen5x: read data ready flag failed.\n");                                                       /* read data ready flag failed */
       
        return 1;                                                                                                           /* return error */
    }
    if (check[2] != a_sen5x_crc(handle, (uint8_t *)check, 2))                                                               /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                                                  /* crc check failed */
       
        return 1;                                                                                                           /* return error */
    }
    if ((check[1] & 0x01) == 0)                                                                                             /* check flag */
    {
        handle->debug_print("sen5x: data not ready.\n");                                                                    /* data not ready */
       
        return 1;                                                                                                           /* return error */
    }
    
    memset(buf, 0, sizeof(uint8_t) * 24);                                                                                   /* clear the buffer */
    res = a_sen5x_iic_read(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_MEASURED_VALUES, (uint8_t *)buf, 24, 20);          /* read measured values command */
    if (res != 0)                                                                                                           /* check result */
    {
        handle->debug_print("sen5x: read measured values failed.\n");                                                       /* read measured values failed */
       
        return 1;                                                                                                           /* return error */
    }
    for (i = 0; i < 8; i++)                                                                                                 /* check crc */
    {
        if (buf[i * 3 + 2] != a_sen5x_crc(handle, (uint8_t *)&buf[i * 3], 2))                                               /* check crc */
        {
            handle->debug_print("sen5x: crc is error.\n");                                                                  /* crc is error */
           
            return 1;                                                                                                       /* return error */
        }
    }
    output->pm1p0_raw = (uint16_t)(((uint16_t)(buf[0]) << 8) | ((uint16_t)(buf[1]) << 0));                                  /* copy pm1.0 */
    output->pm2p5_raw = (uint16_t)(((uint16_t)(buf[3]) << 8) | ((uint16_t)(buf[4]) << 0));                                  /* copy pm2.5 */
    output->pm4p0_raw = (uint16_t)(((uint16_t)(buf[6]) << 8) | ((uint16_t)(buf[7]) << 0));                                  /* copy pm4.0 */
    output->pm10_raw = (uint16_t)(((uint16_t)(buf[9]) << 8) | ((uint16_t)(buf[10]) << 0));                                  /* copy pm10 */
    if ((output->pm1p0_raw == 0xFFFFU) || (output->pm2p5_raw == 0xFFFFU) || 
        (output->pm4p0_raw == 0xFFFFU) || (output->pm10_raw == 0xFFFFU))                                                    /* check invalid */
    {
        output->pm1p0_ug_m3 = FP_NAN;                                                                                       /* set nan */
        output->pm2p5_ug_m3 = FP_NAN;                                                                                       /* set nan */
        output->pm4p0_ug_m3 = FP_NAN;                                                                                       /* set nan */
        output->pm10_ug_m3 = FP_NAN;                                                                                        /* set nan */
        output->pm_valid = 0;                                                                                               /* set invalid */
    }
    else
    {
        output->pm1p0_ug_m3 = (float)(output->pm1p0_raw) / 10.0f;                                                           /* convert pm1.0 */
        output->pm2p5_ug_m3 = (float)(output->pm2p5_raw) / 10.0f;                                                           /* convert pm2.5 */
        output->pm4p0_ug_m3 = (float)(output->pm4p0_raw) / 10.0f;                                                           /* convert pm4.0 */
        output->pm10_ug_m3 = (float)(output->pm10_raw) / 10.0f;                                                             /* convert pm10 */
        output->pm_valid = 1;                                                                                               /* set valid */
    }
    
    return 0;                                                                                                               /* success return 0 */
}

/**
 * @brief      read the pm value
 * @param[in]  *handle pointer to a sen5x handle structure
 * @param[out] *pm pointer to a sen5x pm structure
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t sen5x_read_pm_value(sen5x_handle_t *handle, sen5x_pm_t *pm)
{
    uint8_t res, i;
    uint8_t check[3];
    uint8_t buf[30];
    
    if ((handle == NULL) || (pm == NULL))                                                                                   /* check handle */
    {
        return 2;                                                                                                           /* return error */
    }
    if (handle->inited != 1)                                                                                                /* check handle initialization */
    {
        return 3;                                                                                                           /* return error */
    }
    
    memset(check, 0, sizeof(uint8_t) * 3);                                                                                  /* clear the buffer */
    res = a_sen5x_iic_read(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_DATA_READY_FLAG, (uint8_t *)check, 3, 20);         /* read data ready flag command */
    if (res != 0)                                                                                                           /* check result */
    {
        handle->debug_print("sen5x: read data ready flag failed.\n");                                                       /* read data ready flag failed */
       
        return 1;                                                                                                           /* return error */
    }
    if (check[2] != a_sen5x_crc(handle, (uint8_t *)check, 2))                                                               /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                                                  /* crc check failed */
       
        return 1;                                                                                                           /* return error */
    }
    if ((check[1] & 0x01) == 0)                                                                                             /* check flag */
    {
        handle->debug_print("sen5x: data not ready.\n");                                                                    /* data not ready */
       
        return 1;                                                                                                           /* return error */
    }
    
    memset(buf, 0, sizeof(uint8_t) * 30);                                                                                   /* clear the buffer */
    res = a_sen5x_iic_read(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_MEASURED_PM_VALUES, (uint8_t *)buf, 30, 20);       /* read measured values command */
    if (res != 0)                                                                                                           /* check result */
    {
        handle->debug_print("sen5x: read pm values failed.\n");                                                             /* read pm values failed */
       
        return 1;                                                                                                           /* return error */
    }
    for (i = 0; i < 10; i++)                                                                                                /* check crc */
    {
        if (buf[i * 3 + 2] != a_sen5x_crc(handle, (uint8_t *)&buf[i * 3], 2))                                               /* check crc */
        {
            handle->debug_print("sen5x: crc is error.\n");                                                                  /* crc is error */
           
            return 1;                                                                                                       /* return error */
        }
    }
    pm->mass_concentration_pm1p0_raw = (uint16_t)(((uint16_t)(buf[0]) << 8) | ((uint16_t)(buf[1]) << 0));                   /* copy pm1.0 raw */
    pm->mass_concentration_pm2p5_raw = (uint16_t)(((uint16_t)(buf[3]) << 8) | ((uint16_t)(buf[4]) << 0));                   /* copy pm2.5 raw */
    pm->mass_concentration_pm4p0_raw = (uint16_t)(((uint16_t)(buf[6]) << 8) | ((uint16_t)(buf[7]) << 0));                   /* copy pm4.0 raw */
    pm->mass_concentration_pm10_raw = (uint16_t)(((uint16_t)(buf[9]) << 8) | ((uint16_t)(buf[10]) << 0));                   /* copy pm10 raw */
    pm->number_concentration_pm0p5_raw = (uint16_t)(((uint16_t)(buf[12]) << 8) | ((uint16_t)(buf[13]) << 0));               /* copy pm0.5 raw */
    pm->number_concentration_pm1p0_raw = (uint16_t)(((uint16_t)(buf[15]) << 8) | ((uint16_t)(buf[16]) << 0));               /* copy pm1.0 raw */
    pm->number_concentration_pm2p5_raw = (uint16_t)(((uint16_t)(buf[18]) << 8) | ((uint16_t)(buf[19]) << 0));               /* copy pm2.5 raw */
    pm->number_concentration_pm4p0_raw = (uint16_t)(((uint16_t)(buf[21]) << 8) | ((uint16_t)(buf[22]) << 0));               /* copy pm4.0 raw */
    pm->number_concentration_pm10_raw = (uint16_t)(((uint16_t)(buf[24]) << 8) | ((uint16_t)(buf[25]) << 0));                /* copy pm10 raw */
    pm->typical_particle_raw = (uint16_t)(((uint16_t)(buf[27]) << 8) | ((uint16_t)(buf[28]) << 0));                         /* copy typical particle raw */
    if ((pm->mass_concentration_pm1p0_raw == 0xFFFFU) || (pm->mass_concentration_pm2p5_raw == 0xFFFFU) ||
        (pm->mass_concentration_pm4p0_raw == 0xFFFFU) || (pm->mass_concentration_pm10_raw == 0xFFFFU) ||
        (pm->number_concentration_pm0p5_raw == 0xFFFFU) || (pm->number_concentration_pm1p0_raw == 0xFFFFU) ||
        (pm->number_concentration_pm2p5_raw == 0xFFFFU) || (pm->number_concentration_pm4p0_raw == 0xFFFFU) ||
        (pm->number_concentration_pm10_raw == 0xFFFFU) || (pm->typical_particle_raw == 0xFFFFU)
       )
    {
        pm->pm1p0_ug_m3 = FP_NAN;                                                                                           /* set nan */
        pm->pm2p5_ug_m3 = FP_NAN;                                                                                           /* set nan */
        pm->pm4p0_ug_m3 = FP_NAN;                                                                                           /* set nan */
        pm->pm10_ug_m3 = FP_NAN;                                                                                            /* set nan */
        pm->pm0p5_cm3 = FP_NAN;                                                                                             /* set nan */
        pm->pm1p0_cm3 = FP_NAN;                                                                                             /* set nan */
        pm->pm2p5_cm3 = FP_NAN;                                                                                             /* set nan */
        pm->pm4p0_cm3 = FP_NAN;                                                                                             /* set nan */
        pm->pm10_cm3 = FP_NAN;                                                                                              /* set nan */
        pm->typical_particle_um = FP_NAN;                                                                                   /* set nan */
        pm->pm_valid = 0;                                                                                                   /* set invalid */
    }
    else
    {
        pm->pm1p0_ug_m3 = (float)(pm->mass_concentration_pm1p0_raw) / 10.0f;                                                /* convert data */
        pm->pm2p5_ug_m3 = (float)(pm->mass_concentration_pm2p5_raw) / 10.0f;                                                /* convert data */
        pm->pm4p0_ug_m3 = (float)(pm->mass_concentration_pm4p0_raw) / 10.0f;                                                /* convert data */
        pm->pm10_ug_m3 = (float)(pm->mass_concentration_pm10_raw) / 10.0f;                                                  /* convert data */
        pm->pm0p5_cm3 = (float)(pm->number_concentration_pm0p5_raw) / 10.0f;                                                /* convert data */
        pm->pm1p0_cm3 = (float)(pm->number_concentration_pm1p0_raw) / 10.0f;                                                /* convert data */
        pm->pm2p5_cm3 = (float)(pm->number_concentration_pm2p5_raw) / 10.0f;                                                /* convert data */
        pm->pm4p0_cm3 = (float)(pm->number_concentration_pm4p0_raw) / 10.0f;                                                /* convert data */
        pm->pm10_cm3 = (float)(pm->number_concentration_pm10_raw) / 10.0f;                                                  /* convert data */
        pm->typical_particle_um = (float)(pm->typical_particle_raw) / 1000.0f;                                              /* convert data */
        pm->pm_valid = 1;                                                                                                   /* set valid */
    }
    
    return 0;                                                                                                               /* success return 0 */
}

/**
 * @brief      read raw value
 * @param[in]  *handle pointer to a sen5x handle structure
 * @param[out] *raw pointer to a sen5x raw structure
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 *             - 4 sen50 doesn't support this function
 * @note       sen50 doesn't support this function
 */
uint8_t sen5x_read_raw_value(sen5x_handle_t *handle, sen5x_raw_t *raw)
{
    uint8_t res, i;
    uint8_t check[3];
    uint8_t buf[12];
    
    if ((handle == NULL) || (raw == NULL))                                                                                  /* check handle */
    {
        return 2;                                                                                                           /* return error */
    }
    if (handle->inited != 1)                                                                                                /* check handle initialization */
    {
        return 3;                                                                                                           /* return error */
    }
    if (handle->type == (uint8_t)(SEN50))                                                                                   /* check result */
    {
        handle->debug_print("sen5x: sen50 doesn't support this function.\n");                                               /* sen50 doesn't support this function */
       
        return 4;                                                                                                           /* return error */
    }
    
    memset(check, 0, sizeof(uint8_t) * 3);                                                                                  /* clear the buffer */
    res = a_sen5x_iic_read(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_DATA_READY_FLAG, (uint8_t *)check, 3, 20);         /* read data ready flag command */
    if (res != 0)                                                                                                           /* check result */
    {
        handle->debug_print("sen5x: read data ready flag failed.\n");                                                       /* read data ready flag failed */
       
        return 1;                                                                                                           /* return error */
    }
    if (check[2] != a_sen5x_crc(handle, (uint8_t *)check, 2))                                                               /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                                                  /* crc check failed */
       
        return 1;                                                                                                           /* return error */
    }
    if ((check[1] & 0x01) == 0)                                                                                             /* check flag */
    {
        handle->debug_print("sen5x: data not ready.\n");                                                                    /* data not ready */
       
        return 1;                                                                                                           /* return error */
    }
    
    memset(buf, 0, sizeof(uint8_t) * 12);                                                                                   /* clear the buffer */
    res = a_sen5x_iic_read(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_MEASURED_RAW_VALUES, (uint8_t *)buf, 12, 20);      /* read raw values command */
    if (res != 0)                                                                                                           /* check result */
    {
        handle->debug_print("sen5x: read raw values failed.\n");                                                            /* read raw values failed */
       
        return 1;                                                                                                           /* return error */
    }
    for (i = 0; i < 4; i++)                                                                                                 /* check crc */
    {
        if (buf[i * 3 + 2] != a_sen5x_crc(handle, (uint8_t *)&buf[i * 3], 2))                                               /* check crc */
        {
            handle->debug_print("sen5x: crc is error.\n");                                                                  /* crc is error */
           
            return 1;                                                                                                       /* return error */
        }
    }
    raw->humidity_raw = (int16_t)(((uint16_t)(buf[0]) << 8) | ((uint16_t)(buf[1]) << 0));                                   /* set humidity raw */
    raw->temperature_raw = (int16_t)(((uint16_t)(buf[3]) << 8) | ((uint16_t)(buf[4]) << 0));                                /* set temperature raw */
    raw->voc_raw = (uint16_t)(((uint16_t)(buf[6]) << 8) | ((uint16_t)(buf[7]) << 0));                                       /* set voc raw */
    raw->nox_raw = (uint16_t)(((uint16_t)(buf[9]) << 8) | ((uint16_t)(buf[10]) << 0));                                      /* set nox raw */
    if (raw->humidity_raw != 0x7FFF)                                                                                        /* check valid */
    {
        raw->humidity_percentage = (float)(raw->humidity_raw) / 100.0f;                                                     /* set humidity percentage */
    }
    else
    {
        raw->humidity_percentage = FP_NAN;                                                                                  /* set nan */
    }
    if (raw->temperature_raw != 0x7FFF)                                                                                     /* check valid */
    {
        raw->temperature_degree = (float)(raw->temperature_raw) / 200.0f;                                                   /* set temperature raw */
    }
    else
    {
        raw->temperature_degree = FP_NAN;                                                                                   /* set nan */
    }
    if (raw->voc_raw != 0xFFFFU)                                                                                            /* check valid */
    {
        raw->voc = (float)(raw->voc_raw);                                                                                   /* set voc */
    }
    else
    {
        raw->voc = FP_NAN;                                                                                                  /* set nan */
    }
    if (raw->nox_raw != 0xFFFFU)                                                                                            /* check valid */
    {
        raw->nox = (float)(raw->nox_raw);                                                                                   /* set nox */
    }
    else
    {
        raw->nox = FP_NAN;                                                                                                  /* set nan */
    }
    
    return 0;                                                                                                               /* success return 0 */
}

/**
 * @brief     set temperature compensation
 * @param[in] *handle pointer to a sen5x handle structure
 * @param[in] temperature_offset temperature offset
 * @param[in] normalized_temperature_offset_slope normalized temperature offset slope
 * @param[in] time_constant time constant
 * @return    status code
 *            - 0 success
 *            - 1 set temperature compensation failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t sen5x_set_temperature_compensation(sen5x_handle_t *handle, int16_t temperature_offset,
                                           int16_t normalized_temperature_offset_slope,
                                           uint16_t time_constant)
{
    uint8_t res;
    uint8_t buf[9];
    
    if (handle == NULL)                                                                                          /* check handle */
    {
        return 2;                                                                                                /* return error */
    }
    if (handle->inited != 1)                                                                                     /* check handle initialization */
    {
        return 3;                                                                                                /* return error */
    }

    buf[0] = ((uint16_t)temperature_offset >> 8) & 0xFF;                                                         /* set temperature offset msb */
    buf[1] = ((uint16_t)temperature_offset >> 0) & 0xFF;                                                         /* set temperature offset lsb */
    buf[2] = a_sen5x_crc(handle, (uint8_t *)&buf[0], 2);                                                         /* set crc */
    buf[3] = ((uint16_t)normalized_temperature_offset_slope >> 8) & 0xFF;                                        /* set normalized temperature offset slope msb */
    buf[4] = ((uint16_t)normalized_temperature_offset_slope >> 0) & 0xFF;                                        /* set normalized temperature offset slope lsb */
    buf[5] = a_sen5x_crc(handle, (uint8_t *)&buf[3], 2);                                                         /* set crc */
    buf[6] = ((uint16_t)time_constant >> 8) & 0xFF;                                                              /* set time constant msb */
    buf[7] = ((uint16_t)time_constant >> 0) & 0xFF;                                                              /* set time constant lsb */
    buf[8] = a_sen5x_crc(handle, (uint8_t *)&buf[6], 2);                                                         /* set crc */
    res = a_sen5x_iic_write(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_WRITE_TEMPERATURE_COMPENSATION, 
                           (uint8_t *)buf, 9, 20);                                                               /* set temperature compensation command */
    if (res != 0)                                                                                                /* check result */
    {
        handle->debug_print("sen5x: set temperature compensation failed.\n");                                    /* set temperature compensation failed */
       
        return 1;                                                                                                /* return error */
    }
    
    return 0;                                                                                                    /* success return 0 */
}

/**
 * @brief      get temperature compensation
 * @param[in]  *handle pointer to a sen5x handle structure
 * @param[out] *temperature_offset pointer to a temperature offset buffer
 * @param[out] *normalized_temperature_offset_slope pointer to a normalized temperature offset slope buffer
 * @param[out] *time_constant pointer to a time constant buffer
 * @return     status code
 *             - 0 success
 *             - 1 get temperature compensation failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t sen5x_get_temperature_compensation(sen5x_handle_t *handle, int16_t *temperature_offset,
                                           int16_t *normalized_temperature_offset_slope,
                                           uint16_t *time_constant)
{
    uint8_t res;
    uint8_t buf[9];
    
    if (handle == NULL)                                                                                          /* check handle */
    {
        return 2;                                                                                                /* return error */
    }
    if (handle->inited != 1)                                                                                     /* check handle initialization */
    {
        return 3;                                                                                                /* return error */
    }

    res = a_sen5x_iic_read(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_WRITE_TEMPERATURE_COMPENSATION, 
                          (uint8_t *)buf, 9, 20);                                                                /* get temperature compensation command */
    if (res != 0)                                                                                                /* check result */
    {
        handle->debug_print("sen5x: get temperature compensation failed.\n");                                    /* get temperature compensation failed */
       
        return 1;                                                                                                /* return error */
    }
    if (buf[2] != a_sen5x_crc(handle, (uint8_t *)&buf[0], 2))                                                    /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                                       /* crc check failed */
       
        return 1;                                                                                                /* return error */
    }
    if (buf[5] != a_sen5x_crc(handle, (uint8_t *)&buf[3], 2))                                                    /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                                       /* crc check failed */
       
        return 1;                                                                                                /* return error */
    }
    if (buf[8] != a_sen5x_crc(handle, (uint8_t *)&buf[6], 2))                                                    /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                                       /* crc check failed */
       
        return 1;                                                                                                /* return error */
    }
    *temperature_offset = (int16_t)(((uint16_t)(buf[0]) << 8) | ((uint16_t)(buf[1]) << 0));                      /* set temperature offset */
    *normalized_temperature_offset_slope = (int16_t)(((uint16_t)(buf[3]) << 8) | ((uint16_t)(buf[4]) << 0));     /* set normalized temperature offset slope */
    *time_constant = (uint16_t)(((uint16_t)(buf[6]) << 8) | ((uint16_t)(buf[7]) << 0));                          /* set time constant */
    
    return 0;                                                                                                    /* success return 0 */
}

/**
 * @brief      convert the temperature compensation to the register raw data
 * @param[in]  *handle pointer to a sen5x handle structure
 * @param[in]  temperature_offset_degree temperature offset degree
 * @param[in]  normalized_temperature_offset_slope_factor normalized temperature offset slope factor
 * @param[in]  time_constant_second time constant second
 * @param[out] *temperature_offset_reg pointer to a temperature offset reg buffer
 * @param[out] *normalized_temperature_offset_slope_reg pointer to a normalized temperature offset slope reg buffer
 * @param[out] *time_constant_reg pointer to a time constant reg buffer
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t sen5x_temperature_compensation_convert_to_register(sen5x_handle_t *handle,
                                                           float temperature_offset_degree,
                                                           float normalized_temperature_offset_slope_factor,
                                                           float time_constant_second,
                                                           int16_t *temperature_offset_reg,
                                                           int16_t *normalized_temperature_offset_slope_reg,
                                                           uint16_t *time_constant_reg)
{
    if (handle == NULL)                                                                                                 /* check handle */
    {
        return 2;                                                                                                       /* return error */
    }
    if (handle->inited != 1)                                                                                            /* check handle initialization */
    {
        return 3;                                                                                                       /* return error */
    }
    
    *temperature_offset_reg = (int16_t)(temperature_offset_degree * 200.0f);                                            /* convert real data to register data */
    *normalized_temperature_offset_slope_reg = (int16_t)(normalized_temperature_offset_slope_factor * 10000.0f);        /* convert real data to register data */
    *time_constant_reg = (uint16_t)(time_constant_second);                                                              /* convert real data to register data */
    
    return 0;                                                                                                           /* success return 0 */
}

/**
 * @brief      convert the temperature compensation to the real data
 * @param[in]  *handle pointer to a sen5x handle structure
 * @param[in]  temperature_offset_reg temperature offset register raw data
 * @param[in]  normalized_temperature_offset_slope_reg normalized temperature offset slope register raw data
 * @param[in]  time_constant_reg time constant register raw data
 * @param[out] *temperature_offset_degree pointer to a temperature offset buffer
 * @param[out] *normalized_temperature_offset_slope_factor pointer to a normalized temperature offset slope buffer
 * @param[out] *time_constant_second pointer to a time constant buffer
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t sen5x_temperature_compensation_convert_to_data(sen5x_handle_t *handle,
                                                       int16_t temperature_offset_reg,
                                                       int16_t normalized_temperature_offset_slope_reg,
                                                       uint16_t time_constant_reg,
                                                       float *temperature_offset_degree,
                                                       float *normalized_temperature_offset_slope_factor,
                                                       float *time_constant_second)
{
    if (handle == NULL)                                                                                                 /* check handle */
    {
        return 2;                                                                                                       /* return error */
    }
    if (handle->inited != 1)                                                                                            /* check handle initialization */
    {
        return 3;                                                                                                       /* return error */
    }
    
    *temperature_offset_degree = (float)(temperature_offset_reg) / 200.0f;                                              /* convert raw data to real data */
    *normalized_temperature_offset_slope_factor = (float)(normalized_temperature_offset_slope_reg) / 10000.0f;          /* convert raw data to real data */
    *time_constant_second = (float)(time_constant_reg);                                                                 /* convert raw data to real data */
    
    return 0;                                                                                                           /* success return 0 */
}

/**
 * @brief     set warm start
 * @param[in] *handle pointer to a sen5x handle structure
 * @param[in] param warm start param
 * @return    status code
 *            - 0 success
 *            - 1 set warm start failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t sen5x_set_warm_start(sen5x_handle_t *handle, uint16_t param)
{
    uint8_t res;
    uint8_t buf[3];
    
    if (handle == NULL)                                                                                   /* check handle */
    {
        return 2;                                                                                         /* return error */
    }
    if (handle->inited != 1)                                                                              /* check handle initialization */
    {
        return 3;                                                                                         /* return error */
    }

    buf[0] = ((uint16_t)param >> 8) & 0xFF;                                                               /* set param msb */
    buf[1] = ((uint16_t)param >> 0) & 0xFF;                                                               /* set param lsb */
    buf[2] = a_sen5x_crc(handle, (uint8_t *)&buf[0], 2);                                                  /* set crc */
    res = a_sen5x_iic_write(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_WRITE_WARM_START_PARAMS, 
                           (uint8_t *)buf, 3, 20);                                                        /* set warm start command */
    if (res != 0)                                                                                         /* check result */
    {
        handle->debug_print("sen5x: set warm start failed.\n");                                           /* set warm start failed */
       
        return 1;                                                                                         /* return error */
    }
    
    return 0;                                                                                             /* success return 0 */
}

/**
 * @brief      get warm start
 * @param[in]  *handle pointer to a sen5x handle structure
 * @param[out] *param pointer to a warm start param buffer
 * @return     status code
 *             - 0 success
 *             - 1 get warm start failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t sen5x_get_warm_start(sen5x_handle_t *handle, uint16_t *param)
{
    uint8_t res;
    uint8_t buf[3];
    
    if (handle == NULL)                                                                                  /* check handle */
    {
        return 2;                                                                                        /* return error */
    }
    if (handle->inited != 1)                                                                             /* check handle initialization */
    {
        return 3;                                                                                        /* return error */
    }

    res = a_sen5x_iic_read(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_WRITE_WARM_START_PARAMS, 
                          (uint8_t *)buf, 3, 20);                                                        /* get warm start command */
    if (res != 0)                                                                                        /* check result */
    {
        handle->debug_print("sen5x: get warm start failed.\n");                                          /* get warm start failed */
       
        return 1;                                                                                        /* return error */
    }
    if (buf[2] != a_sen5x_crc(handle, (uint8_t *)&buf[0], 2))                                            /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                               /* crc check failed */
       
        return 1;                                                                                        /* return error */
    }
    *param = (uint16_t)(((uint16_t)(buf[0]) << 8) | ((uint16_t)(buf[1]) << 0));                          /* set param */
    
    return 0;                                                                                            /* success return 0 */
}

/**
 * @brief     set voc algorithm tuning
 * @param[in] *handle pointer to a sen5x handle structure
 * @param[in] index_offset index offset
 * @param[in] learning_time_offset_hour learning time offset hour
 * @param[in] learning_time_gain_hour learning time gain hour
 * @param[in] gating_max_duration_minute gating max duration minute
 * @param[in] std_initial std initial
 * @param[in] gain_factor gain factor
 * @return    status code
 *            - 0 success
 *            - 1 set voc algorithm tuning failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 param is invalid
 * @note      1 <= index_offset <= 250
 *            1 <= learning_time_offset_hour <= 1000
 *            1 <= learning_time_gain_hour <= 1000
 *            0 <= gating_max_duration_minute <= 3000
 *            10 <= std_initial <= 5000
 *            1 <= gain_factor <= 1000
 */
uint8_t sen5x_set_voc_algorithm_tuning(sen5x_handle_t *handle, int16_t index_offset,
                                       int16_t learning_time_offset_hour,
                                       int16_t learning_time_gain_hour,
                                       int16_t gating_max_duration_minute,
                                       int16_t std_initial,
                                       int16_t gain_factor)
{
    uint8_t res;
    uint8_t buf[18];
    
    if (handle == NULL)                                                                                       /* check handle */
    {
        return 2;                                                                                             /* return error */
    }
    if (handle->inited != 1)                                                                                  /* check handle initialization */
    {
        return 3;                                                                                             /* return error */
    }
    if ((index_offset > 250) || (index_offset < 1))                                                           /* check index offset */
    {
        handle->debug_print("sen5x: index_offset > 250 or index_offset < 1.\n");                              /* index_offset > 250 or index_offset < 1 */
       
        return 4;                                                                                             /* return error */
    }
    if ((learning_time_offset_hour > 1000) || (learning_time_offset_hour < 1))                                /* check learning time offset hour */
    {
        handle->debug_print("sen5x: learning time offset hour > 1000 or learning time offset hour < 1.\n");   /* learning time offset hour > 1000 or learning time offset hour < 1 */
       
        return 4;                                                                                             /* return error */
    }
    if ((learning_time_gain_hour > 1000) || (learning_time_gain_hour < 1))                                    /* check learning time gain hour */
    {
        handle->debug_print("sen5x: learning time gain hour > 1000 or learning time gain hour < 1.\n");       /* learning time gain hour > 1000 or learning time gain hour < 1 */
       
        return 4;                                                                                             /* return error */
    }
    if ((gating_max_duration_minute > 3000) || (gating_max_duration_minute < 0))                              /* check gating max duration minute */
    {
        handle->debug_print("sen5x: gating max duration minute > 3000 or gating max duration minute < 0.\n"); /* gating max duration minute > 3000 or gating max duration minute < 0 */
       
        return 4;                                                                                             /* return error */
    }
    if ((std_initial > 5000) || (std_initial < 10))                                                           /* check std initial */
    {
        handle->debug_print("sen5x: std initial > 5000 or std initial < 10.\n");                              /* std initial > 5000 or std initial < 10 */
       
        return 4;                                                                                             /* return error */
    }
    if ((gain_factor > 1000) || (gain_factor < 1))                                                            /* check index offset */
    {
        handle->debug_print("sen5x: gain factor > 1000 or gain factor < 1.\n");                               /* gain factor > 1000 or gain factor < 1 */
       
        return 4;                                                                                             /* return error */
    }
    
    buf[0] = ((uint16_t)index_offset >> 8) & 0xFF;                                                            /* set index offset msb */
    buf[1] = ((uint16_t)index_offset >> 0) & 0xFF;                                                            /* set index offset lsb */
    buf[2] = a_sen5x_crc(handle, (uint8_t *)&buf[0], 2);                                                      /* set crc */
    buf[3] = ((uint16_t)learning_time_offset_hour >> 8) & 0xFF;                                               /* set learning time offset hour msb */
    buf[4] = ((uint16_t)learning_time_offset_hour >> 0) & 0xFF;                                               /* set learning time offset hour lsb */
    buf[5] = a_sen5x_crc(handle, (uint8_t *)&buf[3], 2);                                                      /* set crc */
    buf[6] = ((uint16_t)learning_time_gain_hour >> 8) & 0xFF;                                                 /* set learning time gain hour msb */
    buf[7] = ((uint16_t)learning_time_gain_hour >> 0) & 0xFF;                                                 /* set learning time gain hour lsb */
    buf[8] = a_sen5x_crc(handle, (uint8_t *)&buf[6], 2);                                                      /* set crc */
    buf[9] = ((uint16_t)gating_max_duration_minute >> 8) & 0xFF;                                              /* set gating max duration minute msb */
    buf[10] = ((uint16_t)gating_max_duration_minute >> 0) & 0xFF;                                             /* set gating max duration minute lsb */
    buf[11] = a_sen5x_crc(handle, (uint8_t *)&buf[9], 2);                                                     /* set crc */
    buf[12] = ((uint16_t)std_initial >> 8) & 0xFF;                                                            /* set std initial msb */
    buf[13] = ((uint16_t)std_initial >> 0) & 0xFF;                                                            /* set std initial lsb */
    buf[14] = a_sen5x_crc(handle, (uint8_t *)&buf[12], 2);                                                    /* set crc */
    buf[15] = ((uint16_t)gain_factor >> 8) & 0xFF;                                                            /* set gain factor msb */
    buf[16] = ((uint16_t)gain_factor >> 0) & 0xFF;                                                            /* set gain factor lsb */
    buf[17] = a_sen5x_crc(handle, (uint8_t *)&buf[15], 2);                                                    /* set crc */
    res = a_sen5x_iic_write(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_WRITE_VOC_ALG_TUNING_PARAMS, 
                           (uint8_t *)buf, 18, 20);                                                           /* set voc algorithm tuning command */
    if (res != 0)                                                                                             /* check result */
    {
        handle->debug_print("sen5x: set voc algorithm tuning failed.\n");                                     /* set voc algorithm tuning failed */
       
        return 1;                                                                                             /* return error */
    }
    
    return 0;                                                                                                 /* success return 0 */
}

/**
 * @brief      get voc algorithm tuning
 * @param[in]  *handle pointer to a sen5x handle structure
 * @param[out] *index_offset pointer to an index offset buffer
 * @param[out] *learning_time_offset_hour pointer to a learning time offset hour buffer
 * @param[out] *learning_time_gain_hour pointer to a learning time gain hour buffer
 * @param[out] *gating_max_duration_minute pointer to a gating max duration minute buffer
 * @param[out] *std_initial pointer to a std initial buffer
 * @param[out] *gain_factor pointer to a gain factor buffer
 * @return     status code
 *             - 0 success
 *             - 1 get voc algorithm tuning failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t sen5x_get_voc_algorithm_tuning(sen5x_handle_t *handle, int16_t *index_offset,
                                       int16_t *learning_time_offset_hour,
                                       int16_t *learning_time_gain_hour,
                                       int16_t *gating_max_duration_minute,
                                       int16_t *std_initial,
                                       int16_t *gain_factor)
{
    uint8_t res;
    uint8_t buf[18];
    
    if (handle == NULL)                                                                                          /* check handle */
    {
        return 2;                                                                                                /* return error */
    }
    if (handle->inited != 1)                                                                                     /* check handle initialization */
    {
        return 3;                                                                                                /* return error */
    }

    res = a_sen5x_iic_read(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_WRITE_VOC_ALG_TUNING_PARAMS, 
                          (uint8_t *)buf, 18, 20);                                                               /* get voc algorithm tuning command */
    if (res != 0)                                                                                                /* check result */
    {
        handle->debug_print("sen5x: get voc algorithm tuning failed.\n");                                        /* get voc algorithm tuning failed */
       
        return 1;                                                                                                /* return error */
    }
    if (buf[2] != a_sen5x_crc(handle, (uint8_t *)&buf[0], 2))                                                    /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                                       /* crc check failed */
       
        return 1;                                                                                                /* return error */
    }
    if (buf[5] != a_sen5x_crc(handle, (uint8_t *)&buf[3], 2))                                                    /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                                       /* crc check failed */
       
        return 1;                                                                                                /* return error */
    }
    if (buf[8] != a_sen5x_crc(handle, (uint8_t *)&buf[6], 2))                                                    /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                                       /* crc check failed */
       
        return 1;                                                                                                /* return error */
    }
    if (buf[11] != a_sen5x_crc(handle, (uint8_t *)&buf[9], 2))                                                   /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                                       /* crc check failed */
       
        return 1;                                                                                                /* return error */
    }
    if (buf[14] != a_sen5x_crc(handle, (uint8_t *)&buf[12], 2))                                                  /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                                       /* crc check failed */
       
        return 1;                                                                                                /* return error */
    }
    if (buf[17] != a_sen5x_crc(handle, (uint8_t *)&buf[15], 2))                                                  /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                                       /* crc check failed */
       
        return 1;                                                                                                /* return error */
    }
    *index_offset = (int16_t)(((uint16_t)(buf[0]) << 8) | ((uint16_t)(buf[1]) << 0));                            /* set index offset */
    *learning_time_offset_hour = (int16_t)(((uint16_t)(buf[3]) << 8) | ((uint16_t)(buf[4]) << 0));               /* set learning time offset hour */
    *learning_time_gain_hour = (int16_t)(((uint16_t)(buf[6]) << 8) | ((uint16_t)(buf[7]) << 0));                 /* set learning time gain hour */
    *gating_max_duration_minute = (int16_t)(((uint16_t)(buf[9]) << 8) | ((uint16_t)(buf[10]) << 0));             /* set gating max duration minute */
    *std_initial = (int16_t)(((uint16_t)(buf[12]) << 8) | ((uint16_t)(buf[13]) << 0));                           /* set std initial */
    *gain_factor = (int16_t)(((uint16_t)(buf[15]) << 8) | ((uint16_t)(buf[16]) << 0));                           /* set gain factor */
    
    return 0;                                                                                                    /* success return 0 */
}

/**
 * @brief     set nox algorithm tuning
 * @param[in] *handle pointer to a sen5x handle structure
 * @param[in] index_offset index offset
 * @param[in] learning_time_offset_hour learning time offset hour
 * @param[in] learning_time_gain_hour learning time gain hour
 * @param[in] gating_max_duration_minute gating max duration minute
 * @param[in] std_initial std initial
 * @param[in] gain_factor gain factor
 * @return    status code
 *            - 0 success
 *            - 1 set nox algorithm tuning failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 param is invalid
 * @note      1 <= index_offset <= 250
 *            1 <= learning_time_offset_hour <= 1000
 *            learning_time_gain_hour = 12
 *            0 <= gating_max_duration_minute <= 3000
 *            std_initial = 50
 *            1 <= gain_factor <= 1000
 */
uint8_t sen5x_set_nox_algorithm_tuning(sen5x_handle_t *handle, int16_t index_offset,
                                       int16_t learning_time_offset_hour,
                                       int16_t learning_time_gain_hour,
                                       int16_t gating_max_duration_minute,
                                       int16_t std_initial,
                                       int16_t gain_factor)
{
    uint8_t res;
    uint8_t buf[18];
    
    if (handle == NULL)                                                                                       /* check handle */
    {
        return 2;                                                                                             /* return error */
    }
    if (handle->inited != 1)                                                                                  /* check handle initialization */
    {
        return 3;                                                                                             /* return error */
    }
    if ((index_offset > 250) || (index_offset < 1))                                                           /* check index offset */
    {
        handle->debug_print("sen5x: index_offset > 250 or index_offset < 1.\n");                              /* index_offset > 250 or index_offset < 1 */
       
        return 4;                                                                                             /* return error */
    }
    if ((learning_time_offset_hour > 1000) || (learning_time_offset_hour < 1))                                /* check learning time offset hour */
    {
        handle->debug_print("sen5x: learning time offset hour > 1000 or learning time offset hour < 1.\n");   /* learning time offset hour > 1000 or learning time offset hour < 1 */
       
        return 4;                                                                                             /* return error */
    }
    if (learning_time_gain_hour != 12)                                                                        /* check learning time gain hour */
    {
        handle->debug_print("sen5x: learning time gain hour must be 12.\n");                                  /* learning time gain hour must be 12 */
       
        return 4;                                                                                             /* return error */
    }
    if ((gating_max_duration_minute > 3000) || (gating_max_duration_minute < 0))                              /* check gating max duration minute */
    {
        handle->debug_print("sen5x: gating max duration minute > 3000 or gating max duration minute < 0.\n"); /* gating max duration minute > 3000 or gating max duration minute < 0 */
       
        return 4;                                                                                             /* return error */
    }
    if (std_initial != 50)                                                                                    /* check std initial */
    {
        handle->debug_print("sen5x: std initial must be 50.\n");                                              /* std initial must be 50 */
       
        return 4;                                                                                             /* return error */
    }
    if ((gain_factor > 1000) || (gain_factor < 1))                                                            /* check index offset */
    {
        handle->debug_print("sen5x: gain factor > 1000 or gain factor < 1.\n");                               /* gain factor > 1000 or gain factor < 1 */
       
        return 4;                                                                                             /* return error */
    }
    
    buf[0] = ((uint16_t)index_offset >> 8) & 0xFF;                                                            /* set index offset msb */
    buf[1] = ((uint16_t)index_offset >> 0) & 0xFF;                                                            /* set index offset lsb */
    buf[2] = a_sen5x_crc(handle, (uint8_t *)&buf[0], 2);                                                      /* set crc */
    buf[3] = ((uint16_t)learning_time_offset_hour >> 8) & 0xFF;                                               /* set learning time offset hour msb */
    buf[4] = ((uint16_t)learning_time_offset_hour >> 0) & 0xFF;                                               /* set learning time offset hour lsb */
    buf[5] = a_sen5x_crc(handle, (uint8_t *)&buf[3], 2);                                                      /* set crc */
    buf[6] = ((uint16_t)learning_time_gain_hour >> 8) & 0xFF;                                                 /* set learning time gain hour msb */
    buf[7] = ((uint16_t)learning_time_gain_hour >> 0) & 0xFF;                                                 /* set learning time gain hour lsb */
    buf[8] = a_sen5x_crc(handle, (uint8_t *)&buf[6], 2);                                                      /* set crc */
    buf[9] = ((uint16_t)gating_max_duration_minute >> 8) & 0xFF;                                              /* set gating max duration minute msb */
    buf[10] = ((uint16_t)gating_max_duration_minute >> 0) & 0xFF;                                             /* set gating max duration minute lsb */
    buf[11] = a_sen5x_crc(handle, (uint8_t *)&buf[9], 2);                                                     /* set crc */
    buf[12] = ((uint16_t)std_initial >> 8) & 0xFF;                                                            /* set std initial msb */
    buf[13] = ((uint16_t)std_initial >> 0) & 0xFF;                                                            /* set std initial lsb */
    buf[14] = a_sen5x_crc(handle, (uint8_t *)&buf[12], 2);                                                    /* set crc */
    buf[15] = ((uint16_t)gain_factor >> 8) & 0xFF;                                                            /* set gain factor msb */
    buf[16] = ((uint16_t)gain_factor >> 0) & 0xFF;                                                            /* set gain factor lsb */
    buf[17] = a_sen5x_crc(handle, (uint8_t *)&buf[15], 2);                                                    /* set crc */
    res = a_sen5x_iic_write(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_WRITE_NOX_ALG_TUNING_PARAMS, 
                           (uint8_t *)buf, 18, 20);                                                           /* set nox algorithm tuning command */
    if (res != 0)                                                                                             /* check result */
    {
        handle->debug_print("sen5x: set nox algorithm tuning failed.\n");                                     /* set nox algorithm tuning failed */
       
        return 1;                                                                                             /* return error */
    }
    
    return 0;                                                                                                 /* success return 0 */
}

/**
 * @brief      get nox algorithm tuning
 * @param[in]  *handle pointer to a sen5x handle structure
 * @param[out] *index_offset pointer to an index offset buffer
 * @param[out] *learning_time_offset_hour pointer to a learning time offset hour buffer
 * @param[out] *learning_time_gain_hour pointer to a learning time gain hour buffer
 * @param[out] *gating_max_duration_minute pointer to a gating max duration minute buffer
 * @param[out] *std_initial pointer to a std initial buffer
 * @param[out] *gain_factor pointer to a gain factor buffer
 * @return     status code
 *             - 0 success
 *             - 1 get nox algorithm tuning failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t sen5x_get_nox_algorithm_tuning(sen5x_handle_t *handle, int16_t *index_offset,
                                       int16_t *learning_time_offset_hour,
                                       int16_t *learning_time_gain_hour,
                                       int16_t *gating_max_duration_minute,
                                       int16_t *std_initial,
                                       int16_t *gain_factor)
{
    uint8_t res;
    uint8_t buf[18];
    
    if (handle == NULL)                                                                                          /* check handle */
    {
        return 2;                                                                                                /* return error */
    }
    if (handle->inited != 1)                                                                                     /* check handle initialization */
    {
        return 3;                                                                                                /* return error */
    }

    res = a_sen5x_iic_read(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_WRITE_NOX_ALG_TUNING_PARAMS, 
                          (uint8_t *)buf, 18, 20);                                                               /* get nox algorithm tuning command */
    if (res != 0)                                                                                                /* check result */
    {
        handle->debug_print("sen5x: get nox algorithm tuning failed.\n");                                        /* get nox algorithm tuning failed */
       
        return 1;                                                                                                /* return error */
    }
    if (buf[2] != a_sen5x_crc(handle, (uint8_t *)&buf[0], 2))                                                    /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                                       /* crc check failed */
       
        return 1;                                                                                                /* return error */
    }
    if (buf[5] != a_sen5x_crc(handle, (uint8_t *)&buf[3], 2))                                                    /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                                       /* crc check failed */
       
        return 1;                                                                                                /* return error */
    }
    if (buf[8] != a_sen5x_crc(handle, (uint8_t *)&buf[6], 2))                                                    /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                                       /* crc check failed */
       
        return 1;                                                                                                /* return error */
    }
    if (buf[11] != a_sen5x_crc(handle, (uint8_t *)&buf[9], 2))                                                   /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                                       /* crc check failed */
       
        return 1;                                                                                                /* return error */
    }
    if (buf[14] != a_sen5x_crc(handle, (uint8_t *)&buf[12], 2))                                                  /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                                       /* crc check failed */
       
        return 1;                                                                                                /* return error */
    }
    if (buf[17] != a_sen5x_crc(handle, (uint8_t *)&buf[15], 2))                                                  /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                                       /* crc check failed */
       
        return 1;                                                                                                /* return error */
    }
    *index_offset = (int16_t)(((uint16_t)(buf[0]) << 8) | ((uint16_t)(buf[1]) << 0));                            /* set index offset */
    *learning_time_offset_hour = (int16_t)(((uint16_t)(buf[3]) << 8) | ((uint16_t)(buf[4]) << 0));               /* set learning time offset hour */
    *learning_time_gain_hour = (int16_t)(((uint16_t)(buf[6]) << 8) | ((uint16_t)(buf[7]) << 0));                 /* set learning time gain hour */
    *gating_max_duration_minute = (int16_t)(((uint16_t)(buf[9]) << 8) | ((uint16_t)(buf[10]) << 0));             /* set gating max duration minute */
    *std_initial = (int16_t)(((uint16_t)(buf[12]) << 8) | ((uint16_t)(buf[13]) << 0));                           /* set std initial */
    *gain_factor = (int16_t)(((uint16_t)(buf[15]) << 8) | ((uint16_t)(buf[16]) << 0));                           /* set gain factor */
    
    return 0;                                                                                                    /* success return 0 */
}

/**
 * @brief     set rht acceleration mode
 * @param[in] *handle pointer to a sen5x handle structure
 * @param[in] mode rht acceleration mode
 * @return    status code
 *            - 0 success
 *            - 1 set rht acceleration mode failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t sen5x_set_rht_acceleration_mode(sen5x_handle_t *handle, sen5x_rht_acceleration_mode_t mode)
{
    uint8_t res;
    uint8_t buf[3];
    
    if (handle == NULL)                                                                              /* check handle */
    {
        return 2;                                                                                    /* return error */
    }
    if (handle->inited != 1)                                                                         /* check handle initialization */
    {
        return 3;                                                                                    /* return error */
    }

    buf[0] = ((uint16_t)mode >> 8) & 0xFF;                                                           /* set rht acceleration mode msb */
    buf[1] = ((uint16_t)mode >> 0) & 0xFF;                                                           /* set rht acceleration mode lsb */
    buf[2] = a_sen5x_crc(handle, (uint8_t *)&buf[0], 2);                                             /* set crc */
    res = a_sen5x_iic_write(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_WRITE_RHT_ACC_MODE, 
                           (uint8_t *)buf, 3, 20);                                                   /* set rht acceleration mode command */
    if (res != 0)                                                                                    /* check result */
    {
        handle->debug_print("sen5x: set rht acceleration mode failed.\n");                           /* set rht acceleration mode failed */
       
        return 1;                                                                                    /* return error */
    }
    
    return 0;                                                                                        /* success return 0 */
}

/**
 * @brief      get rht acceleration mode
 * @param[in]  *handle pointer to a sen5x handle structure
 * @param[out] *mode pointer to a rht acceleration mode buffer
 * @return     status code
 *             - 0 success
 *             - 1 get rht acceleration mode failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t sen5x_get_rht_acceleration_mode(sen5x_handle_t *handle, sen5x_rht_acceleration_mode_t *mode)
{
    uint8_t res;
    uint8_t buf[3];
    
    if (handle == NULL)                                                                                    /* check handle */
    {
        return 2;                                                                                          /* return error */
    }
    if (handle->inited != 1)                                                                               /* check handle initialization */
    {
        return 3;                                                                                          /* return error */
    }

    res = a_sen5x_iic_read(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_WRITE_RHT_ACC_MODE, 
                          (uint8_t *)buf, 3, 20);                                                          /* get rht acceleration mode command */
    if (res != 0)                                                                                          /* check result */
    {
        handle->debug_print("sen5x: get rht acceleration mode failed.\n");                                 /* get rht acceleration mode failed */
       
        return 1;                                                                                          /* return error */
    }
    if (buf[2] != a_sen5x_crc(handle, (uint8_t *)&buf[0], 2))                                              /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                                 /* crc check failed */
       
        return 1;                                                                                          /* return error */
    }
    *mode = (sen5x_rht_acceleration_mode_t)(((uint16_t)(buf[0]) << 8) | ((uint16_t)(buf[1]) << 0));        /* set rht acceleration mode */
    
    return 0;                                                                                              /* success return 0 */
}

/**
 * @brief     set voc algorithm state
 * @param[in] *handle pointer to a sen5x handle structure
 * @param[in] *state pointer to a state buffer
 * @return    status code
 *            - 0 success
 *            - 1 set voc algorithm state failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t sen5x_set_voc_algorithm_state(sen5x_handle_t *handle, uint16_t state[4])
{
    uint8_t res;
    uint8_t buf[12];
    
    if (handle == NULL)                                                                               /* check handle */
    {
        return 2;                                                                                     /* return error */
    }
    if (handle->inited != 1)                                                                          /* check handle initialization */
    {
        return 3;                                                                                     /* return error */
    }

    buf[0] = ((uint16_t)state[0] >> 8) & 0xFF;                                                        /* set state0 msb */
    buf[1] = ((uint16_t)state[0] >> 0) & 0xFF;                                                        /* set state0 lsb */
    buf[2] = a_sen5x_crc(handle, (uint8_t *)&buf[0], 2);                                              /* set crc */
    buf[3] = ((uint16_t)state[1] >> 8) & 0xFF;                                                        /* set state1 msb */
    buf[4] = ((uint16_t)state[1] >> 0) & 0xFF;                                                        /* set state1 lsb */
    buf[5] = a_sen5x_crc(handle, (uint8_t *)&buf[3], 2);                                              /* set crc */
    buf[6] = ((uint16_t)state[2] >> 8) & 0xFF;                                                        /* set state2 msb */
    buf[7] = ((uint16_t)state[2] >> 0) & 0xFF;                                                        /* set state2 lsb */
    buf[8] = a_sen5x_crc(handle, (uint8_t *)&buf[6], 2);                                              /* set crc */
    buf[9] = ((uint16_t)state[3] >> 8) & 0xFF;                                                        /* set state3 msb */
    buf[10] = ((uint16_t)state[3] >> 0) & 0xFF;                                                       /* set state3 lsb */
    buf[11] = a_sen5x_crc(handle, (uint8_t *)&buf[9], 2);                                             /* set crc */
    res = a_sen5x_iic_write(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_WRITE_VOC_ALG_STATE, 
                           (uint8_t *)buf, 12, 20);                                                   /* set voc algorithm state command */
    if (res != 0)                                                                                     /* check result */
    {
        handle->debug_print("sen5x: set voc algorithm state failed.\n");                              /* set voc algorithm state failed */
       
        return 1;                                                                                     /* return error */
    }
    
    return 0;                                                                                         /* success return 0 */
}

/**
 * @brief      get voc algorithm state
 * @param[in]  *handle pointer to a sen5x handle structure
 * @param[out] *state pointer to a state buffer
 * @return     status code
 *             - 0 success
 *             - 1 get voc algorithm state failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t sen5x_get_voc_algorithm_state(sen5x_handle_t *handle, uint16_t state[4])
{
    uint8_t res;
    uint8_t buf[12];
    
    if (handle == NULL)                                                                              /* check handle */
    {
        return 2;                                                                                    /* return error */
    }
    if (handle->inited != 1)                                                                         /* check handle initialization */
    {
        return 3;                                                                                    /* return error */
    }

    res = a_sen5x_iic_read(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_WRITE_VOC_ALG_STATE, 
                          (uint8_t *)buf, 12, 20);                                                   /* get voc algorithm state command */
    if (res != 0)                                                                                    /* check result */
    {
        handle->debug_print("sen5x: get voc algorithm state failed.\n");                             /* get voc algorithm state failed */
       
        return 1;                                                                                    /* return error */
    }
    if (buf[2] != a_sen5x_crc(handle, (uint8_t *)&buf[0], 2))                                        /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                           /* crc check failed */
       
        return 1;                                                                                    /* return error */
    }
    if (buf[5] != a_sen5x_crc(handle, (uint8_t *)&buf[3], 2))                                        /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                           /* crc check failed */
       
        return 1;                                                                                    /* return error */
    }
    if (buf[8] != a_sen5x_crc(handle, (uint8_t *)&buf[6], 2))                                        /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                           /* crc check failed */
       
        return 1;                                                                                    /* return error */
    }
    if (buf[11] != a_sen5x_crc(handle, (uint8_t *)&buf[9], 2))                                       /* check crc */
    {
        handle->debug_print("sen5x: crc check failed.\n");                                           /* crc check failed */
       
        return 1;                                                                                    /* return error */
    }
    state[0] = (uint16_t)(((uint16_t)(buf[0]) << 8) | ((uint16_t)(buf[1]) << 0));                    /* set voc algorithm state */
    state[1] = (uint16_t)(((uint16_t)(buf[3]) << 8) | ((uint16_t)(buf[4]) << 0));                    /* set voc algorithm state */
    state[2] = (uint16_t)(((uint16_t)(buf[6]) << 8) | ((uint16_t)(buf[7]) << 0));                    /* set voc algorithm state */
    state[3] = (uint16_t)(((uint16_t)(buf[9]) << 8) | ((uint16_t)(buf[10]) << 0));                   /* set voc algorithm state */
    
    return 0;                                                                                        /* success return 0 */
}

/**
 * @brief     start the fan cleaning
 * @param[in] *handle pointer to a sen5x handle structure
 * @return    status code
 *            - 0 success
 *            - 1 start fan cleaning failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t sen5x_start_fan_cleaning(sen5x_handle_t *handle)
{
    uint8_t res;
    
    if (handle == NULL)                                                                                     /* check handle */
    {
        return 2;                                                                                           /* return error */
    }
    if (handle->inited != 1)                                                                                /* check handle initialization */
    {
        return 3;                                                                                           /* return error */
    }
    
    res = a_sen5x_iic_write(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_START_FAN_CLEANING, NULL, 0, 20);      /* start the fan cleaning command */
    if (res != 0)                                                                                           /* check result */
    {
        handle->debug_print("sen5x: start the fan cleaning failed.\n");                                     /* start the fan cleaning failed */
       
        return 1;                                                                                           /* return error */
    }
    
    return 0;                                                                                               /* success return 0 */
}

/**
 * @brief     set the auto cleaning interval
 * @param[in] *handle pointer to a sen5x handle structure
 * @param[in] second interval
 * @return    status code
 *            - 0 success
 *            - 1 set auto cleaning interval failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 second is invalid
 * @note      10 <= second <= 604800
 */
uint8_t sen5x_set_auto_cleaning_interval(sen5x_handle_t *handle, uint32_t second)
{
    uint8_t res;
    uint8_t buf[6];
    
    if (handle == NULL)                                                                                      /* check handle */
    {
        return 2;                                                                                            /* return error */
    }
    if (handle->inited != 1)                                                                                 /* check handle initialization */
    {
        return 3;                                                                                            /* return error */
    }
    if ((second < 10) || (second > 604800))                                                                  /* check second */
    {
        handle->debug_print("sen5x: second is invalid.\n");                                                  /* second is invalid */
           
        return 4;                                                                                            /* return error */
    }
    
    buf[0] = (second >> 24) & 0xFF;                                                                          /* set byte 4 */
    buf[1] = (second >> 16) & 0xFF;                                                                          /* set byte 3 */
    buf[2] = a_sen5x_crc(handle, (uint8_t *)&buf[0], 2);                                                     /* set crc */
    buf[3] = (second >> 8) & 0xFF;                                                                           /* set byte 2 */
    buf[4] = (second >> 0) & 0xFF;                                                                           /* set byte 1 */
    buf[5] = a_sen5x_crc(handle, (uint8_t *)&buf[3], 2);                                                     /* set crc */
    res = a_sen5x_iic_write(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_WRITE_AUTO_CLEANING_INTERVAL, 
                           (uint8_t *)buf, 6, 20);                                                           /* set auto cleaning interval command */
    if (res != 0)                                                                                            /* check result */
    {
        handle->debug_print("sen5x: set auto cleaning interval failed.\n");                                  /* set auto cleaning interval failed */
       
        return 1;                                                                                            /* return error */
    }

    return 0;                                                                                                /* success return 0 */
}

/**
 * @brief      get the auto cleaning interval
 * @param[in]  *handle pointer to a sen5x handle structure
 * @param[out] *second pointer to an interval buffer
 * @return     status code
 *             - 0 success
 *             - 1 get auto cleaning interval failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t sen5x_get_auto_cleaning_interval(sen5x_handle_t *handle, uint32_t *second)
{
    uint8_t res;
    uint8_t buf[6];
    
    if (handle == NULL)                                                                                      /* check handle */
    {
        return 2;                                                                                            /* return error */
    }
    if (handle->inited != 1)                                                                                 /* check handle initialization */
    {
        return 3;                                                                                            /* return error */
    }
    
    memset(buf, 0, sizeof(uint8_t) * 6);                                                                     /* clear the buffer */
    res = a_sen5x_iic_read(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_WRITE_AUTO_CLEANING_INTERVAL, 
                          (uint8_t *)buf, 6, 20);                                                            /* get auto cleaning interval command */
    if (res != 0)                                                                                            /* check result */
    {
        handle->debug_print("sen5x: get auto cleaning interval failed.\n");                                  /* get auto cleaning interval failed */
       
        return 1;                                                                                            /* return error */
    }
    if (buf[2] != a_sen5x_crc(handle, (uint8_t *)&buf[0], 2))                                                /* check crc */
    {
        handle->debug_print("sen5x: crc is error.\n");                                                       /* crc is error */
       
        return 1;                                                                                            /* return error */
    }
    if (buf[5] != a_sen5x_crc(handle, (uint8_t *)&buf[3], 2))                                                /* check crc */
    {
        handle->debug_print("sen5x: crc is error.\n");                                                       /* crc is error */
       
        return 1;                                                                                            /* return error */
    }
    *second = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | 
              ((uint32_t)buf[3] << 8) | ((uint32_t)buf[4] << 0);                                             /* get second */

    return 0;                                                                                                /* success return 0 */
}

/**
 * @brief     disable the auto cleaning interval
 * @param[in] *handle pointer to a sen5x handle structure
 * @return    status code
 *            - 0 success
 *            - 1 disable auto cleaning interval failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t sen5x_disable_auto_cleaning_interval(sen5x_handle_t *handle)
{
    uint8_t res;
    uint32_t second;
    uint8_t buf[6];
    
    if (handle == NULL)                                                                                      /* check handle */
    {
        return 2;                                                                                            /* return error */
    }
    if (handle->inited != 1)                                                                                 /* check handle initialization */
    {
        return 3;                                                                                            /* return error */
    }
    
    second = 0;                                                                                              /* disable second */
    buf[0] = (second >> 24) & 0xFF;                                                                          /* set byte 4 */
    buf[1] = (second >> 16) & 0xFF;                                                                          /* set byte 3 */
    buf[2] = a_sen5x_crc(handle, (uint8_t *)&buf[0], 2);                                                     /* set crc */
    buf[3] = (second >> 8) & 0xFF;                                                                           /* set byte 2 */
    buf[4] = (second >> 0) & 0xFF;                                                                           /* set byte 1 */
    buf[5] = a_sen5x_crc(handle, (uint8_t *)&buf[3], 2);                                                     /* set crc */
    res = a_sen5x_iic_write(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_WRITE_AUTO_CLEANING_INTERVAL, 
                           (uint8_t *)buf, 6, 20);                                                           /* set auto cleaning interval command */
    if (res != 0)                                                                                            /* check result */
    {
        handle->debug_print("sen5x: set auto cleaning interval failed.\n");                                  /* set auto cleaning interval failed */
       
        return 1;                                                                                            /* return error */
    }

    return 0;                                                                                                /* success return 0 */
}

/**
 * @brief      get the product name
 * @param[in]  *handle pointer to a sen5x handle structure
 * @param[out] *name pointer to a product name buffer
 * @return     status code
 *             - 0 success
 *             - 1 get product name failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t sen5x_get_product_name(sen5x_handle_t *handle, char name[32])
{
    uint8_t res, i, j;
    uint8_t buf[48];
    
    if (handle == NULL)                                                                                               /* check handle */
    {
        return 2;                                                                                                     /* return error */
    }
    if (handle->inited != 1)                                                                                          /* check handle initialization */
    {
        return 3;                                                                                                     /* return error */
    }
    
    memset(buf, 0, sizeof(uint8_t) * 48);                                                                             /* clear the buffer */
    res = a_sen5x_iic_read(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_PRODUCT_NAME, (uint8_t *)buf, 48, 20);       /* read product name command */
    if (res != 0)                                                                                                     /* check result */
    {
        handle->debug_print("sen5x: read product name failed.\n");                                                    /* read product name failed */
       
        return 1;                                                                                                     /* return error */
    }
    for (i = 0; i < 16; i++)                                                                                          /* check crc */
    {
        if (buf[i * 3 + 2] != a_sen5x_crc(handle, (uint8_t *)&buf[i * 3], 2))                                         /* check crc */
        {
            handle->debug_print("sen5x: crc is error.\n");                                                            /* crc is error */
           
            return 1;                                                                                                 /* return error */
        }
    }
    j = 0;                                                                                                            /* set zero */
    for (i = 0; i < 16; i++)                                                                                          /* copy name */
    {
        name[j] = buf[i * 3 + 0];                                                                                     /* set name */
        j++;                                                                                                          /* j++ */
        name[j] = buf[i * 3 + 1];                                                                                     /* set name */
        j++;                                                                                                          /* j++ */
    }
    
    return 0;                                                                                                         /* success return 0 */
}

/**
 * @brief      get the serial number
 * @param[in]  *handle pointer to a sen5x handle structure
 * @param[out] *sn pointer to a serial number buffer
 * @return     status code
 *             - 0 success
 *             - 1 get serial number failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t sen5x_get_serial_number(sen5x_handle_t *handle, char sn[32])
{
    uint8_t res, i, j;
    uint8_t buf[48];
    
    if (handle == NULL)                                                                                               /* check handle */
    {
        return 2;                                                                                                     /* return error */
    }
    if (handle->inited != 1)                                                                                          /* check handle initialization */
    {
        return 3;                                                                                                     /* return error */
    }
    
    memset(buf, 0, sizeof(uint8_t) * 48);                                                                             /* clear the buffer */
    res = a_sen5x_iic_read(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_SERIAL_NUMBER, (uint8_t *)buf, 48, 20);      /* read serial number command */
    if (res != 0)                                                                                                     /* check result */
    {
        handle->debug_print("sen5x: read serial number failed.\n");                                                   /* read serial number failed */
       
        return 1;                                                                                                     /* return error */
    }
    for (i = 0; i < 16; i++)                                                                                          /* check crc */
    {
        if (buf[i * 3 + 2] != a_sen5x_crc(handle, (uint8_t *)&buf[i * 3], 2))                                         /* check crc */
        {
            handle->debug_print("sen5x: crc is error.\n");                                                            /* crc is error */
           
            return 1;                                                                                                 /* return error */
        }
    }
    j = 0;                                                                                                            /* set zero */
    for (i = 0; i < 16; i++)                                                                                          /* copy serial number */
    {
        sn[j] = buf[i * 3 + 0];                                                                                       /* set serial number */
        j++;                                                                                                          /* j++ */
        sn[j] = buf[i * 3 + 1];                                                                                       /* set serial number */
        j++;                                                                                                          /* j++ */
    }
    
    return 0;                                                                                                         /* success return 0 */
}

/**
 * @brief      get the version
 * @param[in]  *handle pointer to a sen5x handle structure
 * @param[out] *version pointer to a version buffer
 * @return     status code
 *             - 0 success
 *             - 1 get version failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t sen5x_get_version(sen5x_handle_t *handle, uint8_t *version)
{
    uint8_t res;
    uint8_t buf[3];
    
    if (handle == NULL)                                                                                        /* check handle */
    {
        return 2;                                                                                              /* return error */
    }
    if (handle->inited != 1)                                                                                   /* check handle initialization */
    {
        return 3;                                                                                              /* return error */
    }
    
    memset(buf, 0, sizeof(uint8_t) * 3);                                                                       /* clear the buffer */
    res = a_sen5x_iic_read(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_VERSION, (uint8_t *)buf, 3, 20);      /* read version command */
    if (res != 0)                                                                                              /* check result */
    {
        handle->debug_print("sen5x: read version failed.\n");                                                  /* read version failed */
       
        return 1;                                                                                              /* return error */
    }
    if (buf[2] != a_sen5x_crc(handle, (uint8_t *)&buf[0], 2))                                                  /* check crc */
    {
        handle->debug_print("sen5x: crc is error.\n");                                                         /* crc is error */
       
        return 1;                                                                                              /* return error */
    }
    *version = buf[0];                                                                                         /* set version */

    return 0;                                                                                                  /* success return 0 */
}

/**
 * @brief      get the device status
 * @param[in]  *handle pointer to a sen5x handle structure
 * @param[out] *status pointer to a status buffer
 * @return     status code
 *             - 0 success
 *             - 1 get device status failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t sen5x_get_device_status(sen5x_handle_t *handle, uint32_t *status)
{
    uint8_t res;
    uint8_t buf[6];
    
    if (handle == NULL)                                                                            /* check handle */
    {
        return 2;                                                                                  /* return error */
    }
    if (handle->inited != 1)                                                                       /* check handle initialization */
    {
        return 3;                                                                                  /* return error */
    }
    
    memset(buf, 0, sizeof(uint8_t) * 6);                                                           /* clear the buffer */
    res = a_sen5x_iic_read(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_READ_DEVICE_STATUS_REG, 
                          (uint8_t *)buf, 6, 20);                                                  /* get device status command */
    if (res != 0)                                                                                  /* check result */
    {
        handle->debug_print("sen5x: get device status failed.\n");                                 /* get device status failed */
       
        return 1;                                                                                  /* return error */
    }
    if (buf[2] != a_sen5x_crc(handle, (uint8_t *)&buf[0], 2))                                      /* check crc */
    {
        handle->debug_print("sen5x: crc is error.\n");                                             /* crc is error */
       
        return 1;                                                                                  /* return error */
    }
    if (buf[5] != a_sen5x_crc(handle, (uint8_t *)&buf[3], 2))                                      /* check crc */
    {
        handle->debug_print("sen5x: crc is error.\n");                                             /* crc is error */
       
        return 1;                                                                                  /* return error */
    }
    *status = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | 
              ((uint32_t)buf[3] << 8) | ((uint32_t)buf[4] << 0);                                   /* get status */

    return 0;                                                                                      /* success return 0 */
}

/**
 * @brief     clear the device status
 * @param[in] *handle pointer to a sen5x handle structure
 * @return    status code
 *            - 0 success
 *            - 1 clear device status failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t sen5x_clear_device_status(sen5x_handle_t *handle)
{
    uint8_t res;
    
    if (handle == NULL)                                                                                          /* check handle */
    {
        return 2;                                                                                                /* return error */
    }
    if (handle->inited != 1)                                                                                     /* check handle initialization */
    {
        return 3;                                                                                                /* return error */
    }
    
    res = a_sen5x_iic_write(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_CLEAR_DEVICE_STATUS_REG, NULL, 0, 20);      /* clear device status command */
    if (res != 0)                                                                                                /* check result */
    {
        handle->debug_print("sen5x: clear device status failed.\n");                                             /* clear device status failed */
       
        return 1;                                                                                                /* return error */
    }

    return 0;                                                                                                    /* success return 0 */
}

/**
 * @brief     reset the chip
 * @param[in] *handle pointer to a sen5x handle structure
 * @return    status code
 *            - 0 success
 *            - 1 reset failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t sen5x_reset(sen5x_handle_t *handle)
{
    uint8_t res;
    
    if (handle == NULL)                                                                          /* check handle */
    {
        return 2;                                                                                /* return error */
    }
    if (handle->inited != 1)                                                                     /* check handle initialization */
    {
        return 3;                                                                                /* return error */
    }

    res = a_sen5x_iic_write(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_RESET, NULL, 0, 200);       /* reset command */
    if (res != 0)                                                                                /* check result */
    {
        handle->debug_print("sen5x: reset failed.\n");                                           /* reset failed */
       
        return 1;                                                                                /* return error */
    }
    
    return 0;                                                                                    /* success return 0 */
}

/**
 * @brief     set the chip type
 * @param[in] *handle pointer to a sen5x handle structure
 * @param[in] type chip type
 * @return    status code
 *            - 0 success
 *            - 2 handle is NULL
 * @note      none
 */
uint8_t sen5x_set_type(sen5x_handle_t *handle, sen5x_type_t type)
{
    if (handle == NULL)                 /* check handle */
    {
        return 2;                       /* return error */
    }

    handle->type = (uint8_t)type;       /* set type */
    
    return 0;                           /* success return 0 */
}

/**
 * @brief      get the chip type
 * @param[in]  *handle pointer to a sen5x handle structure
 * @param[out] *type pointer to a chip type buffer
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 * @note       none
 */
uint8_t sen5x_get_type(sen5x_handle_t *handle, sen5x_type_t *type)
{
    if (handle == NULL)                         /* check handle */
    {
        return 2;                               /* return error */
    }

    *type = (sen5x_type_t)(handle->type);       /* set type */
    
    return 0;                                   /* success return 0 */
}

/**
 * @brief     initialize the chip
 * @param[in] *handle pointer to a sen5x handle structure
 * @return    status code
 *            - 0 success
 *            - 1 iic initialization failed
 *            - 2 handle is NULL
 *            - 3 linked functions is NULL
 *            - 4 reset failed
 * @note      none
 */
uint8_t sen5x_init(sen5x_handle_t *handle)
{
    uint8_t res;
    
    if (handle == NULL)                                                                          /* check handle */
    {
        return 2;                                                                                /* return error */
    }
    if (handle->debug_print == NULL)                                                             /* check debug_print */
    {
        return 3;                                                                                /* return error */
    }
    if (handle->iic_init == NULL)                                                                /* check iic_init */
    {
        handle->debug_print("sen5x: iic_init is null.\n");                                       /* iic_init is null */
    
        return 3;                                                                                /* return error */
    }
    if (handle->iic_deinit == NULL)                                                              /* check iic_deinit */
    {
        handle->debug_print("sen5x: iic_deinit is null.\n");                                     /* iic_deinit is null */
    
        return 3;                                                                                /* return error */
    }
    if (handle->iic_write_cmd == NULL)                                                           /* check iic_write_cmd */
    {
        handle->debug_print("sen5x: iic_write_cmd is null.\n");                                  /* iic_write_cmd is null */
    
        return 3;                                                                                /* return error */
    }
    if (handle->iic_read_cmd == NULL)                                                            /* check iic_read_cmd */
    {
        handle->debug_print("sen5x: iic_read_cmd is null.\n");                                   /* iic_read_cmd is null */
    
        return 3;                                                                                /* return error */
    }
    if (handle->delay_ms == NULL)                                                                /* check delay_ms */
    {
        handle->debug_print("sen5x: delay_ms is null.\n");                                       /* delay_ms is null */
    
        return 3;                                                                                /* return error */
    }
    
    if (handle->iic_init() != 0)                                                                 /* iic init */
    {
        handle->debug_print("sen5x: iic init failed.\n");                                        /* iic init failed */
        
        return 1;                                                                                /* return error */
    }
    res = a_sen5x_iic_write(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_RESET, NULL, 0, 200);       /* reset command */
    if (res != 0)                                                                                /* check result */
    {
        handle->debug_print("sen5x: reset failed.\n");                                           /* reset failed */
        (void)handle->iic_deinit();                                                              /* iic deinit */
        
        return 4;                                                                                /* return error */
    }
    handle->inited = 1;                                                                          /* flag finish initialization */
  
    return 0;                                                                                    /* success return 0 */
}

/**
 * @brief     close the chip
 * @param[in] *handle pointer to a sen5x handle structure
 * @return    status code
 *            - 0 success
 *            - 1 iic deinit failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 soft reset failed
 * @note      none
 */
uint8_t sen5x_deinit(sen5x_handle_t *handle)
{
    uint8_t res;
    
    if (handle == NULL)                                                                          /* check handle */
    {
        return 2;                                                                                /* return error */
    }
    if (handle->inited != 1)                                                                     /* check handle initialization */
    {
        return 3;                                                                                /* return error */
    }    

    res = a_sen5x_iic_write(handle, SEN5X_ADDRESS, SEN5X_IIC_COMMAND_RESET, NULL, 0, 200);       /* reset command */
    if (res != 0)                                                                                /* check result */
    {
        handle->debug_print("sen5x: reset failed.\n");                                           /* reset failed */
       
        return 4;                                                                                /* return error */
    }
    res = handle->iic_deinit();                                                                  /* iic deinit */
    if (res != 0)                                                                                /* check result */
    {
        handle->debug_print("sen5x: iic deinit failed.\n");                                      /* iic deinit */
       
        return 1;                                                                                /* return error */
    }
    handle->inited = 0;                                                                          /* flag close initialization */
  
    return 0;                                                                                    /* success return 0 */
}

/**
 * @brief     set the chip register
 * @param[in] *handle pointer to a sen5x handle structure
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
uint8_t sen5x_set_reg(sen5x_handle_t *handle, uint16_t reg, uint8_t *buf, uint16_t len)
{
    if (handle == NULL)                                                       /* check handle */
    {
        return 2;                                                             /* return error */
    }
    if (handle->inited != 1)                                                  /* check handle initialization */
    {
        return 3;                                                             /* return error */
    }
    
    return a_sen5x_iic_write(handle, SEN5X_ADDRESS, reg, buf, len, 20);       /* write the data */
}

/**
 * @brief      get the chip register
 * @param[in]  *handle pointer to a sen5x handle structure
 * @param[in]  reg iic register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len data buffer length
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t sen5x_get_reg(sen5x_handle_t *handle, uint16_t reg, uint8_t *buf, uint16_t len)
{
    if (handle == NULL)                                                      /* check handle */
    {
        return 2;                                                            /* return error */
    }
    if (handle->inited != 1)                                                 /* check handle initialization */
    {
        return 3;                                                            /* return error */
    }
    
    return a_sen5x_iic_read(handle, SEN5X_ADDRESS, reg, buf, len, 20);       /* read the data */
}

/**
 * @brief      get chip information
 * @param[out] *info pointer to a sen5x info structure
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 * @note       none
 */
uint8_t sen5x_info(sen5x_info_t *info)
{
    if (info == NULL)                                               /* check handle */
    {
        return 2;                                                   /* return error */
    }
    
    memset(info, 0, sizeof(sen5x_info_t));                          /* initialize sen5x info structure */
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
