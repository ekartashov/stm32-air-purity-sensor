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
 * @file      driver_sen5x.h
 * @brief     driver sen5x header file
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

#ifndef DRIVER_SEN5X_H
#define DRIVER_SEN5X_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @defgroup sen5x_driver sen5x driver function
 * @brief    sen5x driver modules
 * @{
 */

/**
 * @addtogroup sen5x_basic_driver
 * @{
 */

/**
 * @brief sen5x type enumeration definition
 */
typedef enum
{
    SEN50 = 0x00,        /**< sen50 */
    SEN54 = 0x01,        /**< sen54 */
    SEN55 = 0x02,        /**< sen55 */
} sen5x_type_t;

/**
 * @brief sen5x data ready flag enumeration definition
 */
typedef enum
{
    SEN5X_DATA_READY_FLAG_NOT_READY = 0x00,        /**< no new measurements available */
    SEN5X_DATA_READY_FLAG_AVAILABLE = 0x01,        /**< new measurements ready to read */
} sen5x_data_ready_flag_t;

/**
 * @brief sen5x status enumeration definition
 */
typedef enum
{
    SEN5X_STATUS_FAN_SPEED_ERROR     = (1 << 21),       /**< fan speed is too high or too low */
    SEN5X_STATUS_FAN_CLEANING_ACTIVE = (1 << 19),       /**< fan cleaning active */
    SEN5X_STATUS_GAS_ERROR           = (1 << 7),        /**< gas sensor error */
    SEN5X_STATUS_RHT_ERROR           = (1 << 6),        /**< error in internal communication with the rht sensor */
    SEN5X_STATUS_LASER_ERROR         = (1 << 5),        /**< laser is switched on and current is out of range */
    SEN5X_STATUS_FAN_ERROR           = (1 << 4),        /**< fan is switched on but the measured fan speed is 0 rpm */
} sen5x_status_t;

/**
 * @brief sen5x rht acceleration mode enumeration definition
 */
typedef enum
{
    SEN5X_RHT_ACCELERATION_LOW    = 0,       /**< low acceleration */
    SEN5X_RHT_ACCELERATION_HIGH   = 1,       /**< high acceleration */
    SEN5X_RHT_ACCELERATION_MEDIUM = 2,       /**< medium acceleration */
} sen5x_rht_acceleration_mode_t;

/**
 * @brief sen55 data structure definition
 */
typedef struct sen55_data_s
{
    uint16_t pm1p0_raw;                                  /**< mass concentration pm1.0 raw */
    uint16_t pm2p5_raw;                                  /**< mass concentration pm2.5 raw */
    uint16_t pm4p0_raw;                                  /**< mass concentration pm4.0 raw */
    uint16_t pm10_raw;                                   /**< mass concentration pm10 raw */
    int16_t compensated_ambient_humidity_raw;            /**< compensated ambient humidity raw */
    int16_t compensated_ambient_temperature_raw;         /**< compensated ambient temperature raw */
    int16_t voc_raw;                                     /**< voc raw */
    int16_t nox_raw;                                     /**< nox raw */
    uint8_t pm_valid;                                    /**< pm valid */
    float pm1p0_ug_m3;                                   /**< mass concentration pm1.0 [μg/m3] */
    float pm2p5_ug_m3;                                   /**< mass concentration pm2.5 [μg/m3] */
    float pm4p0_ug_m3;                                   /**< mass concentration pm4.0 [μg/m3] */
    float pm10_ug_m3;                                    /**< mass concentration pm10 [μg/m3] */
    float compensated_ambient_humidity_percentage;       /**< compensated ambient humidity percentage */
    float compensated_ambient_temperature_degree;        /**< compensated ambient temperature degree */
    float voc_index;                                     /**< voc index */
    float nox_index;                                     /**< nox index */
} sen55_data_t;

/**
 * @brief sen54 data structure definition
 */
typedef struct sen54_data_s
{
    uint16_t pm1p0_raw;                                  /**< mass concentration pm1.0 raw */
    uint16_t pm2p5_raw;                                  /**< mass concentration pm2.5 raw */
    uint16_t pm4p0_raw;                                  /**< mass concentration pm4.0 raw */
    uint16_t pm10_raw;                                   /**< mass concentration pm10 raw */
    int16_t compensated_ambient_humidity_raw;            /**< compensated ambient humidity raw */
    int16_t compensated_ambient_temperature_raw;         /**< compensated ambient temperature raw */
    int16_t voc_raw;                                     /**< voc raw */
    uint8_t pm_valid;                                    /**< pm valid */
    float pm1p0_ug_m3;                                   /**< mass concentration pm1.0 [μg/m3] */
    float pm2p5_ug_m3;                                   /**< mass concentration pm2.5 [μg/m3] */
    float pm4p0_ug_m3;                                   /**< mass concentration pm4.0 [μg/m3] */
    float pm10_ug_m3;                                    /**< mass concentration pm10 [μg/m3] */
    float compensated_ambient_humidity_percentage;       /**< compensated ambient humidity percentage */
    float compensated_ambient_temperature_degree;        /**< compensated ambient temperature degree */
    float voc_index;                                     /**< voc index */
} sen54_data_t;

/**
 * @brief sen50 data structure definition
 */
typedef struct sen50_data_s
{
    uint16_t pm1p0_raw;        /**< mass concentration pm1.0 raw */
    uint16_t pm2p5_raw;        /**< mass concentration pm2.5 raw */
    uint16_t pm4p0_raw;        /**< mass concentration pm4.0 raw */
    uint16_t pm10_raw;         /**< mass concentration pm10 raw */
    uint8_t pm_valid;          /**< pm valid */
    float pm1p0_ug_m3;         /**< mass concentration pm1.0 [μg/m3] */
    float pm2p5_ug_m3;         /**< mass concentration pm2.5 [μg/m3] */
    float pm4p0_ug_m3;         /**< mass concentration pm4.0 [μg/m3] */
    float pm10_ug_m3;          /**< mass concentration pm10 [μg/m3] */
} sen50_data_t;

/**
 * @brief sen5x pm structure definition
 */
typedef struct sen5x_pm_s
{
    uint16_t mass_concentration_pm1p0_raw;          /**< mass concentration pm1.0 raw */
    uint16_t mass_concentration_pm2p5_raw;          /**< mass concentration pm2.5 raw */
    uint16_t mass_concentration_pm4p0_raw;          /**< mass concentration pm4.0 raw */
    uint16_t mass_concentration_pm10_raw;           /**< mass concentration pm10 raw */
    uint16_t number_concentration_pm0p5_raw;        /**< number concentration pm0.5 raw */
    uint16_t number_concentration_pm1p0_raw;        /**< number concentration pm1.0 raw */
    uint16_t number_concentration_pm2p5_raw;        /**< number concentration pm2.5 raw */
    uint16_t number_concentration_pm4p0_raw;        /**< number concentration pm4.0 raw */
    uint16_t number_concentration_pm10_raw;         /**< number concentration pm10 raw */
    uint16_t typical_particle_raw;                  /**< typical particle size raw */
    uint8_t pm_valid;                               /**< pm valid */
    float pm1p0_ug_m3;                              /**< mass concentration pm1.0 [μg/m3] */
    float pm2p5_ug_m3;                              /**< mass concentration pm2.5 [μg/m3] */
    float pm4p0_ug_m3;                              /**< mass concentration pm4.0 [μg/m3] */
    float pm10_ug_m3;                               /**< mass concentration pm10 [μg/m3] */
    float pm0p5_cm3;                                /**< number concentration pm0.5 [#/cm3] */
    float pm1p0_cm3;                                /**< number concentration pm1.0 [#/cm3] */
    float pm2p5_cm3;                                /**< number concentration pm2.5 [#/cm3] */
    float pm4p0_cm3;                                /**< number concentration pm4.0 [#/cm3] */
    float pm10_cm3;                                 /**< number concentration pm10 [#/cm3] */
    float typical_particle_um;                      /**< typical particle size[um] */
} sen5x_pm_t;

/**
 * @brief sen5x raw structure definition
 */
typedef struct sen5x_raw_s
{
    int16_t humidity_raw;             /**< humidity raw */
    int16_t temperature_raw;          /**< temperature raw */
    uint16_t voc_raw;                 /**< voc raw */
    uint16_t nox_raw;                 /**< nox raw */
    float humidity_percentage;        /**< humidity percentage */
    float temperature_degree;         /**< temperature degree */
    float voc;                        /**< voc value*/
    float nox;                        /**< nox value*/
} sen5x_raw_t;

/**
 * @brief sen5x handle structure definition
 */
typedef struct sen5x_handle_s
{
    uint8_t (*iic_init)(void);                                                /**< point to an iic_init function address */
    uint8_t (*iic_deinit)(void);                                              /**< point to an iic_deinit function address */
    uint8_t (*iic_write_cmd)(uint8_t addr, uint8_t *buf, uint16_t len);       /**< point to an iic_write_cmd function address */
    uint8_t (*iic_read_cmd)(uint8_t addr, uint8_t *buf, uint16_t len);        /**< point to an iic_read_cmd function address */
    void (*delay_ms)(uint32_t ms);                                            /**< point to a delay_ms function address */
    void (*debug_print)(const char *const fmt, ...);                          /**< point to a debug_print function address */
    uint8_t type;                                                             /**< type */
    uint8_t inited;                                                           /**< inited flag */
} sen5x_handle_t;

/**
 * @brief sen5x information structure definition
 */
typedef struct sen5x_info_s
{
    char chip_name[32];                /**< chip name */
    char manufacturer_name[32];        /**< manufacturer name */
    char interface[8];                 /**< chip interface name */
    float supply_voltage_min_v;        /**< chip min supply voltage */
    float supply_voltage_max_v;        /**< chip max supply voltage */
    float max_current_ma;              /**< chip max current */
    float temperature_min;             /**< chip min operating temperature */
    float temperature_max;             /**< chip max operating temperature */
    uint32_t driver_version;           /**< driver version */
} sen5x_info_t;

/**
 * @}
 */

/**
 * @defgroup sen5x_link_driver sen5x link driver function
 * @brief    sen5x link driver modules
 * @ingroup  sen5x_driver
 * @{
 */

/**
 * @brief     initialize sen5x_handle_t structure
 * @param[in] HANDLE pointer to a sen5x handle structure
 * @param[in] STRUCTURE sen5x_handle_t
 * @note      none
 */
#define DRIVER_SEN5X_LINK_INIT(HANDLE, STRUCTURE)              memset(HANDLE, 0, sizeof(STRUCTURE))

/**
 * @brief     link iic_init function
 * @param[in] HANDLE pointer to a sen5x handle structure
 * @param[in] FUC pointer to an iic_init function address
 * @note      none
 */
#define DRIVER_SEN5X_LINK_IIC_INIT(HANDLE, FUC)               (HANDLE)->iic_init = FUC

/**
 * @brief     link iic_deinit function
 * @param[in] HANDLE pointer to a sen5x handle structure
 * @param[in] FUC pointer to an iic_deinit function address
 * @note      none
 */
#define DRIVER_SEN5X_LINK_IIC_DEINIT(HANDLE, FUC)             (HANDLE)->iic_deinit = FUC

/**
 * @brief     link iic_write_cmd function
 * @param[in] HANDLE pointer to a sen5x handle structure
 * @param[in] FUC pointer to an iic_write_cmd function address
 * @note      none
 */
#define DRIVER_SEN5X_LINK_IIC_WRITE_COMMAND(HANDLE, FUC)      (HANDLE)->iic_write_cmd = FUC

/**
 * @brief     link iic_read_cmd function
 * @param[in] HANDLE pointer to a sen5x handle structure
 * @param[in] FUC pointer to an iic_read_cmd function address
 * @note      none
 */
#define DRIVER_SEN5X_LINK_IIC_READ_COMMAND(HANDLE, FUC)       (HANDLE)->iic_read_cmd = FUC

/**
 * @brief     link delay_ms function
 * @param[in] HANDLE pointer to a sen5x handle structure
 * @param[in] FUC pointer to a delay_ms function address
 * @note      none
 */
#define DRIVER_SEN5X_LINK_DELAY_MS(HANDLE, FUC)               (HANDLE)->delay_ms = FUC

/**
 * @brief     link debug_print function
 * @param[in] HANDLE pointer to a sen5x handle structure
 * @param[in] FUC pointer to a debug_print function address
 * @note      none
 */
#define DRIVER_SEN5X_LINK_DEBUG_PRINT(HANDLE, FUC)            (HANDLE)->debug_print = FUC

/**
 * @}
 */

/**
 * @defgroup sen5x_basic_driver sen5x basic driver function
 * @brief    sen5x basic driver modules
 * @ingroup  sen5x_driver
 * @{
 */

/**
 * @brief      get chip information
 * @param[out] *info pointer to a sen5x info structure
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 * @note       none
 */
uint8_t sen5x_info(sen5x_info_t *info);

/**
 * @brief     set the chip type
 * @param[in] *handle pointer to a sen5x handle structure
 * @param[in] type chip type
 * @return    status code
 *            - 0 success
 *            - 2 handle is NULL
 * @note      none
 */
uint8_t sen5x_set_type(sen5x_handle_t *handle, sen5x_type_t type);

/**
 * @brief      get the chip type
 * @param[in]  *handle pointer to a sen5x handle structure
 * @param[out] *type pointer to a chip type buffer
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 * @note       none
 */
uint8_t sen5x_get_type(sen5x_handle_t *handle, sen5x_type_t *type);

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
uint8_t sen5x_init(sen5x_handle_t *handle);

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
uint8_t sen5x_deinit(sen5x_handle_t *handle);

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
uint8_t sen5x_start_measurement(sen5x_handle_t *handle);

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
uint8_t sen5x_start_measurement_without_pm(sen5x_handle_t *handle);

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
uint8_t sen5x_stop_measurement(sen5x_handle_t *handle);

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
uint8_t sen5x_read_data_flag(sen5x_handle_t *handle, sen5x_data_ready_flag_t *flag);

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
uint8_t sen55_read(sen5x_handle_t *handle, sen55_data_t *output);

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
uint8_t sen54_read(sen5x_handle_t *handle, sen54_data_t *output);

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
uint8_t sen50_read(sen5x_handle_t *handle, sen50_data_t *output);

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
uint8_t sen5x_read_pm_value(sen5x_handle_t *handle, sen5x_pm_t *pm);

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
uint8_t sen5x_read_raw_value(sen5x_handle_t *handle, sen5x_raw_t *raw);

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
                                           uint16_t time_constant);

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
                                           uint16_t *time_constant);

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
                                                           uint16_t *time_constant_reg);

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
                                                       float *time_constant_second);

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
uint8_t sen5x_set_warm_start(sen5x_handle_t *handle, uint16_t param);

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
uint8_t sen5x_get_warm_start(sen5x_handle_t *handle, uint16_t *param);

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
                                       int16_t gain_factor);

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
                                       int16_t *gain_factor);

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
                                       int16_t gain_factor);

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
                                       int16_t *gain_factor);

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
uint8_t sen5x_set_rht_acceleration_mode(sen5x_handle_t *handle, sen5x_rht_acceleration_mode_t mode);

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
uint8_t sen5x_get_rht_acceleration_mode(sen5x_handle_t *handle, sen5x_rht_acceleration_mode_t *mode);

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
uint8_t sen5x_set_voc_algorithm_state(sen5x_handle_t *handle, uint16_t state[4]);

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
uint8_t sen5x_get_voc_algorithm_state(sen5x_handle_t *handle, uint16_t state[4]);

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
uint8_t sen5x_start_fan_cleaning(sen5x_handle_t *handle);

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
uint8_t sen5x_set_auto_cleaning_interval(sen5x_handle_t *handle, uint32_t second);

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
uint8_t sen5x_get_auto_cleaning_interval(sen5x_handle_t *handle, uint32_t *second);

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
uint8_t sen5x_disable_auto_cleaning_interval(sen5x_handle_t *handle);

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
uint8_t sen5x_get_product_name(sen5x_handle_t *handle, char name[32]);

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
uint8_t sen5x_get_serial_number(sen5x_handle_t *handle, char sn[32]);

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
uint8_t sen5x_get_version(sen5x_handle_t *handle, uint8_t *version);

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
uint8_t sen5x_get_device_status(sen5x_handle_t *handle, uint32_t *status);

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
uint8_t sen5x_clear_device_status(sen5x_handle_t *handle);

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
uint8_t sen5x_reset(sen5x_handle_t *handle);

/**
 * @}
 */

/**
 * @defgroup sen5x_extern_driver sen5x extern driver function
 * @brief    sen5x extern driver modules
 * @ingroup  sen5x_driver
 * @{
 */

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
uint8_t sen5x_set_reg(sen5x_handle_t *handle, uint16_t reg, uint8_t *buf, uint16_t len);

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
uint8_t sen5x_get_reg(sen5x_handle_t *handle, uint16_t reg, uint8_t *buf, uint16_t len);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
