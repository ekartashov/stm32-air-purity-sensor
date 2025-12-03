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
 * @file      driver_scd4x_interface_template.c
 * @brief     driver scd4x interface template source file
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

#include "driver_scd4x_interface.h"
// #include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdarg.h>

extern I2C_HandleTypeDef hi2c4;

/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t scd4x_interface_iic_init(void)
{ // chatgpt says this should not be called because we already call MX_I2C4_Init() in main.c
  if(HAL_I2C_Init(&hi2c4) == HAL_OK) {
      return 0;
  }
  return 1;
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t scd4x_interface_iic_deinit(void)
{

    if(HAL_I2C_DeInit(&hi2c4) == HAL_OK) {
        return 0;
    }
    return 1;
}

/**
 * @brief     interface iic bus write command
 * @param[in] addr iic device write address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t scd4x_interface_iic_write_cmd(uint8_t addr, uint8_t *buf, uint16_t len)
{
  if(HAL_I2C_Master_Transmit(&hi2c4, addr << 1, buf, len, 100) == HAL_OK) {
    return 0;
  } 
  return 1;
}

/**
 * @brief      interface iic bus read command
 * @param[in]  addr iic device write address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of the data buffer
 * @return     status code
 *             - 0 success
*             - 1 read failed
 * @note       none
 */
uint8_t scd4x_interface_iic_read_cmd(uint8_t addr, uint8_t *buf, uint16_t len)
{
  if(HAL_I2C_Master_Receive(&hi2c4, addr << 1, buf, len, 100) == HAL_OK) {
    return 0;
  } 
  return 1;
}

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void scd4x_interface_delay_ms(uint32_t ms)
{
  HAL_Delay(ms);
}

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void scd4x_interface_debug_print(const char *const fmt, ...)
{
  char buf[256];

  va_list args;
  va_start(args, fmt);
  // Leave space for the newline and null terminator by subtracting 2
  int len = vsnprintf(buf, sizeof(buf) - 2, fmt, args); 
  va_end(args);

  if (len < 0)
  {
      return;
  }
  
  
  buf[len] = '\r'; // Append the reset caddy
  len++;           // Increase length to account for the newline
  buf[len] = '\0'; // Keep it null-terminated

  /* Send over USB CDC */
  (void)CDC_Transmit_FS((uint8_t *)buf, (uint16_t)len);
}
