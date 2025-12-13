/* Core/Src/Drivers/Serial/driver_serial.c */

#include "Drivers/Serial/driver_serial.h"

#include "main.h"
#include "usb_device.h"
#include "usbd_def.h"
#include "usbd_cdc_if.h"

#include <stdio.h>
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

/**
 * @brief Return true once the USB device is in CONFIGURED state.
 *
 * This is what you should poll in main():
 *
 *   while (!usb_cdc_is_configured()) {
 *       HAL_Delay(5);
 *   }
 */
bool usb_cdc_is_configured(void)
{
    return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED);
}

void serial_vprint(const char *fmt, va_list args)
{
    char buf[SERIAL_TX_BUF_SZ];

    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    if (len < 0) {
        return;
    }
    if ((size_t)len >= sizeof(buf)) {
        len = (int)sizeof(buf) - 1;
        buf[len] = '\0';
    }

    /* Normalize trailing line ending: strip trailing \r/\n and add \r\n */
    while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r')) {
        buf[--len] = '\0';
    }
    if ((size_t)(len + 2) < sizeof(buf)) {
        buf[len++] = '\r';
        buf[len++] = '\n';
        buf[len] = '\0';
    }

    if (!usb_cdc_is_configured()) {
        return;
    }

    USBD_CDC_HandleTypeDef *hcdc =
        (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hcdc == NULL) {
        return;
    }

    /* Wait until previous TX completes */
    uint32_t t0 = HAL_GetTick();
    while (hcdc->TxState != 0U) {
        if ((HAL_GetTick() - t0) > SERIAL_TX_RETRY_TIMEOUT_MS) {
            return;
        }
        HAL_Delay(1);
    }

    if (CDC_Transmit_FS((uint8_t *)buf, (uint16_t)len) != USBD_OK) {
        return;
    }

    /* Wait until this TX completes (prevents stack-buffer corruption) */
    t0 = HAL_GetTick();
    while (hcdc->TxState != 0U) {
        if ((HAL_GetTick() - t0) > SERIAL_TX_RETRY_TIMEOUT_MS) {
            return;
        }
        HAL_Delay(1);
    }
}

void serial_print(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    serial_vprint(fmt, args);
    va_end(args);
}
