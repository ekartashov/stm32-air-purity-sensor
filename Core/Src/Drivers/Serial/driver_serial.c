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
    char buf[256];

    /* Format string into buffer, leaving space for '\r' and '\0' */
    int len = vsnprintf(buf, sizeof(buf) - 2, fmt, args);
    if (len < 0) {
        return;
    }
    if (len > (int)(sizeof(buf) - 3)) {
        len = (int)(sizeof(buf) - 3);
    }

    buf[len] = '\r';   /* append CR (your terminal already shows LF) */
    len++;
    buf[len] = '\0';

    /* If USB is not configured yet, just drop the message.
     * The idea: in places where you care (like early boot),
     * first call:
     *   while (!usb_cdc_is_configured()) HAL_Delay(5);
     * and only then use serial_print().
     */
    if (!usb_cdc_is_configured()) {
        return;
    }

    /* Try to transmit, simple retry loop if BUSY */
    const uint32_t timeout_ms = 20;
    uint32_t start = HAL_GetTick();

    while (CDC_Transmit_FS((uint8_t *)buf, (uint16_t)len) == USBD_BUSY) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            /* still busy, give up */
            return;
        }
    }
}

void serial_print(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    serial_vprint(fmt, args);
    va_end(args);
}
