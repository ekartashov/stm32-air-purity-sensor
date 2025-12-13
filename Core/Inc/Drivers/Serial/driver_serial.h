/* Core/Inc/Drivers/Serial/driver_serial.h */

#ifndef DRIVER_SERIAL_H
#define DRIVER_SERIAL_H

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#define SERIAL_TX_RETRY_TIMEOUT_MS 5
#define SERIAL_TX_BUF_SZ 1024

/**
 * @brief vprintf-style serial output over USB CDC.
 */
void serial_vprint(const char *fmt, va_list args);

/**
 * @brief printf-style serial output over USB CDC.
 */
void serial_print(const char *fmt, ...);

#endif /* DRIVER_SERIAL_H */
