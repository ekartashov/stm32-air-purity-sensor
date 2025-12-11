/* Core/Inc/Drivers/Serial/driver_serial.h */

#ifndef DRIVER_SERIAL_H
#define DRIVER_SERIAL_H

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>


/**
 * @brief vprintf-style serial output over USB CDC.
 */
void serial_vprint(const char *fmt, va_list args);

/**
 * @brief printf-style serial output over USB CDC.
 */
void serial_print(const char *fmt, ...);

#endif /* DRIVER_SERIAL_H */
