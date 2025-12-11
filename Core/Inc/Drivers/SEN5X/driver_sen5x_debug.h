/* driver_sen5x_debug.h */

#ifndef DRIVER_SEN5X_DEBUG_H
#define DRIVER_SEN5X_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the global SEN5X handle with default links.
 *
 * This only wires up the function pointers and sets the device type
 * to SEN55. It does NOT talk to the sensor yet.
 */
void sen5x_init_handle_default(void);

/**
 * @brief Run a full one-shot debug test of the SEN5x driver.
 *
 * - Initializes the driver
 * - Prints info (chip, manufacturer, interface, type, product, serial, version, status)
 * - Persists settings
 * - Starts measurement
 * - Waits a bit and then:
 *      - Reads PM data (PM1.0 / PM2.5 / PM4.0 / PM10, number concentrations, typical size)
 *      - Tries multiple times to read raw data (T, RH, VOC, NOx)
 * - Stops measurement
 * - Fan cleaning & clear status
 * - Reads a raw register (0x202F) for sanity
 * - Deinitializes the driver
 *
 * You can safely call this from inside while(1); it will only execute once
 * thanks to an internal guard.
 */
void sen5x_run_full_test_once(void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_SEN5X_DEBUG_H */
