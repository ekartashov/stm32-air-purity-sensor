/* driver_scd4x_debug.h */

#ifndef DRIVER_SCD4X_DEBUG_H
#define DRIVER_SCD4X_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Run a full one-shot debug test of the SCD4x driver.
 *
 * You can safely call this from inside while(1):
 * it will only execute once thanks to an internal flag.
 */
void scd4x_run_full_test_once(void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_SCD4X_DEBUG_H */
