/* driver_sen5x_debug.h */

#ifndef DRIVER_SEN5X_DEBUG_H
#define DRIVER_SEN5X_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Run a full one-shot debug test of the SEN5x driver.
 *
 * You can safely call this from inside while(1):
 * it will only execute once thanks to an internal flag.
 */
void sen5x_run_full_test_once(void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_SEN5X_DEBUG_H */
