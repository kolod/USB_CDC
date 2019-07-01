/*
 * utils.h
 *
 *  Created on: 1 июл. 2019 г.
 *      Author: user
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>

#if (INCLUDE_vTaskDelay == 1)
#define millisecondDelay(ms) vTaskDelay(ms)
#elif defined(USE_HAL_DRIVER)
#define millisecondDelay(ms) HAL_Delay(ms)
#else
#define millisecondDelay(ms) mDelay(ms)
#endif

#define microsecondDelay(us) uDelay(us)

#ifdef __cplusplus
extern "C" {
#endif

extern void mDelay(const uint32_t ms);
extern void uDelay(const uint32_t us);
extern uint32_t millis();
extern uint32_t micros();

#ifdef __cplusplus
}
#endif

#endif /* UTILS_H_ */
