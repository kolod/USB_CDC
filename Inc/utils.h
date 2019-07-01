/*
 * utils.h
 *
 *  Created on: 1 июл. 2019 г.
 *      Author: user
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>


#ifndef UNUSED
#define UNUSED(x)                 (void) x
#endif

#ifndef _BV
#define _BV(bit)                  (1 << (bit))
#endif

#ifndef CONSTRAIN
#define CONSTRAIN(val, min, max)  ((val > max) ? max : (val < min) ? min : val)
#endif

#ifndef MIN
#define MIN(x,y)                  ((x > y) ? y : x)
#endif

#ifndef MAX
#define MAX(x,y)                  ((x < y) ? y : x)
#endif

#ifndef LOBYTE
#define LOBYTE(x)                 ((uint8_t)(x & 0x00FF))
#endif

#ifndef HIBYTE
#define HIBYTE(x)                 ((uint8_t)((x & 0xFF00) >>8))
#endif


#define GPIO_PIN_SET(port, pin)   WRITE_REG(port->BSRR, pin)
#define GPIO_PIN_RESET(port, pin) WRITE_REG(port->BSRR, (pin << 16))
#define GPIO_PIN_READ(port, pin)  (READ_BIT(port->IDR, pin) == pin)

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
