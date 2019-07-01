

#include "config.h"
#include "stm32f1xx.h"
#include "utils.h"

extern uint32_t uptime;

#pragma GCC push_options
#pragma GCC optimize ("O3")

void uDelay(const uint32_t us) {
	uint32_t start = SysTick->VAL;
	uint32_t cycles = us * (F_CPU / 1000000UL) - (uint32_t) (0.64 * (F_CPU / 1000000UL));
	while (start - SysTick->VAL < cycles);
}

#pragma GCC pop_options

void mDelay(const uint32_t ms) {
	uint32_t tickstart = uptime;
	uint32_t wait = ms;

	/* Add a freq to guarantee minimum wait */
	if (wait < UINT32_MAX) wait += 1;

	while ((uptime - tickstart) < wait);
}

uint32_t millis() {
	return uptime;
}

uint32_t micros() {
	uint64_t result = (uint64_t) uptime * 1000;
	result += SysTick->VAL / (F_CPU / 1000000UL);
	return result & 0xFFFFFFFF; // TODO: uptime++ if SysTick->VAL > some value
}

