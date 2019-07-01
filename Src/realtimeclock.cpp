// Copyright (c) 2018-2019 Alexandr Kolodkin <alexandr.kolodkin@gmail.com>
// All rights reserved.


#include "realtimeclock.h"

// Put "TZ=EET-2EEST,M3.5.0/3,M10.5.0/4" to system environment constant

RealtimeClock::RealtimeClock() {}

bool RealtimeClock::init() {

	// Enable access to the backup registers and the Clock.
	PWR->CR |= PWR_CR_DBP;

	// TODO: не работает при отключении батарейки
	if (RCC->BDCR & RCC_BDCR_RTCEN) {
		init_rtc:
		// Set the clock source
		// Turn the HSE on and wait while it stabilizes.
		SET_BIT(RCC->BDCR, RCC_BDCR_LSEON);
		while (!READ_BIT(RCC->CR, RCC_CR_HSERDY));   // TODO: Add timeout

		// Choose HSE as the Clock clock source.
		RCC->BDCR |= RCC_BDCR_RTCSEL_LSE;

		// Enter configuration mode.
		enterConfigMode();

		RTC->CRH  = 0;
		RTC->CRL  = 0x20;
		RTC->PRLH = 0;
		RTC->PRLL = 0x7FFF;
		RTC->CNTH = 0;
		RTC->CNTL = 0;
		RTC->ALRH = 0xFFFF;
		RTC->ALRL = 0xFFFF;

		// Exit configuration mode.
		exitConfigMode();

		// Enable the Clock.
		SET_BIT(RCC->BDCR, RCC_BDCR_RTCEN);
	}

	int count = RTC_STURTUP_COUNT;

	// Wait for the RSF bit in RTC_CRL to be set by hardware.
	RTC->CRL &= ~RTC_CRL_RSF;
	int timeout = RTC_STURTUP_TIMEOUT;
	while ((RTC->CRL & RTC_CRL_RSF) == 0) {
		if (--timeout == 0) {
			if (--count) goto init_rtc; else return false;
		}
	}

	// Wait for the last write operation to finish.
	// TODO: Necessary? */
	timeout = RTC_STURTUP_TIMEOUT;
	while ((RTC->CRL & RTC_CRL_RTOFF) == 0) {
		if (--timeout == 0) {
			if (--count) goto init_rtc; else return false;
		}
	}

	return true;
}

void RealtimeClock::enterConfigMode() {
	// Wait until the RTOFF bit is 1 (no Clock register writes ongoing).
	while ((RTC->CRL & RTC_CRL_RTOFF) == 0);

	// Enter configuration mode.
	RTC->CRL |= RTC_CRL_CNF;
}

void RealtimeClock::exitConfigMode() {
	// Exit configuration mode.
	RTC->CRL &= ~RTC_CRL_CNF;

	// Wait until the RTOFF bit is 1 (our Clock register write finished).
	while ((RTC->CRL & RTC_CRL_RTOFF) == 0);
}

time_t RealtimeClock::value() {
	return (RTC->CNTH << 16) | RTC->CNTL;
}

void RealtimeClock::setValue(time_t val) {
	enterConfigMode();
	RTC->CNTH = (val >> 16) & 0x0000ffff;
	RTC->CNTL = (val      ) & 0x0000ffff;
	exitConfigMode();
}

RealtimeClock realtimeClock;

extern "C" int _gettimeofday(struct timeval *tv, struct timezone *tz) {
	UNUSED(tz);

	if (tv == NULL) return -1;

	tv->tv_sec  = realtimeClock.value();
	tv->tv_usec = 0;                    //TODO: Add sub seconds time part

	return 0;
}
