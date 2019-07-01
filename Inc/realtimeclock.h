// Copyright (c) 2018-2019 Alexandr Kolodkin <alexandr.kolodkin@gmail.com>
// All rights reserved.


#ifndef REALTIMECLOCK_H_
#define REALTIMECLOCK_H_

#include "stm32f1xx.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <time.h>

#include "config.h"

class RealtimeClock {
public:
	explicit RealtimeClock();

	bool init();
	time_t value();
	void setValue(time_t val);

private:
	void enterConfigMode();
	void exitConfigMode();
};

extern RealtimeClock realtimeClock;

#endif /* REALTIMECLOCK_H_ */
