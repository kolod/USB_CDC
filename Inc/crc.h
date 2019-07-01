// Copyright (c) 2018-2019 Alexandr Kolodkin <alexandr.kolodkin@gmail.com>
// All rights reserved.


#ifndef CRC_H_
#define CRC_H_

#include "stm32f1xx.h"
#include <string.h>

class Crc {
public:
	explicit Crc();

	uint32_t calculate(uint8_t *data, const size_t length);
	void update(uint8_t *data, const size_t length);
};

extern Crc crc;

#endif /* CRC_H_ */
