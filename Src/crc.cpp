// Copyright (c) 2018-2019 Alexandr Kolodkin <alexandr.kolodkin@gmail.com>
// All rights reserved.


#include "crc.h"

Crc::Crc() {}

uint32_t Crc::calculate(uint8_t *data, size_t length) {

	// Reset hardware
	CRC->CR = CRC_CR_RESET;

	size_t dwordCount    = length / 4;

	// Calculate main part
	while (dwordCount--) {
		CRC->DR = *reinterpret_cast<uint32_t*>(data);
		data += 4;
	}

	// Calculate last part
	switch(length % 4) {
	case 1:
		CRC->DR = static_cast<__IO uint32_t>(*data) << 24;
		break;

	case 2:
		CRC->DR = static_cast<__IO uint32_t>(*reinterpret_cast<uint16_t*>(data)) << 16;
		break;

	case 3:
		CRC->DR =
			(static_cast<__IO uint32_t>(*reinterpret_cast<uint16_t*>(data)) << 8) +
			(static_cast<__IO uint32_t>(*(data+2)) << 24);
	}

	return CRC->DR;
}

void Crc::update(uint8_t *data, const size_t length) {
	uint32_t *crc = reinterpret_cast<uint32_t*>(data + length - 4);
	*crc = calculate(data, length - 4);
}

Crc crc;
