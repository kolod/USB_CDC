
#ifndef USB_CPP_
#define USB_CPP_

#include <stdint.h>
#include <stdbool.h>

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

class USB_Class {
public:
	explicit USB_Class();

	void init();
	void interrupt();

private:
	HAL_StatusTypeDef PCD_WriteEmptyTxFifo(PCD_HandleTypeDef *hpcd, uint32_t epnum);
};

extern USB_Class USB;

#endif /* USB_CPP_ */
