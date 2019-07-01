
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "realtimeclock.h"
#include "utils.h"


extern "C" int main(void) {
	const char text[] = "Hello!\n";

	HAL_Init();
	realtimeClock.init();
	MX_USB_DEVICE_Init();

	GPIO_PIN_SET(USB_EN_GPIO_Port, USB_EN_Pin);

	for(;;) {
		CDC_Transmit_FS((uint8_t*) text, (uint16_t) sizeof(text) - 1);
		millisecondDelay(2000);
	}
}

extern "C" void Error_Handler(void) {}

#ifdef  USE_FULL_ASSERT
extern "C" void assert_failed(uint8_t *file, uint32_t line) {}
#endif // USE_FULL_ASSERT
