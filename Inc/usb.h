
#ifndef USB_CPP_
#define USB_CPP_

#include <stdint.h>
#include <stdbool.h>

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

/*
#undef USBx_DEVICE
#undef USBx_INEP
#undef USBx_OUTEP
#undef USBx_DFIFO

#define USBx_DEVICE   ((USB_OTG_DeviceTypeDef*)      (USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE))
#define USBx_INEP(i)  ((USB_OTG_INEndpointTypeDef*)  (USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE  + (i) * USB_OTG_EP_REG_SIZE))
#define USBx_OUTEP(i) ((USB_OTG_OUTEndpointTypeDef*) (USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (i) * USB_OTG_EP_REG_SIZE))
#define USBx_DFIFO(i) *(__IO uint32_t*)              (USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE         + (i) * USB_OTG_FIFO_SIZE)
*/

class USB_Class {

public:
	explicit USB_Class();

	static void init();
	static void interrupt();

	static HAL_StatusTypeDef writePacket(uint8_t *source, uint32_t epnum, size_t length);
	static HAL_StatusTypeDef writeEmptyTxFifo(PCD_HandleTypeDef *hpcd, uint32_t epnum);

private:

	static inline void maskInterrupt(uint32_t interrupt) {
		USB_OTG_FS->GINTMSK &= ~interrupt;
	}

	static inline void unmaskInterrupt(uint32_t interrupt) {
		USB_OTG_FS->GINTMSK |= interrupt;
	}

	static inline void clearInEndpointInterrupt(int endpoint, uint32_t interrupt) {
		inEndpoint(endpoint)->DIEPINT = interrupt;
	}

	static inline void clearOutEndpointInterrupt(int endpoint, uint32_t interrupt) {
		outEndpoint(endpoint)->DOEPINT = interrupt;
	}

	static inline uint32_t readInterrupts(void) {
		return USB_OTG_FS->GINTSTS & USB_OTG_FS->GINTMSK;
	}

	static inline void clearInerrupt(uint32_t interrupt) {
		USB_OTG_FS->GINTSTS = interrupt;
	}

	static inline bool isInvalidInterrupt(void) {
		return readInterrupts() == 0U;
	}

	static inline bool getInterruptState(uint32_t interrupt) {
		return (readInterrupts() & interrupt) == interrupt;
	}

	// Get OTG mode. Return values: true - host, false - device
	static inline bool getMode() {
		return getInterruptState(USB_OTG_GINTSTS_CMOD);
	}

	constexpr static USB_OTG_DeviceTypeDef *device() {
		return reinterpret_cast<USB_OTG_DeviceTypeDef*> (USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE);
	}

	constexpr static USB_OTG_INEndpointTypeDef *inEndpoint(const int index) {
		return reinterpret_cast<USB_OTG_INEndpointTypeDef*> (
			USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + USB_OTG_EP_REG_SIZE * index
		);
	}

	constexpr static USB_OTG_OUTEndpointTypeDef *outEndpoint(const int index) {
		return reinterpret_cast<USB_OTG_OUTEndpointTypeDef*> (
			USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + USB_OTG_EP_REG_SIZE * index
		);
	}

	constexpr static volatile uint32_t *fifo(const int index) {
		return reinterpret_cast<uint32_t*> (
			USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE + USB_OTG_FIFO_SIZE * index
		);
	}
};

extern USB_Class USB;

#endif /* USB_CPP_ */
