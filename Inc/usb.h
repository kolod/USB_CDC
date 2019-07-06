
#ifndef USB_CPP_
#define USB_CPP_

#include <stdint.h>
#include <stdbool.h>

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "usbd_def.h"

#include "utils.h"

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

	void init();
	void interrupt();

	void activateSetup();
	HAL_StatusTypeDef writePacket(uint8_t *source, uint32_t epnum, size_t length);
	HAL_StatusTypeDef writeEmptyTxFifo(PCD_HandleTypeDef *hpcd, uint32_t epnum);


private:

	void start();
	void setStall(uint8_t endpoint);

	void onConnect();
	void onDisconnect();
	void onIsoOutIncomlete();
	void onIsoInIncomplete();
	void onFrameStart();
	void onRxQLevel();
	void onEnumerationDone();
	void onReset();
	void onSuspend();
	void onResume();
	void onOut();
	void onIn();
	void onSetupStage();
	void onStdDevReq();
	void onGetDescriptor();
	void onSetAddress();
	void onSetConfig();
	void onGetConfig();
	void onGetStatus();
	void onSetFeature();
	void onClearFeature();

	inline void controllError() {
		setStall(0x80);
		setStall(0x00);
	}

	inline void endpoint0OutStart() {
		outEndpoint(0)->DOEPTSIZ  = 0;
		outEndpoint(0)->DOEPTSIZ |= (1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos) & USB_OTG_DOEPTSIZ_PKTCNT;
		outEndpoint(0)->DOEPTSIZ |= (3 * 8);
		outEndpoint(0)->DOEPTSIZ |= USB_OTG_DOEPTSIZ_STUPCNT;
	}

	inline void enableGlobalInterrupt() {
		USB_OTG_FS->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
	}

	inline void deviceConnect() {
		device()->DCTL &= ~USB_OTG_DCTL_SDIS;
	}

	inline void maskInterrupt(uint32_t interrupt) {
		USB_OTG_FS->GINTMSK &= ~interrupt;
	}

	inline void unmaskInterrupt(uint32_t interrupt) {
		USB_OTG_FS->GINTMSK |= interrupt;
	}

	inline void clearInEndpointInterrupt(int endpoint, uint32_t interrupt) {
		inEndpoint(endpoint)->DIEPINT = interrupt;
	}

	inline void clearOutEndpointInterrupt(int endpoint, uint32_t interrupt) {
		outEndpoint(endpoint)->DOEPINT = interrupt;
	}

	inline uint32_t readInterrupts(void) {
		return USB_OTG_FS->GINTSTS & USB_OTG_FS->GINTMSK;
	}

	inline void clearInerrupt(uint32_t interrupt) {
		USB_OTG_FS->GINTSTS = interrupt;
	}

	inline bool isInvalidInterrupt(void) {
		return readInterrupts() == 0U;
	}

	inline bool getInterruptState(uint32_t interrupt) {
		return (readInterrupts() & interrupt) == interrupt;
	}

	// Get OTG mode. Return values: true - host, false - device
	inline bool getMode() {
		return getInterruptState(USB_OTG_GINTSTS_CMOD);
	}

	inline uint32_t ReadDevAllOutEpInterrupt() {
		return (device()->DAINT & device()->DAINTMSK & 0xFFFF0000) >> 16;
	}

	inline uint32_t ReadDevOutEPInterrupt(uint8_t epnum) {
		return outEndpoint(epnum)->DOEPINT & device()->DOEPMSK;
	}

	inline uint32_t ReadDevAllInEpInterrupt() {
		return device()->DAINT & device()->DAINTMSK & 0xFFFF;
	}

	inline uint32_t ReadDevInEPInterrupt(uint8_t epnum) {
		return inEndpoint(epnum)->DIEPINT & (((device()->DIEPEMPMSK >> epnum) & 1) << 7 | device()->DIEPMSK);
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