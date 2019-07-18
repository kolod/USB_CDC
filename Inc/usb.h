
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

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

class USB_Class {

public:
	explicit USB_Class();

	void init();
	void interrupt();

	void activateSetup();
	void writePacket(uint8_t *source, uint32_t epnum, size_t length);
	HAL_StatusTypeDef writeEmptyTxFifo(PCD_HandleTypeDef *hpcd, uint32_t epnum);


private:

	void getUnicodeString(const uint8_t * ascii, uint8_t *unicode, uint16_t *length);
	void getUnicodeString(uint32_t value, uint8_t *unicode, uint8_t length);
	void getSerialNumber(uint8_t *unicode);

	void start();
	void setStall(uint8_t endpoint);
	void endpointTransmit(uint8_t ep_addr, uint8_t *pBuf, uint32_t len);
	void startEndpoint0Xfer(USB_OTG_EPTypeDef *ep);
	void startEndpointXfer(USB_OTG_EPTypeDef *ep);
	void deviceInit();

	bool flushTxFifo(int32_t num);
	bool flushRxFifo();

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

	void onDataInStage(uint8_t epnum, uint8_t *pdata);

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

	inline void sendControlStatus() {
		reinterpret_cast<USBD_HandleTypeDef*>(hpcd_USB_OTG_FS.pData)->ep0_state = USBD_EP0_STATUS_IN;   // Set EP0 State
		endpointTransmit(0x00, nullptr, 0);                                                             // Start the transfer
	}

	inline void enableGlobalInterrupt() {
		USB_OTG_FS->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
	}

	inline void disableGlobalInterrupt() {
		USB_OTG_FS->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;
	}

	inline void setDeviceMode() {
		USB_OTG_FS->GUSBCFG &= ~(USB_OTG_GUSBCFG_FHMOD | USB_OTG_GUSBCFG_FDMOD);
		USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
		millisecondDelay(50);
	}

	inline void setDeviceSpeed(uint8_t speed) {
		device()->DCFG |= speed;
	}

	inline void restartPhyClock() {
		*reinterpret_cast<volatile uint32_t*>(USB_OTG_FS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE) = 0;
	}

	inline void deviceConnect() {
		device()->DCTL &= ~USB_OTG_DCTL_SDIS;
	}

	inline void deviceDisconnect() {
		device()->DCTL |= USB_OTG_DCTL_SDIS;
		millisecondDelay(3);
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

	inline bool selectPhy() {
		// Init the Core (common init.)
		USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;  // TODO: read only bit ?

		// Reset after a PHY select and set Host mode
		for (uint32_t timeout = 200000; timeout; timeout--) {
			if (USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) return true;
		}

		return false;
	}

	inline bool coreSoftReset() {
		USB_OTG_FS->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;

		for (uint32_t timeout = 200000; timeout; timeout--) {
			if ((USB_OTG_FS->GRSTCTL & USB_OTG_GRSTCTL_CSRST) == 0) return true;
		}

		return false;
	}

	// Get OTG mode. Return values: true - host, false - device
	inline bool getMode() {
		return getInterruptState(USB_OTG_GINTSTS_CMOD);
	}

	inline uint32_t readDevAllOutEpInterrupt() {
		return (device()->DAINT & device()->DAINTMSK & 0xFFFF0000) >> 16;
	}

	inline uint32_t readDevOutEPInterrupt(uint8_t epnum) {
		return outEndpoint(epnum)->DOEPINT & device()->DOEPMSK;
	}

	inline uint32_t readDevAllInEpInterrupt() {
		return device()->DAINT & device()->DAINTMSK & 0xFFFF;
	}

	inline uint32_t readDevInEPInterrupt(uint8_t epnum) {
		return inEndpoint(epnum)->DIEPINT & (((device()->DIEPEMPMSK >> epnum) & 1) << 7 | device()->DIEPMSK);
	}

	inline void setRxFiFo(uint16_t size) {
		USB_OTG_FS->GRXFSIZ = size;              // Save size of FiFo buffer
		USB_OTG_FS->DIEPTXF0_HNPTXFSIZ = size;   // Save offset for endpoint 0
	}

	inline void setTxFiFo(const uint8_t fifo, const uint16_t size) {
		if (fifo == 0) {
			USB_OTG_FS->DIEPTXF[0] = USB_OTG_FS->DIEPTXF0_HNPTXFSIZ + size;   // Save offset for endpoint 1
			USB_OTG_FS->DIEPTXF0_HNPTXFSIZ |= (size << 16);                   // Save size of FiFo buffer
		} else {
			USB_OTG_FS->DIEPTXF[fifo] = USB_OTG_FS->DIEPTXF[fifo - 1] + size; // Save offset for next endpoint
			if (fifo < 3) USB_OTG_FS->DIEPTXF[fifo - 1] |= (size << 16);      // Save size of FiFo buffer
		}
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
