
#include "usb.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

#undef USBx_DEVICE
#undef USBx_INEP
#undef USBx_OUTEP
#undef USBx_DFIFO

#define USBx_DEVICE   ((USB_OTG_DeviceTypeDef*)      (USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE))
#define USBx_INEP(i)  ((USB_OTG_INEndpointTypeDef*)  (USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE  + (i) * USB_OTG_EP_REG_SIZE))
#define USBx_OUTEP(i) ((USB_OTG_OUTEndpointTypeDef*) (USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (i) * USB_OTG_EP_REG_SIZE))
#define USBx_DFIFO(i) *(__IO uint32_t*)              (USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE         + (i) * USB_OTG_FIFO_SIZE)


//////////////////////////////////////////////////////////////////////////

static inline uint32_t OTG_ReadInterrupts(void) {
	return USB_OTG_FS->GINTSTS & USB_OTG_FS->GINTMSK;
}

static inline void OTG_ClearInerrupt(uint32_t interrupt) {
	USB_OTG_FS->GINTSTS = interrupt;
}

static inline bool OTG_IsInvalidInterrupt(void) {
	return OTG_ReadInterrupts() == 0U;
}

static inline bool OTG_GetInterruptState(uint32_t interrupt) {
	return (OTG_ReadInterrupts() & interrupt) == interrupt;
}

// Get OTG mode
// Return values:
//   true - host mode
//  false - device mode
inline bool OTG_GetMode() {
	return OTG_GetInterruptState(USB_OTG_GINTSTS_CMOD);
}

//////////////////////////////////////////////////////////////////////////

USB_Class::USB_Class() {
	;
}

HAL_StatusTypeDef USB_Class::PCD_WriteEmptyTxFifo(PCD_HandleTypeDef *hpcd, uint32_t epnum) {

	USB_OTG_EPTypeDef *ep = NULL;
	uint32_t len = 0;
	uint32_t len32b = 0U;
	uint32_t fifoemptymsk = 0U;

	ep = &hpcd->IN_ep[epnum];
	len = ep->xfer_len - ep->xfer_count;

	if (len > ep->maxpacket) len = ep->maxpacket;
	len32b = (len + 3U) / 4U;

	while ((USBx_INEP(epnum)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV) > len32b &&
			ep->xfer_count < ep->xfer_len &&
			ep->xfer_len != 0U)
	{
		/* Write the FIFO */
		len = ep->xfer_len - ep->xfer_count;

		if ((uint32_t)len > ep->maxpacket) len = ep->maxpacket;
		len32b = (len + 3U) / 4U;

		USB_WritePacket(USB_OTG_FS, ep->xfer_buff, epnum, len);

		ep->xfer_buff  += len;
		ep->xfer_count += len;
	}

	if (len <= 0) {
		fifoemptymsk = 0x01U << epnum;
		USBx_DEVICE->DIEPEMPMSK &= ~fifoemptymsk;
	}

	return HAL_OK;
}

void USB_Class::interrupt() {

	uint32_t index = 0U, ep_intr = 0U, epint = 0U, epnum = 0U;
	uint32_t fifoemptymsk = 0U, temp = 0U;
	USB_OTG_EPTypeDef *ep = NULL;

	// ensure that we are in device mode
	if (!OTG_GetMode()) {

		// avoid spurious interrupt
		if (OTG_IsInvalidInterrupt()) return;

		/* incorrect mode, acknowledge the interrupt */
		if (OTG_GetInterruptState(USB_OTG_GINTSTS_MMIS))
			OTG_ClearInerrupt(USB_OTG_GINTSTS_MMIS);

		if (OTG_GetInterruptState(USB_OTG_GINTSTS_OEPINT)) {
			epnum = 0U;

			/* Read in the device interrupt bits */
			ep_intr = USB_ReadDevAllOutEpInterrupt(USB_OTG_FS);

			while (ep_intr) {
				if (ep_intr & 0x1U) {
					epint = USB_ReadDevOutEPInterrupt(USB_OTG_FS, epnum);

					if (( epint & USB_OTG_DOEPINT_XFRC) == USB_OTG_DOEPINT_XFRC) {
						CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_XFRC);
						HAL_PCD_DataOutStageCallback(&hpcd_USB_OTG_FS, epnum);
					}

					if (( epint & USB_OTG_DOEPINT_STUP) == USB_OTG_DOEPINT_STUP) {
						/* Inform the upper layer that a setup packet is available */
						HAL_PCD_SetupStageCallback(&hpcd_USB_OTG_FS);
						CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_STUP);
					}

					if (( epint & USB_OTG_DOEPINT_OTEPDIS) == USB_OTG_DOEPINT_OTEPDIS) {
						CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_OTEPDIS);
					}
				}
				epnum++;
				ep_intr >>= 1U;
			}
		}

		if (OTG_GetInterruptState(USB_OTG_GINTSTS_IEPINT)) {
			/* Read in the device interrupt bits */
			ep_intr = USB_ReadDevAllInEpInterrupt(USB_OTG_FS);

			epnum = 0U;

			while (ep_intr) {
				/* In ITR */
				if (ep_intr & 0x1U) {
					epint = USB_ReadDevInEPInterrupt(USB_OTG_FS, epnum);

					if (( epint & USB_OTG_DIEPINT_XFRC) == USB_OTG_DIEPINT_XFRC) {
						fifoemptymsk = 0x1U << epnum;
						USBx_DEVICE->DIEPEMPMSK &= ~fifoemptymsk;
						CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_XFRC);
						HAL_PCD_DataInStageCallback(&hpcd_USB_OTG_FS, epnum);
					}

					if (( epint & USB_OTG_DIEPINT_TOC) == USB_OTG_DIEPINT_TOC)
						CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_TOC);

					if (( epint & USB_OTG_DIEPINT_ITTXFE) == USB_OTG_DIEPINT_ITTXFE)
						CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_ITTXFE);

					if (( epint & USB_OTG_DIEPINT_INEPNE) == USB_OTG_DIEPINT_INEPNE)
						CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_INEPNE);

					if(( epint & USB_OTG_DIEPINT_EPDISD) == USB_OTG_DIEPINT_EPDISD)
						CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_EPDISD);

					if(( epint & USB_OTG_DIEPINT_TXFE) == USB_OTG_DIEPINT_TXFE)
						PCD_WriteEmptyTxFifo(&hpcd_USB_OTG_FS , epnum);
				}
				epnum++;
				ep_intr >>= 1U;
			}
		}

		/* Handle Resume Interrupt */
		if (OTG_GetInterruptState(USB_OTG_GINTSTS_WKUINT)) {
			/* Clear the Remote Wake-up signalling */
			USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;
			HAL_PCD_ResumeCallback(&hpcd_USB_OTG_FS);
			OTG_ClearInerrupt(USB_OTG_GINTSTS_WKUINT);
		}

		/* Handle Suspend Interrupt */
		if (OTG_GetInterruptState(USB_OTG_GINTSTS_USBSUSP)) {
			if ((USBx_DEVICE->DSTS & USB_OTG_DSTS_SUSPSTS) == USB_OTG_DSTS_SUSPSTS)
				HAL_PCD_SuspendCallback(&hpcd_USB_OTG_FS);
			OTG_ClearInerrupt(USB_OTG_GINTSTS_USBSUSP);
		}

		/* Handle Reset Interrupt */
		if (OTG_GetInterruptState(USB_OTG_GINTSTS_USBRST)) {
			USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;
			USB_FlushTxFifo(USB_OTG_FS ,  0x10U);

			for (index = 0U; index < hpcd_USB_OTG_FS.Init.dev_endpoints ; index++) {
				USBx_INEP(index)->DIEPINT = 0xFFU;
				USBx_OUTEP(index)->DOEPINT = 0xFFU;
			}

			USBx_DEVICE->DAINT = 0xFFFFFFFFU;
			USBx_DEVICE->DAINTMSK |= 0x10001U;

			USBx_DEVICE->DOEPMSK |= (USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_EPDM);
			USBx_DEVICE->DIEPMSK |= (USB_OTG_DIEPMSK_TOM | USB_OTG_DIEPMSK_XFRCM | USB_OTG_DIEPMSK_EPDM);

			/* Set Default Address to 0 */
			USBx_DEVICE->DCFG &= ~USB_OTG_DCFG_DAD;

			/* setup EP0 to receive SETUP packets */
			USB_EP0_OutStart(USB_OTG_FS, (uint8_t *)hpcd_USB_OTG_FS.Setup);

			OTG_ClearInerrupt(USB_OTG_GINTSTS_USBRST);
		}

		/* Handle Enumeration done Interrupt */
		if (OTG_GetInterruptState(USB_OTG_GINTSTS_ENUMDNE)) {
			USB_ActivateSetup(USB_OTG_FS);
			USB_OTG_FS->GUSBCFG &= ~USB_OTG_GUSBCFG_TRDT;

			hpcd_USB_OTG_FS.Init.speed            = USB_OTG_SPEED_FULL;
			hpcd_USB_OTG_FS.Init.ep0_mps          = USB_OTG_FS_MAX_PACKET_SIZE ;
			USB_OTG_FS->GUSBCFG |= (uint32_t)((USBD_FS_TRDT_VALUE << 10U) & USB_OTG_GUSBCFG_TRDT);

			HAL_PCD_ResetCallback(&hpcd_USB_OTG_FS);

			OTG_ClearInerrupt(USB_OTG_GINTSTS_ENUMDNE);
		}

		/* Handle RxQLevel Interrupt */
		if (OTG_GetInterruptState(USB_OTG_GINTSTS_RXFLVL)) {
			USB_MASK_INTERRUPT(USB_OTG_FS, USB_OTG_GINTSTS_RXFLVL);
			temp = USB_OTG_FS->GRXSTSP;
			ep = &hpcd_USB_OTG_FS.OUT_ep[temp & USB_OTG_GRXSTSP_EPNUM];

			if (((temp & USB_OTG_GRXSTSP_PKTSTS) >> 17U) ==  STS_DATA_UPDT) {
				if ((temp & USB_OTG_GRXSTSP_BCNT) != 0U) {
					USB_ReadPacket(USB_OTG_FS, ep->xfer_buff, (temp & USB_OTG_GRXSTSP_BCNT) >> 4U);
					ep->xfer_buff += (temp & USB_OTG_GRXSTSP_BCNT) >> 4U;
					ep->xfer_count += (temp & USB_OTG_GRXSTSP_BCNT) >> 4U;
				}
			} else if (((temp & USB_OTG_GRXSTSP_PKTSTS) >> 17U) ==  STS_SETUP_UPDT) {
				USB_ReadPacket(USB_OTG_FS, (uint8_t *)hpcd_USB_OTG_FS.Setup, 8U);
				ep->xfer_count += (temp & USB_OTG_GRXSTSP_BCNT) >> 4U;
			}
			USB_UNMASK_INTERRUPT(USB_OTG_FS, USB_OTG_GINTSTS_RXFLVL);
		}

		// Handle SOF Interrupt
		if (OTG_GetInterruptState(USB_OTG_GINTSTS_SOF)) {
			HAL_PCD_SOFCallback(&hpcd_USB_OTG_FS);
			OTG_ClearInerrupt(USB_OTG_GINTSTS_SOF);
		}

		// Handle Incomplete ISO IN Interrupt
		if (OTG_GetInterruptState(USB_OTG_GINTSTS_IISOIXFR)) {
			HAL_PCD_ISOINIncompleteCallback(&hpcd_USB_OTG_FS, epnum);
			OTG_ClearInerrupt(USB_OTG_GINTSTS_IISOIXFR);
		}

		// Handle Incomplete ISO OUT Interrupt
		if (OTG_GetInterruptState(USB_OTG_GINTSTS_PXFR_INCOMPISOOUT)) {
			HAL_PCD_ISOOUTIncompleteCallback(&hpcd_USB_OTG_FS, epnum);
			OTG_ClearInerrupt(USB_OTG_GINTSTS_PXFR_INCOMPISOOUT);
		}

		// Handle Connection event Interrupt
		if (OTG_GetInterruptState(USB_OTG_GINTSTS_SRQINT)) {
			HAL_PCD_ConnectCallback(&hpcd_USB_OTG_FS);
			OTG_ClearInerrupt(USB_OTG_GINTSTS_SRQINT);
		}

		// Handle Disconnection event Interrupt
		if (OTG_GetInterruptState(USB_OTG_GINTSTS_OTGINT)) {
			temp = USB_OTG_FS->GOTGINT;

			if ((temp & USB_OTG_GOTGINT_SEDET) == USB_OTG_GOTGINT_SEDET)
				HAL_PCD_DisconnectCallback(&hpcd_USB_OTG_FS);
			USB_OTG_FS->GOTGINT = temp;
		}
	}
}

extern "C" void OTG_FS_IRQHandler(void) {
	USB.interrupt();
}

USB_Class USB;

