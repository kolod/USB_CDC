
#include "usb.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

USB_Class::USB_Class() {
	;
}

HAL_StatusTypeDef USB_Class::writePacket(uint8_t *source, uint32_t epnum, size_t length) {
	uint32_t count32b =  (length + 3) / 4;

	for (uint32_t index = 0; index < count32b; index++, source += 4)
		*fifo(epnum) = *(reinterpret_cast<__packed uint32_t *>(source));

	return HAL_OK;
}

HAL_StatusTypeDef USB_Class::writeEmptyTxFifo(PCD_HandleTypeDef *hpcd, uint32_t epnum) {

	USB_OTG_EPTypeDef *ep = NULL;
	size_t   len = 0;
	uint32_t len32b = 0U;
	uint32_t fifoemptymsk = 0U;

	ep = &hpcd->IN_ep[epnum];
	len = ep->xfer_len - ep->xfer_count;

	if (len > ep->maxpacket) len = ep->maxpacket;
	len32b = (len + 3U) / 4U;

	while ((inEndpoint(epnum)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV) > len32b &&
			ep->xfer_count < ep->xfer_len &&
			ep->xfer_len != 0U)
	{
		/* Write the FIFO */
		len = ep->xfer_len - ep->xfer_count;

		if ((uint32_t)len > ep->maxpacket) len = ep->maxpacket;
		len32b = (len + 3U) / 4U;

		writePacket(ep->xfer_buff, epnum, len);

		ep->xfer_buff  += len;
		ep->xfer_count += len;
	}

	if (len <= 0) {
		fifoemptymsk = 0x01U << epnum;
		device()->DIEPEMPMSK &= ~fifoemptymsk;
	}

	return HAL_OK;
}

void USB_Class::interrupt() {

	uint32_t index = 0U, ep_intr = 0U, epint = 0U, epnum = 0U;
	uint32_t fifoemptymsk = 0U, temp = 0U;
	USB_OTG_EPTypeDef *ep = NULL;

	// ensure that we are in device mode
	if (!getMode()) {

		// avoid spurious interrupt
		if (isInvalidInterrupt()) return;

		/* incorrect mode, acknowledge the interrupt */
		if (getInterruptState(USB_OTG_GINTSTS_MMIS))
			clearInerrupt(USB_OTG_GINTSTS_MMIS);

		if (getInterruptState(USB_OTG_GINTSTS_OEPINT)) {
			epnum = 0U;

			/* Read in the device interrupt bits */
			ep_intr = USB_ReadDevAllOutEpInterrupt(USB_OTG_FS);

			while (ep_intr) {
				if (ep_intr & 0x1U) {
					epint = USB_ReadDevOutEPInterrupt(USB_OTG_FS, epnum);

					if (( epint & USB_OTG_DOEPINT_XFRC) == USB_OTG_DOEPINT_XFRC) {
						clearOutEndpointInterrupt(epnum, USB_OTG_DOEPINT_XFRC);
						HAL_PCD_DataOutStageCallback(&hpcd_USB_OTG_FS, epnum);
					}

					if (( epint & USB_OTG_DOEPINT_STUP) == USB_OTG_DOEPINT_STUP) {
						/* Inform the upper layer that a setup packet is available */
						HAL_PCD_SetupStageCallback(&hpcd_USB_OTG_FS);
						clearOutEndpointInterrupt(epnum, USB_OTG_DOEPINT_STUP);
					}

					if (( epint & USB_OTG_DOEPINT_OTEPDIS) == USB_OTG_DOEPINT_OTEPDIS) {
						clearOutEndpointInterrupt(epnum, USB_OTG_DOEPINT_OTEPDIS);
					}
				}
				epnum++;
				ep_intr >>= 1U;
			}
		}

		if (getInterruptState(USB_OTG_GINTSTS_IEPINT)) {
			/* Read in the device interrupt bits */
			ep_intr = USB_ReadDevAllInEpInterrupt(USB_OTG_FS);

			epnum = 0U;

			while (ep_intr) {
				/* In ITR */
				if (ep_intr & 0x1U) {
					epint = USB_ReadDevInEPInterrupt(USB_OTG_FS, epnum);

					if (( epint & USB_OTG_DIEPINT_XFRC) == USB_OTG_DIEPINT_XFRC) {
						fifoemptymsk = 0x1U << epnum;
						device()->DIEPEMPMSK &= ~fifoemptymsk;
						clearInEndpointInterrupt(epnum, USB_OTG_DIEPINT_XFRC);
						HAL_PCD_DataInStageCallback(&hpcd_USB_OTG_FS, epnum);
					}

					if (( epint & USB_OTG_DIEPINT_TOC) == USB_OTG_DIEPINT_TOC)
						clearInEndpointInterrupt(epnum, USB_OTG_DIEPINT_TOC);

					if (( epint & USB_OTG_DIEPINT_ITTXFE) == USB_OTG_DIEPINT_ITTXFE)
						clearInEndpointInterrupt(epnum, USB_OTG_DIEPINT_ITTXFE);

					if (( epint & USB_OTG_DIEPINT_INEPNE) == USB_OTG_DIEPINT_INEPNE)
						clearInEndpointInterrupt(epnum, USB_OTG_DIEPINT_INEPNE);

					if(( epint & USB_OTG_DIEPINT_EPDISD) == USB_OTG_DIEPINT_EPDISD)
						clearInEndpointInterrupt(epnum, USB_OTG_DIEPINT_EPDISD);

					if(( epint & USB_OTG_DIEPINT_TXFE) == USB_OTG_DIEPINT_TXFE)
						writeEmptyTxFifo(&hpcd_USB_OTG_FS , epnum);
				}
				epnum++;
				ep_intr >>= 1U;
			}
		}

		/* Handle Resume Interrupt */
		if (getInterruptState(USB_OTG_GINTSTS_WKUINT)) {
			/* Clear the Remote Wake-up signalling */
			device()->DCTL &= ~USB_OTG_DCTL_RWUSIG;
			HAL_PCD_ResumeCallback(&hpcd_USB_OTG_FS);
			clearInerrupt(USB_OTG_GINTSTS_WKUINT);
		}

		/* Handle Suspend Interrupt */
		if (getInterruptState(USB_OTG_GINTSTS_USBSUSP)) {
			if ((device()->DSTS & USB_OTG_DSTS_SUSPSTS) == USB_OTG_DSTS_SUSPSTS)
				HAL_PCD_SuspendCallback(&hpcd_USB_OTG_FS);
			clearInerrupt(USB_OTG_GINTSTS_USBSUSP);
		}

		/* Handle Reset Interrupt */
		if (getInterruptState(USB_OTG_GINTSTS_USBRST)) {
			device()->DCTL &= ~USB_OTG_DCTL_RWUSIG;
			USB_FlushTxFifo(USB_OTG_FS ,  0x10U);

			for (index = 0U; index < hpcd_USB_OTG_FS.Init.dev_endpoints ; index++) {
				inEndpoint(index)->DIEPINT = 0xFFU;
				outEndpoint(index)->DOEPINT = 0xFFU;
			}

			device()->DAINT = 0xFFFFFFFFU;
			device()->DAINTMSK |= 0x10001U;

			device()->DOEPMSK |= (USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_EPDM);
			device()->DIEPMSK |= (USB_OTG_DIEPMSK_TOM | USB_OTG_DIEPMSK_XFRCM | USB_OTG_DIEPMSK_EPDM);

			/* Set Default Address to 0 */
			device()->DCFG &= ~USB_OTG_DCFG_DAD;

			/* setup EP0 to receive SETUP packets */
			USB_EP0_OutStart(USB_OTG_FS, (uint8_t *)hpcd_USB_OTG_FS.Setup);

			clearInerrupt(USB_OTG_GINTSTS_USBRST);
		}

		/* Handle Enumeration done Interrupt */
		if (getInterruptState(USB_OTG_GINTSTS_ENUMDNE)) {
			USB_ActivateSetup(USB_OTG_FS);
			USB_OTG_FS->GUSBCFG &= ~USB_OTG_GUSBCFG_TRDT;

			hpcd_USB_OTG_FS.Init.speed            = USB_OTG_SPEED_FULL;
			hpcd_USB_OTG_FS.Init.ep0_mps          = USB_OTG_FS_MAX_PACKET_SIZE ;
			USB_OTG_FS->GUSBCFG |= (uint32_t)((USBD_FS_TRDT_VALUE << 10U) & USB_OTG_GUSBCFG_TRDT);

			HAL_PCD_ResetCallback(&hpcd_USB_OTG_FS);

			clearInerrupt(USB_OTG_GINTSTS_ENUMDNE);
		}

		/* Handle RxQLevel Interrupt */
		if (getInterruptState(USB_OTG_GINTSTS_RXFLVL)) {
			maskInterrupt(USB_OTG_GINTSTS_RXFLVL);
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
			unmaskInterrupt(USB_OTG_GINTSTS_RXFLVL);
		}

		// Handle SOF Interrupt
		if (getInterruptState(USB_OTG_GINTSTS_SOF)) {
			HAL_PCD_SOFCallback(&hpcd_USB_OTG_FS);
			clearInerrupt(USB_OTG_GINTSTS_SOF);
		}

		// Handle Incomplete ISO IN Interrupt
		if (getInterruptState(USB_OTG_GINTSTS_IISOIXFR)) {
			HAL_PCD_ISOINIncompleteCallback(&hpcd_USB_OTG_FS, epnum);
			clearInerrupt(USB_OTG_GINTSTS_IISOIXFR);
		}

		// Handle Incomplete ISO OUT Interrupt
		if (getInterruptState(USB_OTG_GINTSTS_PXFR_INCOMPISOOUT)) {
			HAL_PCD_ISOOUTIncompleteCallback(&hpcd_USB_OTG_FS, epnum);
			clearInerrupt(USB_OTG_GINTSTS_PXFR_INCOMPISOOUT);
		}

		// Handle Connection event Interrupt
		if (getInterruptState(USB_OTG_GINTSTS_SRQINT)) {
			HAL_PCD_ConnectCallback(&hpcd_USB_OTG_FS);
			clearInerrupt(USB_OTG_GINTSTS_SRQINT);
		}

		// Handle Disconnection event Interrupt
		if (getInterruptState(USB_OTG_GINTSTS_OTGINT)) {
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

