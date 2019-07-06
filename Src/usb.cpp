
#include "usb.h"

#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_ctlreq.h"
#include "usbd_ioreq.h"


USBD_HandleTypeDef hUsbDeviceFS;

USB_Class::USB_Class() {
	;
}

void USB_Class::init() {
	/* Init Device Library, add supported class and start the library. */
	if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != UsbStatus::USBD_OK) Error_Handler();
	if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != UsbStatus::USBD_OK) Error_Handler();
	if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != UsbStatus::USBD_OK) Error_Handler();
	start();
}

void USB_Class::start() {
	deviceConnect();
	millisecondDelay(3);
	enableGlobalInterrupt();
}

void USB_Class::setStall(uint8_t endpointAddress) {
	PCD_EPTypeDef *ep = nullptr;

	if (0x80U & endpointAddress) {
		ep = &hpcd_USB_OTG_FS.IN_ep[endpointAddress & 0x7FU];
	} else {
		ep = &hpcd_USB_OTG_FS.OUT_ep[endpointAddress];
	}

	ep->is_stall = true;
	ep->num      = endpointAddress & 0x7FU;
	ep->is_in    = endpointAddress & 0x80U;

	if (ep->is_in) {
		if (((inEndpoint(ep->num)->DIEPCTL) & USB_OTG_DIEPCTL_EPENA) == 0) {
			inEndpoint(ep->num)->DIEPCTL &= ~(USB_OTG_DIEPCTL_EPDIS);
		}
		inEndpoint(ep->num)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
	} else {
		if (((outEndpoint(ep->num)->DOEPCTL) & USB_OTG_DOEPCTL_EPENA) == 0) {
			outEndpoint(ep->num)->DOEPCTL &= ~(USB_OTG_DOEPCTL_EPDIS);
		}
		outEndpoint(ep->num)->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
	}

	if (ep->num == 0) endpoint0OutStart();
}

void USB_Class::startEndpoint0Xfer(USB_OTG_EPTypeDef *ep) {
	if (ep->is_in) { /* IN endpoint */
		/* Zero Length Packet? */
		if (ep->xfer_len == 0) {
			inEndpoint(ep->num)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);
			inEndpoint(ep->num)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (1 << 19));
			inEndpoint(ep->num)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);
		} else {
			/* Program the transfer size and packet count
			 * as follows: xfersize = N * maxpacket +
			 * short_packet pktcnt = N + (short_packet
			 * exist ? 1 : 0)
			 */
			inEndpoint(ep->num)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);
			inEndpoint(ep->num)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);

			if (ep->xfer_len > ep->maxpacket) ep->xfer_len = ep->maxpacket;

			inEndpoint(ep->num)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (1 << 19));
			inEndpoint(ep->num)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_XFRSIZ & ep->xfer_len);
		}

		// Enable the Tx FIFO Empty Interrupt for this EP
		if (ep->xfer_len > 0) {
			device()->DIEPEMPMSK |= 1 << (ep->num);
		}

		/* EP enable, IN data in FIFO */
		inEndpoint(ep->num)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
	} else /* OUT endpoint */ {
		/* Program the transfer size and packet count as follows:
		 * pktcnt = N
		 * xfersize = N * maxpacket
		 */
		outEndpoint(ep->num)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_XFRSIZ);
		outEndpoint(ep->num)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_PKTCNT);

		if (ep->xfer_len > 0) ep->xfer_len = ep->maxpacket;

		outEndpoint(ep->num)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1 << 19));
		outEndpoint(ep->num)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_XFRSIZ & (ep->maxpacket));

		/* EP enable */
		outEndpoint(ep->num)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
	}
}

void USB_Class::startEndpointXfer(USB_OTG_EPTypeDef *ep) {
	uint16_t pktcnt = 0;

	/* IN endpoint */
	if (ep->is_in) {
		/* Zero Length Packet? */
		if (ep->xfer_len == 0) {
			inEndpoint(ep->num)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);
			inEndpoint(ep->num)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (1 << 19)) ;
			inEndpoint(ep->num)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);
		} else {
			/* Program the transfer size and packet count
			 * as follows: xfersize = N * maxpacket +
			 * short_packet pktcnt = N + (short_packet
			 * exist ? 1 : 0)
			 */
			inEndpoint(ep->num)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);
			inEndpoint(ep->num)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);
			inEndpoint(ep->num)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (((ep->xfer_len + ep->maxpacket -1)/ ep->maxpacket) << 19)) ;
			inEndpoint(ep->num)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_XFRSIZ & ep->xfer_len);

			if (ep->type == EP_TYPE_ISOC) {
				inEndpoint(ep->num)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_MULCNT);
				inEndpoint(ep->num)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_MULCNT & (1 << 29));
			}
		}

		if (ep->type != EP_TYPE_ISOC) {
			/* Enable the Tx FIFO Empty Interrupt for this EP */
			if (ep->xfer_len > 0) {
				device()->DIEPEMPMSK |= 1 << ep->num;
			}
		}

		if (ep->type == EP_TYPE_ISOC) {
			if ((device()->DSTS & ( 1 << 8 )) == 0) {
				inEndpoint(ep->num)->DIEPCTL |= USB_OTG_DIEPCTL_SODDFRM;
			} else {
				inEndpoint(ep->num)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
			}
		}

		/* EP enable, IN data in FIFO */
		inEndpoint(ep->num)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);

		if (ep->type == EP_TYPE_ISOC) {
			writePacket(ep->xfer_buff, ep->num, ep->xfer_len);
		}
	} else /* OUT endpoint */ {
		/* Program the transfer size and packet count as follows:
		 * pktcnt = N
		 * xfersize = N * maxpacket
		 */
		outEndpoint(ep->num)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_XFRSIZ);
		outEndpoint(ep->num)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_PKTCNT);

		if (ep->xfer_len == 0) {
			outEndpoint(ep->num)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_XFRSIZ & ep->maxpacket);
			outEndpoint(ep->num)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1 << 19));
		} else {
			pktcnt = (ep->xfer_len + ep->maxpacket -1)/ ep->maxpacket;
			outEndpoint(ep->num)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (pktcnt << 19));
			outEndpoint(ep->num)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_XFRSIZ & (ep->maxpacket * pktcnt));
		}

		if (ep->type == EP_TYPE_ISOC) {
			if ((device()->DSTS & ( 1 << 8 )) == 0) {
				outEndpoint(ep->num)->DOEPCTL |= USB_OTG_DOEPCTL_SODDFRM;
			} else {
				outEndpoint(ep->num)->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
			}
		}
		/* EP enable */
		outEndpoint(ep->num)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
	}
}

void USB_Class::endpointTransmit(uint8_t ep_addr, uint8_t *pBuf, uint32_t len) {
	PCD_EPTypeDef *ep = NULL;

	ep = &hpcd_USB_OTG_FS.IN_ep[ep_addr & 0x7FU];

	/*setup and start the Xfer */
	ep->xfer_buff  = pBuf;
	ep->xfer_len   = len;
	ep->xfer_count = 0U;
	ep->is_in      = 1U;
	ep->num        = ep_addr & 0x7FU;

	if ((ep_addr & 0x7FU) == 0U) {
		startEndpoint0Xfer(ep);
	} else {
		startEndpointXfer(ep);
	}
}

void USB_Class::activateSetup() {

	/* Set the MPS of the IN EP based on the enumeration speed */
	inEndpoint(0)->DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ;

	if ((device()->DSTS & USB_OTG_DSTS_ENUMSPD) == DSTS_ENUMSPD_LS_PHY_6MHZ)
		inEndpoint(0)->DIEPCTL |= 3;

	device()->DCTL |= USB_OTG_DCTL_CGINAK;
}

void USB_Class::writePacket(uint8_t *source, uint32_t epnum, size_t length) {
	uint32_t count32b =  (length + 3) / 4;
	for (uint32_t index = 0; index < count32b; index++, source += 4) {
		*fifo(epnum) = *(reinterpret_cast<__packed uint32_t *>(source));
	}
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
	uint32_t temp = 0;

	// ensure that we are in device mode
	if (!getMode()) {

		// avoid spurious interrupt
		if (isInvalidInterrupt()) {
			return;
		}

		// incorrect mode, acknowledge the interrupt
		if (getInterruptState(USB_OTG_GINTSTS_MMIS)) {
			clearInerrupt(USB_OTG_GINTSTS_MMIS);
		}

		// OUT endpoint interrupt
		if (getInterruptState(USB_OTG_GINTSTS_OEPINT)) {
			onOut();
		}

		// IN endpoint interrupt
		if (getInterruptState(USB_OTG_GINTSTS_IEPINT)) {
			onIn();
		}

		/* Handle Resume Interrupt */
		if (getInterruptState(USB_OTG_GINTSTS_WKUINT)) {
			onResume();
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
			onReset();
			clearInerrupt(USB_OTG_GINTSTS_USBRST);
		}

		/* Handle Enumeration done Interrupt */
		if (getInterruptState(USB_OTG_GINTSTS_ENUMDNE)) {
			onEnumerationDone();
			clearInerrupt(USB_OTG_GINTSTS_ENUMDNE);
		}

		/* Handle RxQLevel Interrupt */
		if (getInterruptState(USB_OTG_GINTSTS_RXFLVL)) {
			onRxQLevel();
		}

		// Handle SOF Interrupt
		if (getInterruptState(USB_OTG_GINTSTS_SOF)) {
			onFrameStart();
			clearInerrupt(USB_OTG_GINTSTS_SOF);
		}

		// Handle Incomplete ISO IN Interrupt
		if (getInterruptState(USB_OTG_GINTSTS_IISOIXFR)) {
			onIsoInIncomplete();
			clearInerrupt(USB_OTG_GINTSTS_IISOIXFR);
		}

		// Handle Incomplete ISO OUT Interrupt
		if (getInterruptState(USB_OTG_GINTSTS_PXFR_INCOMPISOOUT)) {
			onIsoOutIncomlete();
			clearInerrupt(USB_OTG_GINTSTS_PXFR_INCOMPISOOUT);
		}

		// Handle Connection event Interrupt
		if (getInterruptState(USB_OTG_GINTSTS_SRQINT)) {
			onConnect();
			clearInerrupt(USB_OTG_GINTSTS_SRQINT);
		}

		// Handle Disconnection event Interrupt
		if (getInterruptState(USB_OTG_GINTSTS_OTGINT)) {
			temp = USB_OTG_FS->GOTGINT;
			if ((temp & USB_OTG_GOTGINT_SEDET) == USB_OTG_GOTGINT_SEDET) {
				onDisconnect();
			}
			USB_OTG_FS->GOTGINT = temp;
		}
	}
}

void USB_Class::onConnect() {
	;
}

void USB_Class::onDisconnect() {
	PCD_HandleTypeDef *hpcd = &hpcd_USB_OTG_FS;
	USBD_HandleTypeDef *pdev = (USBD_HandleTypeDef*) hpcd->pData;

	pdev->dev_state = USBD_STATE_DEFAULT;
	pdev->pClass->DeInit(pdev, pdev->dev_config);
}

void USB_Class::onIsoOutIncomlete() {
	;
}

void USB_Class::onIsoInIncomplete() {
	;
}

void USB_Class::onFrameStart() {
	;
}

void USB_Class::onRxQLevel() {
	maskInterrupt(USB_OTG_GINTSTS_RXFLVL);

	uint32_t temp = USB_OTG_FS->GRXSTSP;
	USB_OTG_EPTypeDef *ep = &hpcd_USB_OTG_FS.OUT_ep[temp & USB_OTG_GRXSTSP_EPNUM];

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

void USB_Class::onEnumerationDone() {
	activateSetup();

	hpcd_USB_OTG_FS.Init.speed    = USB_OTG_SPEED_FULL;
	hpcd_USB_OTG_FS.Init.ep0_mps  = USB_OTG_FS_MAX_PACKET_SIZE ;

	USB_OTG_FS->GUSBCFG &= ~USB_OTG_GUSBCFG_TRDT;
	USB_OTG_FS->GUSBCFG |= (uint32_t)((USBD_FS_TRDT_VALUE << 10U) & USB_OTG_GUSBCFG_TRDT);

	HAL_PCD_ResetCallback(&hpcd_USB_OTG_FS);
}

void USB_Class::onReset() {
	device()->DCTL &= ~USB_OTG_DCTL_RWUSIG;
	USB_FlushTxFifo(USB_OTG_FS ,  0x10U);

	for (uint32_t index = 0U; index < hpcd_USB_OTG_FS.Init.dev_endpoints; index++) {
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
}

void USB_Class::onSuspend() {
	;
}

void USB_Class::onResume() {
	// Clear the Remote Wake-up signalling
	device()->DCTL &= ~USB_OTG_DCTL_RWUSIG;

	HAL_PCD_ResumeCallback(&hpcd_USB_OTG_FS);
}

void USB_Class::onIn() {
	uint32_t epnum = 0;
	uint32_t fifoemptymsk = 0;

	// Read in the device interrupt bits
	uint32_t ep_intr = ReadDevAllInEpInterrupt();

	while (ep_intr) {
		if (ep_intr & 1) {

			uint32_t epint = ReadDevInEPInterrupt(epnum);

			if ((epint & USB_OTG_DIEPINT_XFRC) == USB_OTG_DIEPINT_XFRC) {
				fifoemptymsk = 0x1U << epnum;
				device()->DIEPEMPMSK &= ~fifoemptymsk;
				clearInEndpointInterrupt(epnum, USB_OTG_DIEPINT_XFRC);
				HAL_PCD_DataInStageCallback(&hpcd_USB_OTG_FS, epnum);
			}

			if ((epint & USB_OTG_DIEPINT_TOC) == USB_OTG_DIEPINT_TOC)
				clearInEndpointInterrupt(epnum, USB_OTG_DIEPINT_TOC);

			if ((epint & USB_OTG_DIEPINT_ITTXFE) == USB_OTG_DIEPINT_ITTXFE)
				clearInEndpointInterrupt(epnum, USB_OTG_DIEPINT_ITTXFE);

			if ((epint & USB_OTG_DIEPINT_INEPNE) == USB_OTG_DIEPINT_INEPNE)
				clearInEndpointInterrupt(epnum, USB_OTG_DIEPINT_INEPNE);

			if ((epint & USB_OTG_DIEPINT_EPDISD) == USB_OTG_DIEPINT_EPDISD)
				clearInEndpointInterrupt(epnum, USB_OTG_DIEPINT_EPDISD);

			if ((epint & USB_OTG_DIEPINT_TXFE) == USB_OTG_DIEPINT_TXFE)
				writeEmptyTxFifo(&hpcd_USB_OTG_FS , epnum);
		}

		epnum++;
		ep_intr >>= 1;
	}
}

void USB_Class::onOut() {
	uint32_t epnum = 0U;

	// Read in the device interrupt bits
	uint32_t ep_intr = ReadDevAllOutEpInterrupt();

	while (ep_intr) {
		if (ep_intr & 1) {

			uint32_t epint = ReadDevOutEPInterrupt(epnum);

			if (( epint & USB_OTG_DOEPINT_XFRC) == USB_OTG_DOEPINT_XFRC) {
				clearOutEndpointInterrupt(epnum, USB_OTG_DOEPINT_XFRC);
				HAL_PCD_DataOutStageCallback(&hpcd_USB_OTG_FS, epnum);
			}

			if (( epint & USB_OTG_DOEPINT_STUP) == USB_OTG_DOEPINT_STUP) {
				/* Inform the upper layer that a setup packet is available */
				onSetupStage();
				clearOutEndpointInterrupt(epnum, USB_OTG_DOEPINT_STUP);
			}

			if (( epint & USB_OTG_DOEPINT_OTEPDIS) == USB_OTG_DOEPINT_OTEPDIS) {
				clearOutEndpointInterrupt(epnum, USB_OTG_DOEPINT_OTEPDIS);
			}
		}

		epnum++;
		ep_intr >>= 1;
	}
}

void USB_Class::onSetupStage() {

	PCD_HandleTypeDef *hpcd = &hpcd_USB_OTG_FS;
	USBD_HandleTypeDef *pdev = (USBD_HandleTypeDef*) hpcd->pData;
	uint8_t *psetup = (uint8_t*) hpcd->Setup;

	// Parse setup request
	pdev->request.bmRequest     = psetup[0];
	pdev->request.bRequest      = psetup[1];
	pdev->request.wValue        = WORD(psetup[3], psetup[2]);
	pdev->request.wIndex        = WORD(psetup[5], psetup[4]);
	pdev->request.wLength       = WORD(psetup[7], psetup[6]);

	pdev->ep0_state             = USBD_EP0_SETUP;
	pdev->ep0_data_len          = pdev->request.wLength;

	switch (pdev->request.bmRequest & 0x1F) {
	case USB_REQ_RECIPIENT_DEVICE:
		onStdDevReq();
		break;

	case USB_REQ_RECIPIENT_INTERFACE:
		USBD_StdItfReq(pdev, &pdev->request);
		break;

	case USB_REQ_RECIPIENT_ENDPOINT:
		USBD_StdEPReq(pdev, &pdev->request);
		break;

	default:
		USBD_LL_StallEP(pdev , pdev->request.bmRequest & 0x80);
		break;
	}
}

void USB_Class::onStdDevReq() {
	PCD_HandleTypeDef *hpcd = &hpcd_USB_OTG_FS;
	USBD_HandleTypeDef *pdev = (USBD_HandleTypeDef*) hpcd->pData;

	switch (pdev->request.bRequest) {
	case USB_REQ_GET_DESCRIPTOR:
		onGetDescriptor();
		break;

	case USB_REQ_SET_ADDRESS:
		onSetAddress();
		break;

	case USB_REQ_SET_CONFIGURATION:
		onSetConfig();
		break;

	case USB_REQ_GET_CONFIGURATION:
		onGetConfig();
		break;

	case USB_REQ_GET_STATUS:
		onGetStatus();
		break;

	case USB_REQ_SET_FEATURE:
		onSetFeature();
		break;

	case USB_REQ_CLEAR_FEATURE:
		onClearFeature();
		break;

	default:
		controllError();
	}
}

void USB_Class::onGetDescriptor() {
	PCD_HandleTypeDef *hpcd = &hpcd_USB_OTG_FS;
	USBD_HandleTypeDef *pdev = (USBD_HandleTypeDef*) hpcd->pData;

	uint16_t len;
	uint8_t *pbuf;

	switch (pdev->request.wValue >> 8) {
#if (USBD_LPM_ENABLED == 1)
	case USB_DESC_TYPE_BOS:
		pbuf = pdev->pDesc->GetBOSDescriptor(pdev->dev_speed, &len);
		break;
#endif
	case USB_DESC_TYPE_DEVICE:
		pbuf = pdev->pDesc->GetDeviceDescriptor(pdev->dev_speed, &len);
		break;

	case USB_DESC_TYPE_CONFIGURATION:
		if (pdev->dev_speed == UsbSpeed::USBD_SPEED_HIGH ) {
			pbuf   = (uint8_t *)pdev->pClass->GetHSConfigDescriptor(&len);
			pbuf[1] = USB_DESC_TYPE_CONFIGURATION;
		} else {
			pbuf   = (uint8_t *)pdev->pClass->GetFSConfigDescriptor(&len);
			pbuf[1] = USB_DESC_TYPE_CONFIGURATION;
		}
		break;

	case USB_DESC_TYPE_STRING:
		switch ((uint8_t)(pdev->request.wValue)) {
		case USBD_IDX_LANGID_STR:
			pbuf = pdev->pDesc->GetLangIDStrDescriptor(pdev->dev_speed, &len);
			break;

		case USBD_IDX_MFC_STR:
			pbuf = pdev->pDesc->GetManufacturerStrDescriptor(pdev->dev_speed, &len);
			break;

		case USBD_IDX_PRODUCT_STR:
			pbuf = pdev->pDesc->GetProductStrDescriptor(pdev->dev_speed, &len);
			break;

		case USBD_IDX_SERIAL_STR:
			pbuf = pdev->pDesc->GetSerialStrDescriptor(pdev->dev_speed, &len);
			break;

		case USBD_IDX_CONFIG_STR:
			pbuf = pdev->pDesc->GetConfigurationStrDescriptor(pdev->dev_speed, &len);
			break;

		case USBD_IDX_INTERFACE_STR:
			pbuf = pdev->pDesc->GetInterfaceStrDescriptor(pdev->dev_speed, &len);
			break;

		default:
#if (USBD_SUPPORT_USER_STRING == 1)
	pbuf = pdev->pClass->GetUsrStrDescriptor(pdev, (req->wValue) , &len);
	break;
#else
	controllError();
	return;
#endif
		}
		break;
		case USB_DESC_TYPE_DEVICE_QUALIFIER:

			if (pdev->dev_speed == UsbSpeed::USBD_SPEED_HIGH  ) {
				pbuf   = (uint8_t *)pdev->pClass->GetDeviceQualifierDescriptor(&len);
				break;
			} else {
				controllError();
				return;
			}

		case USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION:
			if (pdev->dev_speed == UsbSpeed::USBD_SPEED_HIGH  ) {
				pbuf   = (uint8_t *)pdev->pClass->GetOtherSpeedConfigDescriptor(&len);
				pbuf[1] = USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION;
				break;
			} else {
				controllError();
				return;
			}

		default:
			controllError();
			return;
	}

	if ((len != 0)&& (pdev->request.wLength != 0)) {
		len = MIN(len , pdev->request.wLength);
		USBD_CtlSendData (pdev, pbuf, len);
	}

}

void USB_Class::onSetAddress() {
	PCD_HandleTypeDef *hpcd = &hpcd_USB_OTG_FS;
	USBD_HandleTypeDef *pdev = (USBD_HandleTypeDef*) hpcd->pData;

	uint8_t  dev_addr;

	if ((pdev->request.wIndex == 0) && (pdev->request.wLength == 0)) {
		dev_addr = (uint8_t)(pdev->request.wValue) & 0x7F;

		if (pdev->dev_state == USBD_STATE_CONFIGURED) {
			controllError();
		} else {
			pdev->dev_address = dev_addr;
			USBD_LL_SetUSBAddress(pdev, dev_addr);
			sendControlStatus();

			if (dev_addr != 0) {
				pdev->dev_state  = USBD_STATE_ADDRESSED;
			} else {
				pdev->dev_state  = USBD_STATE_DEFAULT;
			}
		}
	} else {
		controllError();
	}
}

void USB_Class::onSetConfig() {
	PCD_HandleTypeDef *hpcd = &hpcd_USB_OTG_FS;
	USBD_HandleTypeDef *pdev = (USBD_HandleTypeDef*) hpcd->pData;

	static uint8_t  cfgidx = (uint8_t) (pdev->request.wValue);

	if (cfgidx > USBD_MAX_NUM_CONFIGURATION ) {
		controllError();
	} else {
		switch (pdev->dev_state) {
		case USBD_STATE_ADDRESSED:
			if (cfgidx) {
				pdev->dev_config = cfgidx;
				pdev->dev_state = USBD_STATE_CONFIGURED;
				if (USBD_SetClassConfig(pdev , cfgidx) == UsbStatus::USBD_FAIL) {
					controllError();
					return;
				}
				sendControlStatus();
			} else {
				sendControlStatus();
			}
			break;

		case USBD_STATE_CONFIGURED:
			if (cfgidx == 0) {
				pdev->dev_state = USBD_STATE_ADDRESSED;
				pdev->dev_config = cfgidx;
				USBD_ClrClassConfig(pdev , cfgidx);
				sendControlStatus();

			} else  if (cfgidx != pdev->dev_config) {
				/* Clear old configuration */
				USBD_ClrClassConfig(pdev , pdev->dev_config);

				/* set new configuration */
				pdev->dev_config = cfgidx;
				if (USBD_SetClassConfig(pdev , cfgidx) == UsbStatus::USBD_FAIL) {
					controllError();
					return;
				}
				sendControlStatus();
			} else {
				sendControlStatus();
			}
			break;

		default:
			controllError();
			break;
		}
	}
}

void USB_Class::onGetConfig() {
	PCD_HandleTypeDef *hpcd = &hpcd_USB_OTG_FS;
	USBD_HandleTypeDef *pdev = (USBD_HandleTypeDef*) hpcd->pData;

	if (pdev->request.wLength != 1) {
		controllError();
	} else {
		switch (pdev->dev_state) {
		case USBD_STATE_ADDRESSED:
			pdev->dev_default_config = 0;
			USBD_CtlSendData (pdev, (uint8_t *) &pdev->dev_default_config, 1);
			break;

		case USBD_STATE_CONFIGURED:
			USBD_CtlSendData (pdev, (uint8_t *) &pdev->dev_config, 1);
			break;

		default:
			controllError();
			break;
		}
	}
}

void USB_Class::onGetStatus() {
	PCD_HandleTypeDef *hpcd = &hpcd_USB_OTG_FS;
	USBD_HandleTypeDef *pdev = (USBD_HandleTypeDef*) hpcd->pData;

	switch (pdev->dev_state) {
	case USBD_STATE_ADDRESSED:
	case USBD_STATE_CONFIGURED:

#if ( USBD_SELF_POWERED == 1)
		pdev->dev_config_status = USB_CONFIG_SELF_POWERED;
#else
		pdev->dev_config_status = 0;
#endif

		if (pdev->dev_remote_wakeup) {
			pdev->dev_config_status |= USB_CONFIG_REMOTE_WAKEUP;
		}

		USBD_CtlSendData (pdev, (uint8_t *) &pdev->dev_config_status, 2);
		break;

	default:
		controllError();
		break;
	}
}

void USB_Class::onSetFeature() {
	PCD_HandleTypeDef *hpcd = &hpcd_USB_OTG_FS;
	USBD_HandleTypeDef *pdev = (USBD_HandleTypeDef*) hpcd->pData;

	if (pdev->request.wValue == USB_FEATURE_REMOTE_WAKEUP) {
		pdev->dev_remote_wakeup = 1;
		pdev->pClass->Setup (pdev, &pdev->request);
		sendControlStatus();
	}
}

void USB_Class::onClearFeature() {
	PCD_HandleTypeDef *hpcd = &hpcd_USB_OTG_FS;
	USBD_HandleTypeDef *pdev = (USBD_HandleTypeDef*) hpcd->pData;

	switch (pdev->dev_state) {
	case USBD_STATE_ADDRESSED:
	case USBD_STATE_CONFIGURED:
		if (pdev->request.wValue == USB_FEATURE_REMOTE_WAKEUP) {
			pdev->dev_remote_wakeup = 0;
			pdev->pClass->Setup (pdev, &pdev->request);
			sendControlStatus();
		}
		break;

	default:
		controllError();
		break;
	}
}

extern "C" void OTG_FS_IRQHandler(void) {
	USB.interrupt();
}

USB_Class USB;

