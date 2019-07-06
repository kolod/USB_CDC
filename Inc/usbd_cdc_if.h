
#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_cdc.h"


/** CDC Interface callback. */
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/** @defgroup USBD_CDC_IF_Exported_FunctionsPrototype USBD_CDC_IF_Exported_FunctionsPrototype
  * @brief Public functions declaration.
  * @{
  */

UsbStatus CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

#ifdef __cplusplus
}
#endif

#endif // __USBD_CDC_IF_H__
