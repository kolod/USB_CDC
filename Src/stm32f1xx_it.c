

#include "main.h"
#include "stm32f1xx_it.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USB OTG FS global interrupt.
  */
void OTG_FS_IRQHandler(void) {
	HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}
