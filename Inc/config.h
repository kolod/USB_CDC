
#ifndef CONFIG_H_
#define CONFIG_H_

#define F_CPU                     72000000UL

#define RTC_STURTUP_TIMEOUT       50000
#define RTC_STURTUP_COUNT         5

#define USB_VBUS_Pin              GPIO_PIN_9
#define USB_VBUS_GPIO_Port        GPIOA
#define USB_EN_Pin                GPIO_PIN_10
#define USB_EN_GPIO_Port          GPIOA
#define USB_DM_Pin                GPIO_PIN_11
#define USB_DM_GPIO_Port          GPIOA
#define USB_DP_Pin                GPIO_PIN_12
#define USB_DP_GPIO_Port          GPIOA
#define SWDIO_Pin                 GPIO_PIN_13
#define SWDIO_GPIO_Port           GPIOA
#define SWCLK_Pin                 GPIO_PIN_14
#define SWCLK_GPIO_Port           GPIOA

#endif /* CONFIG_H_ */
