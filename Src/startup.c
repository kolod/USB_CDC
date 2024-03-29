// Copyright (c) 2018-2019 Alexandr Kolodkin <alexandr.kolodkin@gmail.com>
// All rights reserved.

#define HSE_STARTUP_TIMEOUT    ((uint32_t) 0x00001000)
#define VECT_TAB_OFFSET        ((uint32_t)          0)

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f1xx.h"

#include "utils.h"
#include "config.h"

// Linker generated Symbols
extern uint32_t _sidata;    // start address for the initialization values of the .data section.
extern uint32_t _sdata;     // start address for the .data section.
extern uint32_t _edata;     // end address for the .data section.
extern uint32_t _sbss;      // start address for the .bss section.
extern uint32_t _ebss;      // end address for the .bss section.
extern uint32_t __StackTop; //

// Exception / Interrupt Handler Function Prototype
typedef void( *pFunc )( void );

// External References
extern void main();
extern void __libc_init_array();
extern void __assert(const char *file, int line, const char *failedexpr);

volatile uint32_t uptime;

static inline bool SetSysClock() {

	// SYSCLK, HCLK, PCLK2 and PCLK1 configuration
	SET_BIT(RCC->CR, RCC_CR_HSEON);

	// Wait till HSE is ready and if Time out is reached exit
	volatile uint32_t timeout = HSE_STARTUP_TIMEOUT;
	while (!READ_BIT(RCC->CR, RCC_CR_HSERDY)) {
		if (--timeout == 0) return false;
	}

	SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE);                             // Enable Prefetch Buffer
	MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2);    // Flash 2 wait state
	SET_BIT(RCC->CFGR, RCC_CFGR_HPRE_DIV1);                            // HCLK = SYSCLK
	SET_BIT(RCC->CFGR, RCC_CFGR_PPRE2_DIV1);                           // PCLK2 = HCLK
	SET_BIT(RCC->CFGR, RCC_CFGR_PPRE1_DIV2);                           // PCLK1 = HCLK

	// PLL configuration: PLLCLK = PREDIV1 * 9 = 72 MHz
	MODIFY_REG(RCC->CFGR,
		RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL,
		RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLSRC
	);

	SET_BIT(RCC->CR, RCC_CR_PLLON);                                    // Enable PLL
	while(!READ_BIT(RCC->CR, RCC_CR_PLLRDY));                          // Wait till PLL is ready
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);               // Select PLL as system clock source
	while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);     // Wait till PLL is used as system clock source

	// Enable clocking
	SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN | RCC_AHBENR_CRCEN | RCC_AHBENR_FLITFEN |
		RCC_AHBENR_OTGFSEN | RCC_AHBENR_SRAMEN);
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN | RCC_APB1ENR_PWREN | RCC_APB1ENR_DACEN |
		RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN | RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM7EN);
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN |
		RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_TIM1EN |
		RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN);

	// Pins remapping
	SET_BIT(AFIO->MAPR, AFIO_MAPR_I2C1_REMAP | AFIO_MAPR_USART1_REMAP | AFIO_MAPR_SWJ_CFG_JTAGDISABLE);

	return true;
}

static inline bool MySystemInit (void) {

	// Reset the RCC clock configuration to the default reset state(for debug purpose)
	SET_BIT(RCC->CR, RCC_CR_HSION);
	CLEAR_BIT(RCC->CFGR, RCC_CFGR_HPRE | RCC_CFGR_SW | RCC_CFGR_PPRE1 | RCC_CFGR_ADCPRE | RCC_CFGR_MCO);
	CLEAR_BIT(RCC->CR, RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON);
	CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);
	CLEAR_BIT(RCC->CFGR, RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL | RCC_CFGR_OTGFSPRE);
	CLEAR_BIT(RCC->CR, RCC_CR_PLL2ON | RCC_CR_PLL3ON);

	//  Disable all interrupts and clear pending bits
	WRITE_REG(RCC->CIR, RCC_CIR_LSIRDYC | RCC_CIR_LSERDYC | RCC_CIR_HSIRDYC | RCC_CIR_HSERDYC |
		RCC_CIR_PLLRDYC | RCC_CIR_PLL2RDYC | RCC_CIR_PLL3RDYC | RCC_CIR_CSSC);

	// Reset CFGR2 register
	WRITE_REG(RCC->CFGR2, 0);

	// Vector Table Relocation in Internal FLASH.
	WRITE_REG(SCB->VTOR, FLASH_BASE | VECT_TAB_OFFSET);

	/* Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers */
	/* Configure the Flash Latency cycles and enable prefetch buffer */

	return SetSysClock();
}

// Reset Handler called on controller reset
void Reset_Handler(void) {
	uint32_t *source;
	uint32_t *destination;

	// Копируем данные из флешки в память
	source = &_sidata;
	for (destination = &_sdata; destination < &_edata; *(destination++) = *(source++));

	// Обнуляем
	for (destination = &_sbss; destination < &_ebss; *(destination++) = 0);

	// Настраиваем частоты тактования
	if (!MySystemInit()) __assert(__FILE__, __LINE__, "SystemInit failed");

	// Initialize GPIO
//	GPIO_PIN_SET(USB_EN_GPIO_Port, USB_EN_Pin);

	//             11111100                00000000
	//             54321098                76543210
	GPIOA->CRH = 0x88844343;GPIOA->CRL = 0x00000000;
	GPIOB->CRH = 0x333333FF;GPIOB->CRL = 0x4B088320;
	GPIOC->CRH = 0x44088833;GPIOC->CRL = 0x33000000;
	GPIOD->CRH = 0x44444444;GPIOD->CRL = 0x44444844;

	__libc_init_array();

	main();
	for (;;);
}


// Default Handlers for Exceptions / Interrupts
void Default_Handler(void) {
	__assert(__FILE__, __LINE__, "Unhandled interrupt");
}

void Default_NMI_Handler(void) {}

void Default_HardFault_Handler(void) {
	__assert(__FILE__, __LINE__, "HardFault interrupt");
	for (;;);
}

void Default_MemManage_Handler(void) {
	__assert(__FILE__, __LINE__, "MemManage interrupt");
	for (;;);
}

void Default_BusFault_Handler(void) {
	__assert(__FILE__, __LINE__, "BusFault interrupt");
	for (;;);
}

void Default_UsageFault_Handler(void) {
	__assert(__FILE__, __LINE__, "UsageFault interrupt");
	for (;;);
}

void Default_SVC_Handler(void) {}

void Default_DebugMon_Handler(void) {}

void Default_PendSV_Handler(void) {}

void Default_SysTick_Handler(void) {
	uptime++;

#ifdef USE_HAL_DRIVER
	HAL_IncTick();
#endif

}
// Exception / Interrupt Handler

// Cortex-M3 Processor Exceptions
void NMI_Handler                     (void) __attribute__ ((weak, alias("Default_NMI_Handler")));
void HardFault_Handler               (void) __attribute__ ((weak, alias("Default_HardFault_Handler")));
void MemManage_Handler               (void) __attribute__ ((weak, alias("Default_MemManage_Handler")));
void BusFault_Handler                (void) __attribute__ ((weak, alias("Default_BusFault_Handler")));
void UsageFault_Handler              (void) __attribute__ ((weak, alias("Default_UsageFault_Handler")));
void SVC_Handler                     (void) __attribute__ ((weak, alias("Default_SVC_Handler")));
void DebugMon_Handler                (void) __attribute__ ((weak, alias("Default_DebugMon_Handler")));
void PendSV_Handler                  (void) __attribute__ ((weak, alias("Default_PendSV_Handler")));
void SysTick_Handler                 (void) __attribute__ ((weak, alias("Default_SysTick_Handler")));

// STM32F105RB Specific Interrupts
void WWDG_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void PVD_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void TAMPER_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void RTC_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void FLASH_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void RCC_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI0_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI1_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI2_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI3_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI4_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel1_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel2_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel3_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel4_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel5_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel6_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel7_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC1_2_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN1_TX_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN1_RX0_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN1_RX1_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN1_SCE_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI9_5_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_BRK_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_UP_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_TRG_COM_IRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM2_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM3_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM4_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_EV_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_ER_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C2_EV_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C2_ER_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI1_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI2_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void USART1_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void USART2_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void USART3_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI15_10_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void RTCAlarm_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void OTG_FS_WKUP_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM5_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI3_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void UART4_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void UART5_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM6_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM7_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel1_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel2_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel3_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel4_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel5_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void ETH_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void ETH_WKUP_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN2_TX_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN2_RX0_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN2_RX1_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN2_SCE_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void OTG_FS_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));

// Exception / Interrupt Vector table

const pFunc __Vectors[] __attribute__ ((section(".isr_vector"))) = {
	// Cortex-M0+ Exceptions Handler
	(pFunc) &__StackTop,                      // -16:  Initial Stack Pointer
	Reset_Handler,                            // -15:  Reset Handler
	NMI_Handler,                              // -14:  NMI Handler
	HardFault_Handler,                        // -13:  Hard Fault Handler
	MemManage_Handler,                        // -12:  Memory managment Handler
	BusFault_Handler,                         // -11:  Bus Fault Handler
	UsageFault_Handler,                       // -10:  Usage Fault Handler
	0,                                        //  -9:  Reserved
	0,                                        //  -8:  Reserved
	0,                                        //  -7:  Reserved
	0,                                        //  -6:  Reserved
	SVC_Handler,                              //  -5:  SVCall Handler
	DebugMon_Handler,                         //  -4:  Debug Monitor Handler
	0,                                        //  -3:  Reserved
	PendSV_Handler,                           //  -2:  PendSV Handler
	SysTick_Handler,                          //  -1:  SysTick Handler

	// External interrupts
	WWDG_IRQHandler,                          //   0:  Watchdog Timer
	PVD_IRQHandler,                           //   1:  PVD through EXTI Line detect
	TAMPER_IRQHandler,                        //
	RTC_IRQHandler,                           //
	FLASH_IRQHandler,                         //
	RCC_IRQHandler,                           //
	EXTI0_IRQHandler,                         //
	EXTI1_IRQHandler,                         //
	EXTI2_IRQHandler,                         //
	EXTI3_IRQHandler,                         //
	EXTI4_IRQHandler,                         //
	DMA1_Channel1_IRQHandler,                 //
	DMA1_Channel2_IRQHandler,                 //
	DMA1_Channel3_IRQHandler,                 //
	DMA1_Channel4_IRQHandler,                 //
	DMA1_Channel5_IRQHandler,                 //
	DMA1_Channel6_IRQHandler,                 //
	DMA1_Channel7_IRQHandler,                 //
	ADC1_2_IRQHandler,                        //
	CAN1_TX_IRQHandler,                       //
	CAN1_RX0_IRQHandler,                      //
	CAN1_RX1_IRQHandler,                      //
	CAN1_SCE_IRQHandler,                      //
	EXTI9_5_IRQHandler,                       //
	TIM1_BRK_IRQHandler,                      //
	TIM1_UP_IRQHandler,                       //
	TIM1_TRG_COM_IRQHandler,                  //
	TIM1_CC_IRQHandler,                       //
	TIM2_IRQHandler,                          //
	TIM3_IRQHandler,                          //
	TIM4_IRQHandler,                          //
	I2C1_EV_IRQHandler,                       //
	I2C1_ER_IRQHandler,                       //
	I2C2_EV_IRQHandler,                       //
	I2C2_ER_IRQHandler,                       //
	SPI1_IRQHandler,                          //
	SPI2_IRQHandler,                          //
	USART1_IRQHandler,                        //
	USART2_IRQHandler,                        //
	USART3_IRQHandler,                        //
	EXTI15_10_IRQHandler,                     //
	RTCAlarm_IRQHandler,                      //
	OTG_FS_WKUP_IRQHandler,                   //
	0,                                        //
	0,                                        //
	0,                                        //
	0,                                        //
	0,                                        //
	0,                                        //
	0,                                        //
	TIM5_IRQHandler,                          //
	SPI3_IRQHandler,                          //
	UART4_IRQHandler,                         //
	UART5_IRQHandler,                         //
	TIM6_IRQHandler,                          //
	TIM7_IRQHandler,                          //
	DMA2_Channel1_IRQHandler,                 //
	DMA2_Channel2_IRQHandler,                 //
	DMA2_Channel3_IRQHandler,                 //
	DMA2_Channel4_IRQHandler,                 //
	DMA2_Channel5_IRQHandler,                 //
	ETH_IRQHandler,                           //
	ETH_WKUP_IRQHandler,                      //
	CAN2_TX_IRQHandler,                       //
	CAN2_RX0_IRQHandler,                      //
	CAN2_RX1_IRQHandler,                      //
	CAN2_SCE_IRQHandler,                      //
	OTG_FS_IRQHandler                         //
};
