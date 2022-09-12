/*****************************************************************************
 *
 * Module: STM32F401xE
 *
 * File Name: STM32F401RE.h
 *
 * Description: File include SPI Addresses, Registers and Bits
 *
 * Author: Aya Sayed
 *
 ******************************************************************************/


#ifndef STM32F401XE_H_
#define STM32F401XE_H_

/*******************************************************************************
                                     Includes
*******************************************************************************/
#include <stdio.h>
#include <stdint.h>


/*******************************************************************************
                                     RCC
*******************************************************************************/
/******************************RCC Registers' Bits*****************************/
// Clock control register (RCC_CR)
typedef union{
	volatile uint32_t CR;
	struct{
		volatile uint32_t HSION:1;
		volatile uint32_t HSIRDY:1;
		volatile uint32_t Reserved:1;
		volatile uint32_t HSITRIM:5;
		volatile uint32_t HSICAL:8;
		volatile uint32_t HSEON:1;
		volatile uint32_t HSERDY:1;
		volatile uint32_t HSEBYP:1;
		volatile uint32_t CSSON:1;
		volatile uint32_t Reserved1:4;
		volatile uint32_t PLLON:1;
		volatile uint32_t PLLRDY:1;
		volatile uint32_t PLLI2SON:1;
		volatile uint32_t PLLI2SRDY:1;
		volatile uint32_t Reserved2:4;
	};
}RCC_CR_t;

// RCC PLL configuration register (RCC_PLLCFGR)
typedef union{
	volatile uint32_t PLLCFGR;
	struct{
		volatile uint32_t PLLM0:1;
		volatile uint32_t PLLM1:1;
		volatile uint32_t PLLM2:1;
		volatile uint32_t PLLM3:1;
		volatile uint32_t PLLM4:1;
		volatile uint32_t PLLM5:1;
		volatile uint32_t PLLN:9;
		volatile uint32_t Reserved0:1;
		volatile uint32_t PLLP0:1;
		volatile uint32_t PLLP1:1;
		volatile uint32_t Reserved1:4;
		volatile uint32_t PLLSRC:1;
		volatile uint32_t Reserved2:1;
		volatile uint32_t PLLQ0:1;
		volatile uint32_t PLLQ1:1;
		volatile uint32_t PLLQ2:1;
		volatile uint32_t PLLQ3:1;
		volatile uint32_t Reserved3:4;
	};
}RCC_PLLCFGR_t;

// Clock configuration register (RCC_CFGR)
typedef union{
	volatile uint32_t CFGR;
	struct{
		volatile uint32_t SW0:1;
		volatile uint32_t SW1:1;
		volatile uint32_t SWS0:1;
		volatile uint32_t SWS1:1;
		volatile uint32_t HPRE:4;
		volatile uint32_t Reserved0:2;
		volatile uint32_t PPRE1:3;
		volatile uint32_t PPRE2:3;
		volatile uint32_t RTCPRE:5;
		volatile uint32_t MCO1:2;
		volatile uint32_t I2SSCR:1;
		volatile uint32_t MCO1PRE:3;
		volatile uint32_t MCO2PRE:3;
		volatile uint32_t MCO2:2;
	};
}RCC_CFGR_t;

// Clock interrupt register (RCC_CIR)
typedef union{
	volatile uint32_t CIR;
	struct{
		volatile uint32_t LSIRDYF:1;
		volatile uint32_t LSERDYF:1;
		volatile uint32_t HSIRDYF:1;
		volatile uint32_t HSERDYF:1;
		volatile uint32_t PLLRDYF:1;
		volatile uint32_t PLLI2SRDYF:1;
		volatile uint32_t Reserved:1;
		volatile uint32_t CSSF:1;
		volatile uint32_t LSIRDYIE:1;
		volatile uint32_t LSERDYIE:1;
		volatile uint32_t HSIRDYIE:1;
		volatile uint32_t HSERDYIE:1;
		volatile uint32_t PLLRDYIE:1;
		volatile uint32_t PLLI2SRDYIE:1;
		volatile uint32_t Reserved1:2;
		volatile uint32_t LSIRDYC:1;
		volatile uint32_t LSERDYC:1;
		volatile uint32_t HSIRDYC:1;
		volatile uint32_t HSERDYC:1;
		volatile uint32_t PLLRDYC:1;
		volatile uint32_t PLLI2SRDYC:1;
		volatile uint32_t Reserved2:1;
		volatile uint32_t CSSC:1;
		volatile uint32_t Reserved3:8;
	};
}RCC_CIR_t;

// AHB1 peripheral reset register (RCC_AHB1RSTR)
typedef union{
	volatile uint32_t AHB1RSTR;
	struct{
		volatile uint32_t GPIOARST:1;
		volatile uint32_t GPIOBRST:1;
		volatile uint32_t GPIOCRST:1;
		volatile uint32_t GPIODRST:1;
		volatile uint32_t GPIOERST:1;
		volatile uint32_t Reserved0:2;
		volatile uint32_t GPIOHRST:1;
		volatile uint32_t Reserved1:4;
		volatile uint32_t CRCRST:1;
		volatile uint32_t Reserved2:8;
		volatile uint32_t DMA1RST:1;
		volatile uint32_t Reserved3:9;
	};
}RCC_AHB1RSTR_t;

// RCC AHB2 peripheral reset register (RCC_AHB2RSTR)
typedef union{
	volatile uint32_t AHB2RSTR;
	struct{
		volatile uint32_t Reserved0:7;
		volatile uint32_t OTGFSRST:1;
		volatile uint32_t Reserved1:24;
	};
}RCC_AHB2RSTR_t;

// RCC APB1 peripheral reset register for (RCC_APB1RSTR)
typedef union{
	volatile uint32_t APB1RSTR;
	struct{
		volatile uint32_t TIM2RST:1;
		volatile uint32_t TIM3RST:1;
		volatile uint32_t TIM4RST:1;
		volatile uint32_t TIM5RST:1;
		volatile uint32_t Reserved0:7;
		volatile uint32_t WWDGRST:1;
		volatile uint32_t Reserved1:2;
		volatile uint32_t SPI2RST:1;
		volatile uint32_t SPI3RST:1;
		volatile uint32_t Reserved2:1;
		volatile uint32_t USART2RST:1;
		volatile uint32_t Reserved3:3;
		volatile uint32_t I2C1RST:1;
		volatile uint32_t I2C2RST:1;
		volatile uint32_t I2C3RST:1;
		volatile uint32_t Reserved4:4;
		volatile uint32_t PWRRST:1;
		volatile uint32_t Reserved5:3;
	};
}RCC_APB1RSTR_t;

// RCC APB2 peripheral reset register (RCC_APB2RSTR)
typedef union{
	volatile uint32_t APB2RSTR;
	struct{
		volatile uint32_t TIM1RST:1;
		volatile uint32_t Reserved0:3;
		volatile uint32_t USART1RST:1;
		volatile uint32_t USART6RST:1;
		volatile uint32_t Reserved1:2;
		volatile uint32_t ADC1RST:1;
		volatile uint32_t Reserved2:2;
		volatile uint32_t SDIORST:1;
		volatile uint32_t SPI1RST:1;
		volatile uint32_t SPI4RST:1;
		volatile uint32_t SYSCFGRST:1;
		volatile uint32_t Reserved3:1;
		volatile uint32_t TIM9RST:1;
		volatile uint32_t TIM10RST:1;
		volatile uint32_t TIM11RST:1;
		volatile uint32_t Reserved4:13;
	};
}RCC_APB2RSTR_t;

// AHB1 peripheral clock enable register (RCC_AHB1ENR)
typedef union{
	volatile uint32_t AHB1ENR;
	struct{
		volatile uint32_t GPIOAEN:1;
		volatile uint32_t GPIOBEN:1;
		volatile uint32_t GPIOCEN:1;
		volatile uint32_t GPIODEN:1;
		volatile uint32_t GPIOEEN:1;
		volatile uint32_t Reserved0:2;
		volatile uint32_t GPIOHEN:1;
		volatile uint32_t Reserved1:4;
		volatile uint32_t CRCEN:1;
		volatile uint32_t Reserved2:8;
		volatile uint32_t DMA1EN:1;
		volatile uint32_t DMA2EN:1;
		volatile uint32_t Reserved4:9;
	};
}RCC_AHB1ENR_t;

// AHB2 peripheral clock enable register (RCC_AHB2ENR)
typedef union{
	volatile uint32_t AHB2ENR;
	struct{
		volatile uint32_t Reserved0:7;
		volatile uint32_t OTGFSEN:1;
		volatile uint32_t Reserved1:24;
	};
}RCC_AHB2ENR_t;

// APB1 peripheral clock enable register (RCC_APB1ENR)
typedef union{
	volatile uint32_t APB1ENR;
	struct{
		volatile uint32_t TIM2EN:1;
		volatile uint32_t TIM3EN:1;
		volatile uint32_t TIM4EN:1;
		volatile uint32_t TIM5EN:1;
		volatile uint32_t Reserved0:7;
		volatile uint32_t WWDGEN:1;
		volatile uint32_t Reserved1:2;
		volatile uint32_t SPI2EN:1;
		volatile uint32_t SPI3EN:1;
		volatile uint32_t Reserved2:1;
		volatile uint32_t USART2EN:1;
		volatile uint32_t Reserved3:3;
		volatile uint32_t I2C1EN:1;
		volatile uint32_t I2C2EN:1;
		volatile uint32_t I2C3EN:1;
		volatile uint32_t Reserved4:4;
		volatile uint32_t PWREN:1;
		volatile uint32_t Reserved5:3;
	};
}RCC_APB1ENR_t;

// RCC APB2 peripheral clock enable register(RCC_APB2ENR)
typedef union{
	volatile uint32_t APB2ENR;
	struct{
		volatile uint32_t TIM1EN:1;
		volatile uint32_t Reserved0:3;
		volatile uint32_t USART1EN:1;
		volatile uint32_t USART6EN:1;
		volatile uint32_t Reserved1:2;
		volatile uint32_t ADC1EN:1;
		volatile uint32_t Reserved2:2;
		volatile uint32_t SDIOEN:1;
		volatile uint32_t SPI1EN:1;
		volatile uint32_t SPI4EN:1;
		volatile uint32_t SYSCFGEN:1;
		volatile uint32_t Reserved3:1;
		volatile uint32_t TIM9EN:1;
		volatile uint32_t TIM10EN:1;
		volatile uint32_t TIM11EN:1;
		volatile uint32_t Reserved4:13;
	};
}RCC_APB2ENR_t;

// RCC AHB1 peripheral clock enable in low power mode register(RCC_AHB1LPENR)
typedef union{
	volatile uint32_t AHB1LPENR;
	struct{
		volatile uint32_t GPIOALPEN:1;
		volatile uint32_t GPIOBLPEN:1;
		volatile uint32_t GPIOCLPEN:1;
		volatile uint32_t GPIODLPEN:1;
		volatile uint32_t GPIOELPEN:1;
		volatile uint32_t Reserved0:2;
		volatile uint32_t GPIOHLPEN:1;
		volatile uint32_t Reserved1:4;
		volatile uint32_t CRCLPEN:1;
		volatile uint32_t Reserved2:2;
		volatile uint32_t FLITFLPEN:1;
		volatile uint32_t SRAM1LPEN:1;
		volatile uint32_t Reserved3:4;
		volatile uint32_t DMA1LPEN:1;
		volatile uint32_t DMA2LPEN:1;
		volatile uint32_t Reserved4:9;
	};
}RCC_AHB1LPENR_t;

// RCC AHB2 peripheral clock enable in low power mode register(RCC_AHB2LPENR)
typedef union{
	volatile uint32_t AHB2LPENR;
	struct{
		volatile uint32_t Reserved0:7;
		volatile uint32_t OTGFSLPEN:1;
		volatile uint32_t Reserved1:24;
	};
}RCC_AHB2LPENR_t;

// RCC APB1 peripheral clock enable in low power mode register(RCC_APB1LPENR)
typedef union{
	volatile uint32_t APB1LPENR;
	struct{
		volatile uint32_t TIM2LPEN:1;
		volatile uint32_t TIM3LPEN:1;
		volatile uint32_t TIM4LPEN:1;
		volatile uint32_t TIM5LPEN:1;
		volatile uint32_t Reserved0:7;
		volatile uint32_t WWDGLPEN:1;
		volatile uint32_t Reserved1:2;
		volatile uint32_t SPI2LPEN:1;
		volatile uint32_t SPI3LPEN:1;
		volatile uint32_t Reserved2:1;
		volatile uint32_t USART2LPEN:1;
		volatile uint32_t Reserved3:3;
		volatile uint32_t I2C1LPEN:1;
		volatile uint32_t I2C2LPEN:1;
		volatile uint32_t I2C3LPEN:1;
		volatile uint32_t Reserved4:4;
		volatile uint32_t PWRLPEN:1;
		volatile uint32_t Reserved5:3;
	};
}RCC_APB1LPENR_t;

// RCC APB2 peripheral clock enabled in low power mode register(RCC_APB2LPENR)
typedef union{
	volatile uint32_t APB2LPENR;
	struct{
		volatile uint32_t TIM1LPEN:1;
		volatile uint32_t Reserved0:3;
		volatile uint32_t USART1LPEN:1;
		volatile uint32_t USART6LPEN:1;
		volatile uint32_t Reserved1:2;
		volatile uint32_t ADC1LPEN:1;
		volatile uint32_t Reserved2:2;
		volatile uint32_t SDIOLPEN:1;
		volatile uint32_t SPI1LPEN:1;
		volatile uint32_t SPI4LPEN:1;
		volatile uint32_t SYSCFGLPEN:1;
		volatile uint32_t Reserved3:1;
		volatile uint32_t TIM9LPEN:1;
		volatile uint32_t TIM10LPEN:1;
		volatile uint32_t TIM11LPEN:1;
		volatile uint32_t Reserved4:13;
	};
}RCC_APB2LPENR_t;

// Backup domain control register (RCC_BDCR)
typedef union{
	volatile uint32_t BDCR;
	struct{
		volatile uint32_t LSEON:1;
		volatile uint32_t LSERDY:1;
		volatile uint32_t LSEBYP:1;
		volatile uint32_t Reserved:5;
		volatile uint32_t RTCSEL:2;
		volatile uint32_t Reserved1:5;
		volatile uint32_t RTCEN:1;
		volatile uint32_t BDRST:1;
		volatile uint32_t Reserved2:15;
	};
}RCC_BDCR_t;

// Control/status register (RCC_CSR)
typedef union{
	volatile uint32_t CSR;
	struct{
		volatile uint32_t LSION:1;
		volatile uint32_t LSIRDY:1;
		volatile uint32_t Reserved:22;
		volatile uint32_t RMVF:1;
		volatile uint32_t BORRSTF:1;
		volatile uint32_t PINRSTF:1;
		volatile uint32_t PORRSTF:1;
		volatile uint32_t SETRSTF:1;
		volatile uint32_t IWDGRSTF:1;
		volatile uint32_t WWDGRSTF:1;
		volatile uint32_t LPWRRSTF:1;
	};
}RCC_CSR_t;

// RCC spread spectrum clock generation register (RCC_SSCGR)
typedef union{
	volatile uint32_t SSCGR;
	struct{
		volatile uint32_t MODPER:13;
		volatile uint32_t INCSTEP:15;
		volatile uint32_t Reserved:2;
		volatile uint32_t SPREADSEL:1;
		volatile uint32_t SSCGEN:1;
	};
}RCC_SSCGR_t;

// RCC PLLI2S configuration register (RCC_PLLI2SCFGR)
typedef union{
	volatile uint32_t PLLI2SCFGR;
	struct{
		volatile uint32_t Reserved0:6;
		volatile uint32_t PLLI2SN0:1;
		volatile uint32_t PLLI2SN1:1;
		volatile uint32_t PLLI2SN2:1;
		volatile uint32_t PLLI2SN3:1;
		volatile uint32_t PLLI2SN4:1;
		volatile uint32_t PLLI2SN5:1;
		volatile uint32_t PLLI2SN6:1;
		volatile uint32_t PLLI2SN7:1;
		volatile uint32_t PLLI2SN8:1;
		volatile uint32_t Reserved1:13;
		volatile uint32_t PLLI2SR0:1;
		volatile uint32_t PLLI2SR1:1;
		volatile uint32_t PLLI2SR2:1;
		volatile uint32_t Reserved2:1;
	};
}RCC_PLLI2SCFGR_t;

// RCC Dedicated Clocks Configuration Register (RCC_DCKCFGR)
typedef union{
	volatile uint32_t DCKCFGR;
	struct{
		volatile uint32_t Reserved0:24;
		volatile uint32_t TIMPRE:1;
		volatile uint32_t Reserved1:7;
	};
}RCC_DCKCFGR_t;

/*********************************RCC Registers********************************/
typedef struct{
	volatile RCC_CR_t CR;
	volatile RCC_PLLCFGR_t PLLCFGR;
	volatile RCC_CFGR_t CFGR;
	volatile RCC_CIR_t CIR;
	volatile RCC_AHB1RSTR_t AHB1RSTR;
	volatile RCC_AHB2RSTR_t AHB2RSTR;
	volatile RCC_APB1RSTR_t APB1RSTR;
	volatile RCC_APB2RSTR_t APB2RSTR;
	volatile RCC_AHB1ENR_t AHB1ENR;
	volatile RCC_AHB2ENR_t AHB2ENR;
	volatile RCC_APB1ENR_t APB1ENR;
	volatile RCC_APB2ENR_t APB2ENR;
	volatile RCC_AHB1LPENR_t AHB1LPENR;
	volatile RCC_AHB2LPENR_t AHB2LPENR;
	volatile RCC_APB1LPENR_t APB1LPENR;
	volatile RCC_APB2LPENR_t APB2LPENR;
	volatile RCC_BDCR_t BDCR;
	volatile RCC_CSR_t CSR;
	volatile RCC_SSCGR_t SSCGR;
	volatile RCC_PLLI2SCFGR_t PLLI2SCFGR;
	volatile RCC_DCKCFGR_t DCKCFGR;
}RCC_t;

/**********************************RCC Address*********************************/
#define RCC                                              ((RCC_t *)0x40023800)


/*******************************************************************************
                                    GPIO
*******************************************************************************/
/*******************************GPIO Registers*********************************/
typedef struct{
	volatile uint32_t GPIOx_MODER;
	volatile uint32_t GPIOx_OTYPER;
	volatile uint32_t GPIOx_OSPEEDR;
	volatile uint32_t GPIOx_PUPDR;
	volatile uint32_t GPIOx_IDR;
	volatile uint32_t GPIOx_ODR;
	volatile uint32_t GPIOx_BSRR;
	volatile uint32_t GPIOx_LCKR;
	volatile uint32_t GPIOx_AFRL;
	volatile uint32_t GPIOx_AFRH;
}GPIO_t;

/***************************GPIO Peripheral Instants***************************/
#define GPIOA                                             ((GPIO_t *)0x40020000)
#define GPIOB                                             ((GPIO_t *)0x40020400)
#define GPIOC                                             ((GPIO_t *)0x40020800)
#define GPIOD                                             ((GPIO_t *)0x40020C00)
#define GPIOE                                             ((GPIO_t *)0x40021000)
#define GPIOH                                             ((GPIO_t *)0x40021C00)


/*******************************************************************************
                                    SPI
*******************************************************************************/
/****************************SPI Registers' Bits*******************************/
// SPI control register 1 (SPI_CR1)
typedef union{
	volatile uint32_t CR1;
	struct{
		volatile uint32_t CPHA:1;
		volatile uint32_t CPOL:1;
		volatile uint32_t MSTR:1;
		volatile uint32_t BR:3;
		volatile uint32_t SPE:1;
		volatile uint32_t LSBFIRST:1;
		volatile uint32_t SSI:1;
		volatile uint32_t SSM:1;
		volatile uint32_t RXONLY:1;
		volatile uint32_t DFF:1;
		volatile uint32_t CRCNEXT:1;
		volatile uint32_t CRCEN:1;
		volatile uint32_t BIDIOE:1;
		volatile uint32_t BIDIMODE:1;
		volatile uint32_t Reserved:16;
	};
}SPI_CR1_t;

// SPI control register 2 (SPI_CR2)
typedef union{
	volatile uint32_t CR2;
	struct{
		volatile uint32_t RXDMAEN:1;
		volatile uint32_t TXDMAEN:1;
		volatile uint32_t SSOE:1;
		volatile uint32_t Reserved1:2;
		volatile uint32_t ERRIE:1;
		volatile uint32_t RXNEIE:1;
		volatile uint32_t TXEIE:1;
		volatile uint32_t Reserved2:24;
	};
}SPI_CR2_t;

// SPI status register (SPI_SR)
typedef union{
	volatile uint32_t SR;
	struct{
		volatile uint32_t RXNE:1;
		volatile uint32_t TXE:1;
		volatile uint32_t CHSIDE:1;
		volatile uint32_t UDR:1;
		volatile uint32_t CRCERR:1;
		volatile uint32_t MODF:1;
		volatile uint32_t OVR:1;
		volatile uint32_t BSY:1;
		volatile uint32_t FRE:1;
		volatile uint32_t Reserved:23;
	};
}SPI_SR_t;

// SPI data register (SPI_DR)
typedef union{
	volatile uint32_t _DR;
	struct{
		volatile uint32_t DR:16;
		volatile uint32_t Reserved:16;
	};
}SPI_DR_t;

// SPI CRC polynomial register (SPI_CRCPR)
typedef union{
	volatile uint32_t _CRCPOLY;
	struct{
		volatile uint32_t CRCPOLY:16;
		volatile uint32_t Reserved:16;
	};
}SPI_CRCPR_t;

// SPI RX CRC register (SPI_RXCRCR)
typedef union{
	volatile uint32_t _RXCRC;
	struct{
		volatile uint32_t RXCRC:16;
		volatile uint32_t Reserved:16;
	};
}SPI_RXCRCR_t;

// SPI TX CRC register (SPI_TXCRCR)
typedef union{
	volatile uint32_t _TXCRC;
	struct{
		volatile uint32_t TXCRC:16;
		volatile uint32_t Reserved:16;
	};
}SPI_TXCRCR_t;

/********************************SPI Registers*********************************/
typedef struct{
	volatile SPI_CR1_t CR1;
	volatile SPI_CR2_t CR2;
	volatile SPI_SR_t SR;
	volatile SPI_DR_t DR;
	volatile SPI_CRCPR_t SPI_CRCPR;
	volatile SPI_RXCRCR_t SPI_RXCRCR;
	volatile SPI_TXCRCR_t SPI_TXCRCR;
	volatile uint32_t SPI_I2SCFGR;
	volatile uint32_t SPI_I2SPR;
}SPI_t;


/****************************SPI Peripheral Instants***************************/
#define SPI1                                              ((SPI_t *)0x40013000)
#define SPI2                                              ((SPI_t *)0x40003800)
#define SPI3                                              ((SPI_t *)0x40003C00)
#define SPI4                                              ((SPI_t *)0x40013400)



/*******************************************************************************
               Base addresses for Peripherals related with CORE
*******************************************************************************/
/*****************Nested vectored interrupt controller (NVIC)******************/
#define NVIC_BASE                                                  0xE000E100UL

/********************************NVIC Registers********************************/
// Interrupt set-enable registers (NVIC_ISERx)
typedef struct{
	volatile uint32_t NVIC_ISER0;
	volatile uint32_t NVIC_ISER1;
	volatile uint32_t NVIC_ISER2;
}NVIC_ISER_t;
#define NVIC_ISER                ((volatile NVIC_ISER_t *)(NVIC_BASE + 0x00))

// Interrupt clear-enable registers (NVIC_ICERx)
typedef struct{
	volatile uint32_t NVIC_ICER0;
	volatile uint32_t NVIC_ICER1;
	volatile uint32_t NVIC_ICER2;
}NVIC_ICER_t;
#define NVIC_ICER                ((volatile NVIC_ICER_t *)(NVIC_BASE + 0x80))

// Interrupt set-pending registers (NVIC_ISPRx)
typedef struct{
	volatile uint32_t NVIC_ISPR0;
	volatile uint32_t NVIC_ISPR1;
	volatile uint32_t NVIC_ISPR2;
}NVIC_ISPR_t;
#define NVIC_ISPR                ((volatile NVIC_ISPR_t *)(NVIC_BASE + 0x100))

// Interrupt clear-pending registers (NVIC_ICPRx)
typedef struct{
	volatile uint32_t NVIC_ICPR0;
	volatile uint32_t NVIC_ICPR1;
	volatile uint32_t NVIC_ICPR2;
}NVIC_ICPR_t;
#define NVIC_ICPR                ((volatile NVIC_ICPR_t *)(NVIC_BASE + 0x180))

// Interrupt active bit registers (NVIC_IABRx)
typedef struct{
	volatile uint32_t NVIC_IABR0;
	volatile uint32_t NVIC_IABR1;
	volatile uint32_t NVIC_IABR2;
}NVIC_IABR_t;
#define NVIC_IABR                ((volatile NVIC_IABR_t *)(NVIC_BASE + 0x200))

// Interrupt priority registers (NVIC_IPRx)
typedef struct{
	volatile uint32_t NVIC_IPR[21];
}NVIC_IPR_t;
#define NVIC_IPR                 ((volatile NVIC_IPR_t *)(NVIC_BASE + 0x300))

// Software trigger interrupt register (NVIC_STIR)
#define NVIC_STIR                ((volatile uint32_t   *)(NVIC_BASE + 0xE00))

/*******************************************************************************
              Nested vectored interrupt controller (NVIC) Macros
                           Interrupt Request Macros
*******************************************************************************/
/************************SPI Request Position Macros***************************/
#define SPI1_IRQ	          35
#define SPI2_IRQ		      36
#define SPI3_IRQ              51
#define SPI4_IRQ              84


/*******************************************************************************
                                  Generic Macros
*******************************************************************************/
#define TRUE        1
#define FALSE       0




#endif /* STM32F401XE_H_ */
