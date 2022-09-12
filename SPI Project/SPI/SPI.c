/*
 * SPI.c
 *
 *  Created on: Sep 11, 2022
 *      Author: Aya Sayed
 *
 *******************************************************************************
                                     Includes
*******************************************************************************/
#include "SPI.h"

/*******************************************************************************
                                Global Variables
*******************************************************************************/
static SPI_Config_t* Global_SPI_Config[4]= {NULL};
#define SPI1_Index              0
#define SPI2_Index              1
#define SPI3_Index              2
#define SPI4_Index              3

/*******************************************************************************
                     APIs Supported by "MCAL SPI DRIVER"
*******************************************************************************/
/*******************************************************************************
* Fn                -MCAL_SPI_Init.
*
* Brief             -Initializes SPIx according to the specified parameters in
*                    SPI_Config_t.
*
* Param [in]        -SPIx: where x can be (1..4 depending on device used) to
*                    select SPI peripheral.
*
* Param [in]        -SPI_Config: All SPI configuration.
*
* Retval            -None.
*
* Note              -Supported for SPI FULL DUPLEX Master/Slave only and
* 					 NSS Hardware/Software.
* 					-You have to configure RCC to select clock for the
* 					 selected SPI Module.
*/
void MCAL_SPI_Init(SPI_t *SPIx, SPI_Config_t *SPI_Config)
{
	// 1. Enable the clock for given SPI peripheral
	if      (SPIx == SPI1) { RCC->APB2ENR.SPI1EN = TRUE; Global_SPI_Config[SPI1_Index] = SPI_Config; }
	else if (SPIx == SPI2) { RCC->APB1ENR.SPI2EN = TRUE; Global_SPI_Config[SPI2_Index] = SPI_Config; }
	else if (SPIx == SPI3) { RCC->APB1ENR.SPI3EN = TRUE; Global_SPI_Config[SPI3_Index] = SPI_Config; }
	else if (SPIx == SPI4) { RCC->APB2ENR.SPI4EN = TRUE; Global_SPI_Config[SPI4_Index] = SPI_Config; }
	else                   { /*  Misra  */ }

	// 2. Set Master or Slave
	SPIx->CR1.MSTR = SPI_Config->SPI_Mode;

	// 3. Set Communication Mode
	switch (SPI_Config->Communication_Mode){
		case SPI_COMMUNICATION_MODE_2LINE_FULL_DUPLEX: SPIx->CR1.BIDIMODE = FALSE; break;
		case SPI_COMMUNICATION_MODE_2LINE_RXE_ONLY:    SPIx->CR1.RXONLY   = FALSE; break;
		case SPI_COMMUNICATION_MODE_1LINE_RX_ONLY:
			SPIx->CR1.BIDIMODE = TRUE ;
			SPIx->CR1.BIDIOE   = FALSE;
			break;
		case SPI_COMMUNICATION_MODE_1LINE_TX_ONLY:
			SPIx->CR1.BIDIMODE = TRUE;
			SPIx->CR1.BIDIOE   = TRUE;
			break;
	}

	// 4. Set Frame Format
	SPIx->CR1.LSBFIRST = SPI_Config->Frame_Format;

	// 5. Set Data size
	SPIx->CR1.DFF = SPI_Config->Frame_Size;

	// 6. Set Clock Polarity
	SPIx->CR1.CPOL = SPI_Config->CLK_Polarity;

	// 7. Set Clock Phase
	SPIx->CR1.CPHA = SPI_Config->CLK_Phase;

	// 8. Set Slave Select Management
	if      (SPI_Config->NSS == SPI_NSS_HW_SLAVE)                       {SPIx->CR2.SSOE = FALSE;}
	else if (SPI_Config->NSS == SPI_NSS_HW_MASTER_SS_OUTPUT_ENABLED)    {SPIx->CR2.SSOE = TRUE; }
	else if (SPI_Config->NSS == SPI_NSS_HW_MASTER_SS_OUTPUT_DISABLED)   {SPIx->CR2.SSOE = FALSE;}
	else if (SPI_Config->NSS == SPI_NSS_SW_SSI_RESET)                   {SPIx->CR1.SSM  = TRUE; }
	else if (SPI_Config->NSS == SPI_NSS_SW_SSI_SET){SPIx->CR1.SSM  = TRUE; SPIx->CR1.SSI = TRUE;}
	else                                                                        { /*  Misra  */ }

	// 9. Set BoudRate Pre-scaler
	SPIx->CR1.BR = SPI_Config->BaudRate_Prescaler;

	// 10. Set Interrupt
	if (SPI_Config->IRQ_Enable != SPI_IRQ_ENABLE_NONE)
	{
		if      (SPI_Config->IRQ_Enable == SPI_IRQ_ENABLE_TXEIE)  { SPIx->CR2.TXEIE  = TRUE; }
		else if (SPI_Config->IRQ_Enable == SPI_IRQ_ENABLE_ERRIE)  { SPIx->CR2.ERRIE  = TRUE; }
		else if (SPI_Config->IRQ_Enable == SPI_IRQ_ENABLE_RXNEIE) { SPIx->CR2.RXNEIE = TRUE; }
		else                                                      { /*  Misra  */ }

		// 11. Open the global Interrupt for each peripheral
		if      (SPIx == SPI1){ NVIC_ISER->NVIC_ISER1 |= (1 << (SPI1_IRQ - 32)); }
		else if (SPIx == SPI2){ NVIC_ISER->NVIC_ISER1 |= (1 << (SPI2_IRQ - 32)); }
		else if (SPIx == SPI3){ NVIC_ISER->NVIC_ISER1 |= (1 << (SPI3_IRQ - 32)); }
		else if (SPIx == SPI4){ NVIC_ISER->NVIC_ISER2 |= (1 << (SPI4_IRQ - 64)); }
		else                  { /*  Misra  */ }

	}
	else { /*  Misra  */ }

	// 12. Enable SPI
	SPIx->CR1.SPE = TRUE;
}

/*******************************************************************************
* Fn                -MCAL_SPI_DeInit.
*
* Brief             -Resets Selected SPI Module.
*
* Param [in]        -SPIx: where x can be (1..2 depending on device used) to
*                    select SPI peripheral.
*
* Retval            -None.
*
* Note              -Reset The Module By RCC & Disable NVIC.
*/
void MCAL_SPI_DeInit(SPI_t *SPIx)
{
	if      (SPIx == SPI1) { RCC->APB2RSTR.SPI1RST = TRUE; NVIC_ICER->NVIC_ICER1 |= (1 << (SPI1_IRQ - 32)); }
	else if (SPIx == SPI2) { RCC->APB1RSTR.SPI2RST = TRUE; NVIC_ICER->NVIC_ICER1 |= (1 << (SPI2_IRQ - 32)); }
	else if (SPIx == SPI3) { RCC->APB1RSTR.SPI3RST = TRUE; NVIC_ICER->NVIC_ICER1 |= (1 << (SPI3_IRQ - 32)); }
	else if (SPIx == SPI4) { RCC->APB2RSTR.SPI4RST = TRUE; NVIC_ICER->NVIC_ICER2 |= (1 << (SPI4_IRQ - 64)); }
	else                   { /*  Misra  */ }

}

/*******************************************************************************
* Fn                -MCAL_SPI_Send_Data.
*
* Brief             -Send Buffer With SPI.
*
* Param [in]        -SPIx: where x can be (1..2 depending on device used) to
*                    select SPI peripheral.
*
* param [in] 		-P_TxBuffer: Pointer to buffer Which holds the TX data.
*
* Param [in]        -Mechanism is defined at  @Ref Mechanism_define.
*
* Retval            -None.
*
* Note              -None.
*/
void MCAL_SPI_Send_Data(SPI_t *SPIx, uint16_t *P_TxBuffer, uint8_t Mechanism)
{
	// 1. Wait until TXE (Transmit data register empty)
	if (Mechanism == Polling_Mechanism) { while(!(SPIx->SR.TXE)); }
	else                                { /*  Misra  */ }

	// 2. Start transmission, Write data to SPI data register
	SPIx->DR.DR = *(P_TxBuffer);
}

/*******************************************************************************
* Fn                -MCAL_SPI_Recieve_Data.
*
* Brief             -Receive Buffer With SPI.
*
* Param [in]        -SPIx: where x can be (1..2 depending on device used) to
*                    select SPI peripheral.
*
* param [in] 		-P_TxBuffer: Pointer to buffer Which holds the RX data.
*
* Param [in]        -Mechanism is defined at  @Ref Mechanism_define.
*
* Retval            -None.
*
* Note              -None.
*/
void MCAL_SPI_Recieve_Data(SPI_t *SPIx, uint16_t *P_RxBuffer, uint8_t Mechanism)
{
	// 1. Wait until RXNE (Read data register not empty)
	if (Mechanism == Polling_Mechanism) { while(!(SPIx->SR.RXNE)); }
	else                                { /*  Misra  */ }

	// 2. Start reception, Write data to SPI data register
	*(P_RxBuffer) = (uint16_t)SPIx->DR.DR;
}

/*******************************************************************************
* Fn                -MCAL_SPI_TX_RX.
*
* Brief             -Transmit and Receive Data.
*
* Param [in]        -SPIx: where x can be (1..2 depending on device used) to
*                    select SPI peripheral.
*
* param [in] 		-P_TxBuffer: Pointer to buffer Which holds the RX data.
*
* Param [in]        -Mechanism is defined at  @Ref Mechanism_define.
*
* Retval            -None.
*
* Note              -None.
*/
void MCAL_SPI_TX_RX(SPI_t *SPIx, uint16_t *P_TxBuffer, uint8_t Mechanism)
{
	// 1. Wait until TXE (Transmit data register empty)
	if (Mechanism == Polling_Mechanism) { while(!(SPIx->SR.TXE)); }
	else                                { /*  Misra  */ }

	// 2. Start transmission, Write data to SPI data register
	SPIx->DR.DR = *(P_TxBuffer);

	// 3. Wait until RXNE (Read data register not empty)
	if (Mechanism == Polling_Mechanism) { while(!(SPIx->SR.RXNE)); }
	else                                { /*  Misra  */ }

	// 4. Start reception, Write data to SPI data register
	*(P_TxBuffer) = (uint16_t)SPIx->DR.DR;
}

/*******************************************************************************
* Fn                -MCAL_SPI_GPIO_Set_Pins.
*
* Brief             -Initializes GPIO Pins to be connected with the selected SPI.
*
* Param [in]        -SPIx: where x can be (1..2 depending on device used) to
*                    select SPI peripheral.
*
* Retval            -None.
*
* Note              -Must open clock for AFIO & GPIO After GPIO Initialization.
*                   -Supported for SPI FULL DUPLEX Master/Slave only & NSS Hardware/Software.
*                   -Should implement GPIO driver to I will replace the following code by an
*                    optimized one.
*/
void MCAL_SPI_GPIO_Set_Pins(SPI_t *SPIx)
{
	if (SPIx == SPI1)
	{
		// Enable GPIOA
		RCC->AHB1ENR.GPIOAEN = TRUE;

		//Configure MOSI, MISO, SCK, NSS Pins
		if (Global_SPI_Config[SPI1_Index]->NSS == SPI_NSS_HW_MASTER_SS_OUTPUT_DISABLED
		 || Global_SPI_Config[SPI1_Index]->NSS == SPI_NSS_HW_MASTER_SS_OUTPUT_ENABLED
		 || Global_SPI_Config[SPI1_Index]->NSS == SPI_NSS_HW_SLAVE	)
		{
			GPIOA->GPIOx_AFRH    |=  (1U << 28); // Select AF5 to enable SPI1
			GPIOA->GPIOx_AFRH    |=  (1U << 30);
			GPIOA->GPIOx_MODER   &= ~(1U << 30); // Specify GPIOA Pin15 Alternate function mode
			GPIOA->GPIOx_MODER   |=  (1U << 31);
			GPIOA->GPIOx_OTYPER  &= ~(1U << 15); // Specify GPIOA Pin15 Output push-pull
			GPIOA->GPIOx_OSPEEDR |=  (3U << 30); // Specify GPIOA Pin15 at a Very high speed
			GPIOA->GPIOx_PUPDR   &= ~(3U << 30); // Specify GPIOA Pin15 at No pull-up, pull-down

		}
		else { /* Misra */ }

		GPIOA->GPIOx_AFRL    |=  (1U << 20); // Select AF5 to enable SPI1
		GPIOA->GPIOx_AFRL    |=  (1U << 22);
		GPIOA->GPIOx_MODER   &= ~(1U << 10); // Specify GPIOA Pin5 Alternate function mode
		GPIOA->GPIOx_MODER   |=  (1U << 11);
		GPIOA->GPIOx_OTYPER  &= ~(1U << 5 ); // Specify GPIOA Pin5 Output push-pull
		GPIOA->GPIOx_OSPEEDR |=  (3U << 10); // Specify GPIOA Pin5 at a Very high speed
		GPIOA->GPIOx_PUPDR   &= ~(3U << 10); // Specify GPIOA Pin5 at No pull-up, pull-down

		GPIOA->GPIOx_AFRL    |=  (1U << 24); // Select AF5 to enable SPI1
		GPIOA->GPIOx_AFRL    |=  (1U << 26);
		GPIOA->GPIOx_MODER   &= ~(1U << 12); // Specify GPIOA Pin6 Alternate function mode
		GPIOA->GPIOx_MODER   |=  (1U << 13);
		GPIOA->GPIOx_OTYPER  &= ~(1U << 6 ); // Specify GPIOA Pin6 Output push-pull
		GPIOA->GPIOx_OSPEEDR |=  (3U << 12); // Specify GPIOA Pin6 at a Very high speed
		GPIOA->GPIOx_PUPDR   &= ~(3U << 12); // Specify GPIOA Pin6 at No pull-up, pull-down

		GPIOA->GPIOx_AFRL    |=  (1U << 28); // Select AF5 to enable SPI1
		GPIOA->GPIOx_AFRL    |=  (1U << 30);
		GPIOA->GPIOx_MODER   &= ~(1U << 14); // Specify GPIOA Pin7 Alternate function mode
		GPIOA->GPIOx_MODER   |=  (1U << 15);
		GPIOA->GPIOx_OTYPER  &= ~(1U << 7 ); // Specify GPIOA Pin7 Output push-pull
		GPIOA->GPIOx_OSPEEDR |=  (3U << 14); // Specify GPIOA Pin7 at a Very high speed
		GPIOA->GPIOx_PUPDR   &= ~(3U << 14); // Specify GPIOA Pin7 at No pull-up, pull-down


	}
	else if (SPIx == SPI2)
	{
		//Enable GPIOB and GPIOC
		RCC->AHB1ENR.GPIOBEN = TRUE;
		RCC->AHB1ENR.GPIOCEN = TRUE;

		//Configure MOSI, MISO, SCK, NSS Pins
		if (Global_SPI_Config[SPI2_Index]->NSS == SPI_NSS_HW_MASTER_SS_OUTPUT_DISABLED
		 || Global_SPI_Config[SPI2_Index]->NSS == SPI_NSS_HW_MASTER_SS_OUTPUT_ENABLED
		 || Global_SPI_Config[SPI2_Index]->NSS == SPI_NSS_HW_SLAVE	)
		{
			GPIOB->GPIOx_AFRH    |=  (1U << 16); // Select AF5 to enable SPI1
			GPIOB->GPIOx_AFRH    |=  (1U << 18);
			GPIOB->GPIOx_MODER   &= ~(1U << 24); // Specify GPIOA Pin12 Alternate function mode
			GPIOB->GPIOx_MODER   |=  (1U << 25);
			GPIOB->GPIOx_OTYPER  &= ~(1U << 12); // Specify GPIOA Pin12 Output push-pull
			GPIOB->GPIOx_OSPEEDR |=  (3U << 24); // Specify GPIOA Pin12 at a Very high speed
			GPIOB->GPIOx_PUPDR   &= ~(3U << 24); // Specify GPIOA Pin12 at No pull-up, pull-down

		}
		else { /* Misra */ }

		GPIOC->GPIOx_AFRL    |=  (1U <<  8); // Select AF5 to enable SPI1
		GPIOC->GPIOx_AFRL    |=  (1U << 10);
		GPIOC->GPIOx_MODER   &= ~(1U << 4 ); // Specify GPIOC Pin2 Alternate function mode
		GPIOC->GPIOx_MODER   |=  (1U << 5 );
		GPIOC->GPIOx_OTYPER  &= ~(1U << 2 ); // Specify GPIOC Pin2 Output push-pull
		GPIOC->GPIOx_OSPEEDR |=  (3U << 4 ); // Specify GPIOC Pin2 at a Very high speed
		GPIOC->GPIOx_PUPDR   &= ~(3U << 4 ); // Specify GPIOC Pin2 at No pull-up, pull-down

		GPIOC->GPIOx_AFRL    |=  (1U << 12); // Select AF5 to enable SPI1
		GPIOC->GPIOx_AFRL    |=  (1U << 14);
		GPIOC->GPIOx_MODER   &= ~(1U << 6);  // Specify GPIOA Pin3 Alternate function mode
		GPIOC->GPIOx_MODER   |=  (1U << 7);
		GPIOC->GPIOx_OTYPER  &= ~(1U << 3);  // Specify GPIOA Pin3 Output push-pull
		GPIOC->GPIOx_OSPEEDR |=  (3U << 6);  // Specify GPIOA Pin3 at a Very high speed
		GPIOC->GPIOx_PUPDR   &= ~(3U << 6);  // Specify GPIOA Pin3 at No pull-up, pull-down

		GPIOB->GPIOx_AFRH    |=  (1U <<  8); // Select AF5 to enable SPI1
		GPIOB->GPIOx_AFRH    |=  (1U << 10);
		GPIOB->GPIOx_MODER   &= ~(1U << 20); // Specify GPIOA Pin10 Alternate function mode
		GPIOB->GPIOx_MODER   |=  (1U << 21);
		GPIOB->GPIOx_OTYPER  &= ~(1U << 10); // Specify GPIOA Pin10 Output push-pull
		GPIOB->GPIOx_OSPEEDR |=  (3U << 20); // Specify GPIOA Pin10 at a Very high speed
		GPIOB->GPIOx_PUPDR   &= ~(3U << 20); // Specify GPIOA Pin10 at No pull-up, pull-down

	}
	else if (SPIx == SPI3)
	{
		//Enable GPIOA and GPIOC
		RCC->AHB1ENR.GPIOAEN = TRUE;
		RCC->AHB1ENR.GPIOCEN = TRUE;

		//Configure MOSI, MISO, SCK, NSS Pins
		if (Global_SPI_Config[SPI3_Index]->NSS == SPI_NSS_HW_MASTER_SS_OUTPUT_DISABLED
		 || Global_SPI_Config[SPI3_Index]->NSS == SPI_NSS_HW_MASTER_SS_OUTPUT_ENABLED
		 || Global_SPI_Config[SPI3_Index]->NSS == SPI_NSS_HW_SLAVE	)
		{
			GPIOA->GPIOx_AFRL    |=  (1U << 17); // Select AF6 to enable SPI1
			GPIOA->GPIOx_AFRL    |=  (1U << 18);
			GPIOA->GPIOx_MODER   &= ~(1U << 8); // Specify GPIOA Pin4 Alternate function mode
			GPIOA->GPIOx_MODER   |=  (1U << 9);
			GPIOA->GPIOx_OTYPER  &= ~(1U << 4); // Specify GPIOA Pin4 Output push-pull
			GPIOA->GPIOx_OSPEEDR |=  (3U << 8); // Specify GPIOA Pin4 at a Very high speed
			GPIOA->GPIOx_PUPDR   &= ~(3U << 8); // Specify GPIOA Pin4 at No pull-up, pull-down

		}
		else { /* Misra */ }

		GPIOC->GPIOx_AFRH    |=  (1U <<  9); // Select AF6 to enable SPI1
		GPIOC->GPIOx_AFRH    |=  (1U << 10);
		GPIOC->GPIOx_MODER   &= ~(1U << 20); // Specify GPIOC Pin10 Alternate function mode
		GPIOC->GPIOx_MODER   |=  (1U << 21);
		GPIOC->GPIOx_OTYPER  &= ~(1U << 10); // Specify GPIOC Pin10 Output push-pull
		GPIOC->GPIOx_OSPEEDR |=  (3U << 20); // Specify GPIOC Pin10 at a Very high speed
		GPIOC->GPIOx_PUPDR   &= ~(3U << 20); // Specify GPIOC Pin10 at No pull-up, pull-down

		GPIOC->GPIOx_AFRH    |=  (1U << 13); // Select AF6 to enable SPI1
		GPIOC->GPIOx_AFRH    |=  (1U << 14);
		GPIOC->GPIOx_MODER   &= ~(1U << 22); // Specify GPIOC Pin11 Alternate function mode
		GPIOC->GPIOx_MODER   |=  (1U << 23);
		GPIOC->GPIOx_OTYPER  &= ~(1U << 11); // Specify GPIOC Pin11 Output push-pull
		GPIOC->GPIOx_OSPEEDR |=  (3U << 22); // Specify GPIOC Pin11 at a Very high speed
		GPIOC->GPIOx_PUPDR   &= ~(3U << 22); // Specify GPIOC Pin11 at No pull-up, pull-down

		GPIOC->GPIOx_AFRH    |=  (1U << 17); // Select AF6 to enable SPI1
		GPIOC->GPIOx_AFRH    |=  (1U << 18);
		GPIOC->GPIOx_MODER   &= ~(1U << 24); // Specify GPIOC Pin12 Alternate function mode
		GPIOC->GPIOx_MODER   |=  (1U << 25);
		GPIOC->GPIOx_OTYPER  &= ~(1U << 12); // Specify GPIOC Pin12 Output push-pull
		GPIOC->GPIOx_OSPEEDR |=  (3U << 24); // Specify GPIOC Pin12 at a Very high speed
		GPIOC->GPIOx_PUPDR   &= ~(3U << 24); // Specify GPIOC Pin12 at No pull-up, pull-down


	}
	else
	{

	}

}


/*******************************************************************************
                                  IRQ Handlers
*******************************************************************************/
void SPI1_IRQHandler(void)
{
	Global_SPI_Config[SPI1_Index]->P_IRQ_CallBack();
}

void SPI2_IRQHandler(void)
{
	Global_SPI_Config[SPI2_Index]->P_IRQ_CallBack();
}

void SPI3_IRQHandler(void)
{
	Global_SPI_Config[SPI3_Index]->P_IRQ_CallBack();
}

void SPI4_IRQHandler(void)
{
	Global_SPI_Config[SPI4_Index]->P_IRQ_CallBack();
}
