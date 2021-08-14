/***************************************************************************
 * Artekit Wavetooeasy
 * https://www.artekit.eu/products/devboards/wavetooeasy
 *
 * Written by Ivan Meleca
 * Copyright (c) 2021 Artekit Labs
 * https://www.artekit.eu

### variant.cpp

#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program; if not, write to the Free Software
#   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

***************************************************************************/

#include "variant.h"
#include <sys/types.h>
#include <ServiceTimer.h>

RingBuffer rx_bufferSerial;
RingBuffer tx_bufferSerial;
UARTClass Serial(USART1, USART1_IRQn, &rx_bufferSerial, &tx_bufferSerial);
AudioManager Audio = AudioManager::instance();
FATFS fs;

extern uint32_t program_offset;
extern "C" void __libc_init_array(void);

#define WITH_BOOTLOADER	1

static const uint8_t signature[] __attribute__ ((section(".signature"), used )) =
{
	"AAKK-71EA-EE5F-8F4E-88FFE568BCAE"
};

const PinDescription g_APinDescription[] =
{
	// Channels
	{ GPIOA,  0,  GPIO_Pin_0, 	NULL,  0, NULL, 0, 	0, 0, 0 },
	{ GPIOA,  1,  GPIO_Pin_1, 	NULL,  0, NULL, 0, 	1, 0, 0 },
	{ GPIOA,  2,  GPIO_Pin_2, 	NULL,  0, NULL, 0, 	2, 0, 0 },
	{ GPIOA,  3,  GPIO_Pin_3, 	NULL,  0, NULL, 0, 	3, 0, 0 },
	{ GPIOA,  4,  GPIO_Pin_4, 	NULL,  0, NULL, 0, 	4, 0, 0 },
	{ GPIOA,  5,  GPIO_Pin_5, 	NULL,  0, NULL, 0, 	5, 0, 0 },
	{ GPIOA,  6,  GPIO_Pin_6, 	NULL,  0, NULL, 0, 	6, 0, 0 },
	{ GPIOA,  7,  GPIO_Pin_7, 	NULL,  0, NULL, 0, 	7, 0, 0 },
	{ GPIOA,  8,  GPIO_Pin_8, 	NULL,  0, NULL, 0, 	8, 0, 0 },
	{ GPIOB,  9,  GPIO_Pin_9, 	NULL,  0, NULL, 0, 	9, 1, 0 },
	{ GPIOB, 10,  GPIO_Pin_10,	NULL,  0, NULL, 0, 10, 1, 0 },
	{ GPIOA, 11,  GPIO_Pin_11, 	NULL,  0, NULL, 0, 11, 0, 0 },
	{ GPIOA, 12,  GPIO_Pin_12, 	NULL,  0, NULL, 0, 12, 0, 0 },
	{ GPIOC, 13,  GPIO_Pin_13, 	NULL,  0, NULL, 0, 13, 2, 0 },
	{ GPIOC, 14,  GPIO_Pin_14, 	NULL,  0, NULL, 0, 14, 2, 0 },
	{ GPIOC, 15,  GPIO_Pin_15, 	NULL,  0, NULL, 0, 15, 2, 0 },

	// Latch
	{ GPIOC, 4,   GPIO_Pin_4, 	NULL,  0, NULL, 0,  4, 2, 0 },

	// LEDs
	{ GPIOB,  0,  GPIO_Pin_0, 	NULL,  0, NULL, 0, NIRQ, 0, 0 },
	{ GPIOB,  1,  GPIO_Pin_1, 	NULL,  0, NULL, 0, NIRQ, 0, 0 },
};

extern "C" void USART1_IRQHandler(void)
{
	Serial.IrqHandler();
}

static void initADC()
{
	ADC_InitTypeDef ADC_InitStruct;
	ADC_CommonInitTypeDef ADCCommon_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	ADCCommon_InitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADCCommon_InitStruct.ADC_Mode = ADC_Mode_Independent;
	ADCCommon_InitStruct.ADC_Prescaler = ADC_Prescaler_Div4;
	ADCCommon_InitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADCCommon_InitStruct);

	ADC_StructInit(&ADC_InitStruct);
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStruct.ADC_NbrOfConversion = 1;
	ADC_InitStruct.ADC_ScanConvMode = ENABLE;
	ADC_Init(ADC1, &ADC_InitStruct);
	ADC_Cmd(ADC1, ENABLE);
	ADC_TempSensorVrefintCmd(ENABLE);

	// Configure battery AD (PC0)
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
}

static inline void initClocks()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB |
						   RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD,
						   ENABLE);
}

void init(void)
{
#if WITH_BOOTLOADER
	// SystemInit() was already called by the bootloader, so...
	// Initialize FPU (usually done by SystemInit())
	#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
	#endif

	// And reallocate vector table
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, ((uint32_t) &program_offset) - NVIC_VectTab_FLASH);
#else
	/* Call the clock system initialization function.*/
	SystemInit();
#endif

	/* Call static constructors */
	__libc_init_array();

	SysTick_Config((SystemCoreClock/1000) - 1);
	NVIC_SetPriority(SysTick_IRQn, VARIANT_PRIO_SYSTICK);

	initClocks();

	// Configure all pins as inputs (no pull-up/down)
	for (unsigned i = 0; i < PINS_COUNT; i++)
		pinMode(i, INPUT);

	delay(50);

	i2c.begin(400000);

	initADC();

	f_mount(&fs, "", 1);
}

