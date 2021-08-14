/***************************************************************************
 * Artekit Wavetooeasy
 * https://www.artekit.eu/products/devboards/wavetooeasy
 *
 * Written by Ivan Meleca
 * Copyright (c) 2021 Artekit Labs
 * https://www.artekit.eu

### variant.h

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

#ifndef __VARIANT_H__
#define __VARIANT_H__

#include "Arduino.h"
#ifdef __cplusplus
#include "UARTClass.h"
#endif

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

#define PINS_COUNT 			19
#define NUM_DIGITAL_PINS 	19
#define NUM_ANALOG_INPUTS 	0
#define NUM_IRQ_PINS			16
#define NUM_MAX_EXT_IRQ		16

#define digitalPinToPort(P)        ( g_APinDescription[P].pin_port )
#define digitalPinToBitMask(P)     ( g_APinDescription[P].pin_mask )
#define portOutputRegister(port)   ( &(port->ODR) )
#define portInputRegister(port)    ( &(port->IDR) )
#define digitalPinHasPWM(P)        ( g_APinDescription[P].timer != NULL )

static const uint8_t CHANNEL1	= 0;
static const uint8_t CHANNEL2 	= 1;
static const uint8_t CHANNEL3 	= 2;
static const uint8_t CHANNEL4 	= 3;
static const uint8_t CHANNEL5 	= 4;
static const uint8_t CHANNEL6 	= 5;
static const uint8_t CHANNEL7 	= 6;
static const uint8_t CHANNEL8 	= 7;
static const uint8_t CHANNEL9 	= 8;
static const uint8_t CHANNEL10 	= 9;
static const uint8_t CHANNEL11 	= 10;
static const uint8_t CHANNEL12 	= 11;
static const uint8_t CHANNEL13 	= 12;
static const uint8_t CHANNEL14 	= 13;
static const uint8_t CHANNEL15 	= 14;
static const uint8_t CHANNEL16 	= 15;
static const uint8_t LATCH 		= 16;
static const uint8_t LED1 		= 17;
static const uint8_t LED2 		= 18;

#define VARIANT_PRIO_UART			0
#define VARIANT_PRIO_I2S_DMA			1
#define VARIANT_PRIO_SDIO			2
#define VARIANT_PRIO_SYSTICK			4
#define VARIANT_PRIO_ST				6
#define VARIANT_PRIO_USER_EXTI		10
#define VARIANT_PRIO_PENDSV			255

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern UARTClass Serial;
#endif

#endif /* __VARIANT_H__ */
