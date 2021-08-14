/***************************************************************************
 * Artekit Wavetooeasy
 * https://www.artekit.eu/products/devboards/wavetooeasy
 *
 * Written by Ivan Meleca
 * Copyright (c) 2021 Artekit Labs
 * https://www.artekit.eu

### LowPower.cpp

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

#include "LowPower.h"
#include "ServiceTimer.h"

void enterLowPowerMode()
{
	// Unmount SD
	f_mount(NULL, "", 0);

	// Shutdown SD
	sdDeinitialize();

	// Service timer
	stDeinit();

	PWR_WakeUpPinCmd(ENABLE);

	delay(250);

	// Disable Systick
	NVIC_DisableIRQ(SysTick_IRQn);
	NVIC_ClearPendingIRQ(SysTick_IRQn);
	SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);

	__disable_irq();

	PWR->CR |= PWR_CR_CWUF | PWR_CR_CSBF;

	// Enter STAND-BY mode
	PWR_EnterSTANDBYMode();

	// We should never reach this code
	while (true);
}


