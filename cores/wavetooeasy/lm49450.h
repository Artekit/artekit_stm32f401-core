/***************************************************************************
 * Artekit Wavetooeasy
 * https://www.artekit.eu/products/devboards/wavetooeasy
 *
 * Written by Ivan Meleca
 * Copyright (c) 2021 Artekit Labs
 * https://www.artekit.eu

### lm49450.h

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

#ifndef LM49450_H_
#define LM49450_H_

#include <Arduino.h>

bool lm49450Initialize(uint32_t fs);
bool lm49450Disable();
bool lm49450Mute();
bool lm49450Unmute();
void lm49450DumpRegisters(UARTClass* serial);
bool lm49450ReadRegister(uint8_t reg, uint8_t* value);
bool lm49450WriteRegister(uint8_t reg, uint8_t value, bool verify = true);
bool lm49450SetSpeakersVolume(float db);
bool lm49450SetHeadphoneVolume(float db);
float lm49450GetSpeakersVolume();
float lm49450GetHeadphoneVolume();

#endif /* LM49450_H_ */
