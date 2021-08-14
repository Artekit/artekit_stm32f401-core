/***************************************************************************
 * Artekit Wavetooeasy
 * https://www.artekit.eu/products/devboards/wavetooeasy
 *
 * Written by Ivan Meleca
 * Copyright (c) 2021 Artekit Labs
 * https://www.artekit.eu

### lm49450.cpp

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

#include "lm49450.h"

#define LM49450_ADDRESS				0xFA

#define LM49450_REG_MODE_CONTROL		0x00
#define LM49450_REG_CLOCK				0x01
#define LM49450_REG_PUMP_FREQ			0x02
#define LM49450_REG_I2S_MODE			0x03
#define LM49450_REG_I2S_CLOCK			0x04
#define LM49450_REG_HEADPHONE_3D_CTRL	0x05
#define LM49450_REG_SPEAKER_3D_CTRL		0x06
#define LM49450_REG_HEADPHONE_VOL_CTRL	0x07
#define LM49450_REG_SPEAKER_VOL_CTRL	0x08
#define LM49450_REG_CMP_0_LSB			0x09
#define LM49450_REG_CMP_0_MSB			0x0A
#define LM49450_REG_CMP_1_LSB			0x0B
#define LM49450_REG_CMP_1_MSB			0x0C
#define LM49450_REG_CMP_2_LSB			0x0D
#define LM49450_REG_CMP_2_MSB			0x0E

static const float hpVolumeTable[] = {
		-59, -48, -40.5f, -34.5f, -30, -27, -24, -21, -18, -15, -13.5f,
		-12, -10.5f, -9, -7.5f, -6, -4.5f, -3, -1.5f, 0, 1.5f, 3, 4.5f,
		6, 7.5f, 9, 10.5f, 12, 13.5f, 15, 16.5f, 18
};

static const float spkrVolumeTable[] = {
		-53, -42, -34.5f, -28.5f, -24, -21, -18, -15, -12, -9, -7.5f,
		-6, -4.5f, -3, -1.5f, 0, 1.5f, 3, 4.5f, 6, 7.5f, 9, 10.5f, 12,
		13.5f, 15, 16.5f, 18, 19.5f, 21, 22.5f, 24

};

bool lm49450ReadRegister(uint8_t reg, uint8_t* value)
{
	return i2c.readRegister(LM49450_ADDRESS, reg, 1, value, 1);
}

bool lm49450WriteRegister(uint8_t reg, uint8_t value, bool verify)
{
	uint8_t read_value;

	if (!i2c.writeRegister(LM49450_ADDRESS, reg, 1, &value, 1))
		return false;

	if (verify)
	{
		if (!lm49450ReadRegister(reg, &read_value) || read_value != value)
			return false;
	}

	return true;
}

static bool lm49450Enable()
{
	uint8_t value;
	if (!lm49450ReadRegister(LM49450_REG_MODE_CONTROL, &value))
		return false;

	value |= 0x01;

	if (!lm49450WriteRegister(LM49450_REG_MODE_CONTROL, value))
		return false;

	return true;
}

bool lm49450Disable()
{
	uint8_t value;
	if (!lm49450ReadRegister(LM49450_REG_MODE_CONTROL, &value))
		return false;

	value &= ~0x01;

	if (!lm49450WriteRegister(LM49450_REG_MODE_CONTROL, value))
		return false;

	return true;
}

bool lm49450Initialize(uint32_t fs)
{
	uint8_t pump_charge = 0;

	if (!lm49450Disable())
		return false;

	// Set I2S to 16 bit, normal mode
	if (!lm49450WriteRegister(LM49450_REG_I2S_MODE, 0x00))
		return false;

	// Set headphone and speakers volume to 0dB
	if (!lm49450SetHeadphoneVolume(0))
		return false;

	if (!lm49450SetSpeakersVolume(0))
		return false;

	if (fs <= 82000)
	{
		// Set DAC mode to 01 (OSR = 128, CLK at B = 256fs) and mute
		if (!lm49450WriteRegister(LM49450_REG_MODE_CONTROL, 0x24))
			return false;

		switch (fs)
		{
			case 22050:	pump_charge = 33; break;	// 5.6448
			case 24000: pump_charge = 37; break;	// 6.144
			case 32000: pump_charge = 49; break;	// 8.192
			case 44100:	pump_charge = 68; break;	// 11.2896
			case 48000:	pump_charge = 75; break;	// 12.288
			default:	pump_charge = 0; break;		// bypass
		}
	} else {
		// Set DAC mode to 10 (OSR = 64, CLK at B = 128fs), mute
		if (!lm49450WriteRegister(LM49450_REG_MODE_CONTROL, 0x44))
			return false;

		// Set RDIV to 2
		if (!lm49450WriteRegister(LM49450_REG_CLOCK, 0x03))
			return false;

		switch (fs)
		{
			case 82000: pump_charge = 68; break;	// 11.2896
			case 96000: pump_charge = 75; break;	// 12.288
			default:	pump_charge = 0; break;		// bypass
		}
	}

	if (!lm49450WriteRegister(LM49450_REG_PUMP_FREQ, pump_charge))
		return false;

	return lm49450Enable();
}

bool lm49450Mute()
{
	uint8_t value;
	if (!lm49450ReadRegister(LM49450_REG_MODE_CONTROL, &value))
		return false;

	value |= 0x04;

	return lm49450WriteRegister(LM49450_REG_MODE_CONTROL, value);
}

bool lm49450Unmute()
{
	uint8_t value;
	if (!lm49450ReadRegister(LM49450_REG_MODE_CONTROL, &value))
		return false;

	value &= ~0x04;

	return lm49450WriteRegister(LM49450_REG_MODE_CONTROL, value);
}

void lm49450DumpRegisters(UARTClass* serial)
{
	for (uint32_t i = 0; i <= 0x0E; i++)
	{
		uint8_t value;
		if (lm49450ReadRegister(i, &value))
		{
			serial->print("Register 0x");
			serial->print(i, HEX);
			serial->print(" = 0x");
			serial->println(value, HEX);
		} else {
			serial->print("Error reading register 0x");
			serial->println(i, HEX);
		}
	}
}

bool lm49450SetSpeakersVolume(float db)
{
	uint8_t i;

	for (i = 0; i < sizeof(spkrVolumeTable) / sizeof(float); i++)
	{
		if (db < spkrVolumeTable[i])
			break;
	}

	if (i > 32)
		i = 32;

	return lm49450WriteRegister(LM49450_REG_SPEAKER_VOL_CTRL, i);
}

bool lm49450SetHeadphoneVolume(float db)
{
	uint8_t i;

	for (i = 0; i < sizeof(hpVolumeTable) / sizeof(float); i++)
	{
		if (db < hpVolumeTable[i])
			break;
	}

	if (i > 32)
		i = 32;

	return lm49450WriteRegister(LM49450_REG_HEADPHONE_VOL_CTRL, i);
}

float lm49450GetSpeakersVolume()
{
	uint8_t value;
	if (!lm49450ReadRegister(LM49450_REG_SPEAKER_VOL_CTRL, &value))
		return 0;

	if (value)
		value--;

	return spkrVolumeTable[value];
}

float lm49450GetHeadphoneVolume()
{
	uint8_t value;
	if (!lm49450ReadRegister(LM49450_REG_HEADPHONE_VOL_CTRL, &value))
		return 0;

	if (value)
		value--;

	return hpVolumeTable[value];
}
