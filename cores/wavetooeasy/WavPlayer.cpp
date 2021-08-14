/***************************************************************************
 * Artekit Wavetooeasy
 * https://www.artekit.eu/products/devboards/wavetooeasy
 *
 * Written by Ivan Meleca
 * Copyright (c) 2021 Artekit Labs
 * https://www.artekit.eu

### WavPlayer.cpp

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

#include "AudioManager.h"
#include "WavPlayer.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

WavPlayer::WavPlayer()
{
}
	
WavPlayer::~WavPlayer()
{
}

bool WavPlayer::preload(const char* filename, PlayMode mode)
{
	// Has to be stopped
	if (status != AudioSourceStopped)
		return false;

	audio_file.close();

/*
	if (!audio_file.isOpened() ||
		strncasecmp(filename, audio_file.getFileName(), FF_MAX_LFN) != 0) // Check file name
	{
		if (!audio_file.openWav(filename, mode == PlayModeLoop))
			return false;
	}
*/

	if (!audio_file.openWav(filename, mode == PlayModeLoop))
		return false;

	if (!RawPlayer::begin(audio_file.getWavHeader()->format.sample_rate,
						  audio_file.getWavHeader()->format.bits_per_sample,
						  audio_file.getWavHeader()->format.channels == 1,
						  audio_file.getHeaderSize()))
	{
		audio_file.close();
		return false;
	}

	if (!refill())
		return false;

	preloaded = true;
	return true;
}

bool WavPlayer::play(const char* filename, PlayMode mode)
{
	if (status != AudioSourceStopped)
		stop();

	audio_file.close();

	/*
	if (!audio_file.isOpened() ||
		strncasecmp(filename, audio_file.getFileName(), FF_MAX_LFN) != 0) // Check file name
	{
		if (!audio_file.openWav(filename, mode == PlayModeLoop))
			return false;
	}
	*/

	if (!audio_file.openWav(filename, mode == PlayModeLoop))
		return false;

	return beginAndPlay(mode);
}

bool WavPlayer::beginAndPlay(PlayMode mode)
{
	if (!preloaded)
	{
		if (!RawPlayer::begin(audio_file.getWavHeader()->format.sample_rate,
							  audio_file.getWavHeader()->format.bits_per_sample,
							  audio_file.getWavHeader()->format.channels == 1,
							  audio_file.getHeaderSize()))
		{
			audio_file.close();
			return false;
		}
	}

	return RawPlayer::doPlay(mode);
}
