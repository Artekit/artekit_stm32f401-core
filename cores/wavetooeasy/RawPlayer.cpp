/***************************************************************************
 * Artekit Wavetooeasy
 * https://www.artekit.eu/products/devboards/wavetooeasy
 *
 * Written by Ivan Meleca
 * Copyright (c) 2021 Artekit Labs
 * https://www.artekit.eu

### RawPlayer.cpp

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
#include "RawPlayer.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern "C" uint32_t micros( void ) ;

RawPlayer::RawPlayer()
{
	play_mode = PlayModeNormal;
	header_size = 0;
	update_requested = false;
	samples_end = samples_data = samples_alloc = NULL;
	read_ptr = write_ptr = NULL;
	samples_available = 0;
	total_samples = 0;
	alloc_size = 0;
	update_threshold = 0;
	preloaded = false;
	read_time = max_read_time = 0;
}

RawPlayer::~RawPlayer()
{
}

bool RawPlayer::begin(uint32_t fs, uint8_t bps, bool mono, uint32_t hdrsize)
{
	// Remember the header size
	header_size = hdrsize;

	// sample_size is calculated in here
	AudioSource::begin(fs, bps, mono);

	// Allocate circular buffer
	return allocate();
}

bool RawPlayer::allocate()
{
	// TBD: get a proper buffer size
	uint32_t samples = Audio.getOutputSamples() * 20;
	uint32_t new_size = samples * sample_size;

	if (samples_alloc)
	{
		// Check if the requested size can fit in the buffer we have
		if (new_size > alloc_size)
			// It won't
			deallocate();
	}

	if (!samples_alloc)
	{
		samples_alloc = (uint8_t*) malloc(new_size + 4);
		if (!samples_alloc)
			return false;

		alloc_size = new_size;
		total_samples = samples;

		// Check alignment
		samples_data = samples_alloc;
		if ((uint32_t) samples_data & 0x03)
			samples_data += (4 - ((uint32_t) samples_alloc & 0x03));

		samples_end = samples_data + samples * sample_size;
	}

	read_ptr = write_ptr = samples_data;
	samples_available = 0;
	update_threshold = total_samples / 2;
	return true;
}

void RawPlayer::end()
{
	stop();
	if (audio_file.isOpened())
		audio_file.close();
	deallocate();
}

uint32_t RawPlayer::getSamplesLeft()
{
	return samples_available;
}

UpdateResult RawPlayer::update(AudioFileHelper* file)
{
	if (samples_available > update_threshold)
		return SourceUpdated;

	// Read from file
	uint32_t time = micros();
	uint32_t read = file->fillBuffer(write_ptr, update_threshold);
	if ((read != update_threshold) && !file->eofReached())
		// Something went wrong
		return UpdateError;
	read_time = micros() - time;

	if (read_time > max_read_time)
		max_read_time = read_time;

	// Update write pointer
	write_ptr += read * sample_size;
	if (write_ptr == samples_end)
		write_ptr = samples_data;

	// Make samples available right away
	__disable_irq();
	samples_available += read;
	__enable_irq();

	updates_done++;

	return SourceUpdated;
}

UpdateResult RawPlayer::update()
{
	// Check if we have reached EOF
	if (!samples_available && audio_file.eofReached())
		// Return SourceRemove so Audio can remove this AudioSource from the list.
		return SourceRemove;

	return update(&audio_file);
}

bool RawPlayer::refill(AudioFileHelper* file)
{
	write_ptr = read_ptr = samples_data;
	samples_available = 0;

	if (!file->rewind())
		return false;

	uint32_t read_samples = file->fillBuffer(samples_data, total_samples);

	if (!read_samples)
		return false;

	samples_available = read_samples;
	return true;
}

bool RawPlayer::refill()
{
	if (refill(&audio_file))
		return true;

	return false;
}

bool RawPlayer::doPlay(PlayMode mode)
{
	updates_done = 0;
	max_read_time = 0;

	play_mode = mode;

	if (!preloaded)
	{
		if (!refill())
			return false;
	}

	if (!Audio.addSource(this))
	{
		stop();
		return false;
	}

	status = AudioSourcePlaying;

	if (play_mode == PlayModeBlocking)
		while (playing());

	return true;
}

bool RawPlayer::play(const char* filename, PlayMode mode)
{
	if (status != AudioSourceStopped)
		stop();

	audio_file.close();

	if (!audio_file.openRaw(filename, sample_size, header_size, mode == PlayModeLoop))
		return false;

	return doPlay(mode);
}

bool RawPlayer::replay()
{
	if (status == AudioSourceStopped)
		return false;

	if (!audio_file.rewind())
		return false;

	if (samples_available == 0)
		return refill();

	return true;
}

bool RawPlayer::stop()
{
	status = AudioSourceStopped;
	Audio.removeSource(this);
	return true;
}

uint32_t RawPlayer::mixingStarts(uint32_t samples)
{
	if (samples_available < samples)
		samples = samples_available;

	changeVolume(read_ptr, samples);
	return samples;
}

void RawPlayer::mixingEnded(uint32_t samples)
{
	read_ptr += samples * sample_size;
	if (read_ptr == samples_end)
		read_ptr = samples_data;

	samples_available -= samples;

	if (!samples_available && audio_file.eofReached())
		stop();
	else if (samples_available <= update_threshold)
		Audio.triggerUpdate();
}

void RawPlayer::dump(UARTClass* serial)
{
	if (playing()) serial->print("[P] ");
	else serial->print("[S] ");

	serial->print(updates_done);
	serial->print("\t");

	serial->print(samples_available);
	serial->print("/");
	serial->print(total_samples);

	serial->print("\t");
	serial->print(read_time);
	serial->print("-");
	serial->println(max_read_time);
}
