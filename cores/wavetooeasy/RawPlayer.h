/***************************************************************************
 * Artekit Wavetooeasy
 * https://www.artekit.eu/products/devboards/wavetooeasy
 *
 * Written by Ivan Meleca
 * Copyright (c) 2021 Artekit Labs
 * https://www.artekit.eu

### RawPlayer.h

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

#ifndef __RAWPLAYER_H__
#define __RAWPLAYER_H__

#include <stm32f4xx.h>
#include <ff.h>
#include "AudioSource.h"
#include "AudioFileHelper.h"
#include "UARTClass.h"

enum PlayMode
{
	PlayModeNormal = 0,
	PlayModeLoop,
	PlayModeBlocking
};

class RawPlayer : public AudioSource
{
public:
	RawPlayer();
	~RawPlayer();

	bool begin(uint32_t fs, uint8_t bps, bool mono, uint32_t hdrsize);
	void end();
	bool play(const char* filename, PlayMode mode = PlayModeNormal);
	// bool playRandom(const char* filename, const char* ext, uint32_t min, uint32_t max, PlayMode mode = PlayModeNormal);
	bool replay();
	bool stop();
	char* getFileName();
	UpdateResult update();
	uint32_t getSamplesLeft();
	uint32_t duration() { return audio_file.getDuration(); }
	inline void* getSamplesPtr() { return read_ptr;	}
	void dump(UARTClass* serial);

protected:

	bool doPlay(PlayMode mode);
	uint32_t mixingStarts(uint32_t samples);
	void mixingEnded(uint32_t samples);
	bool refill();
	bool refill(AudioFileHelper* file);
	UpdateResult update(AudioFileHelper* file);

	AudioFileHelper audio_file;
	PlayMode play_mode;
	uint32_t header_size;
	bool update_requested;
	bool preloaded;

	bool allocate();
	void deallocate()
	{
		if (samples_alloc)
			free(samples_alloc);

		samples_data = samples_alloc = NULL;
		read_ptr = write_ptr = NULL;
		samples_available = 0;
		total_samples = 0;
		alloc_size = 0;
	}

	uint8_t* samples_data;
	uint8_t* samples_end;
	uint8_t* samples_alloc;
	uint8_t* read_ptr;
	uint8_t* write_ptr;
	volatile uint32_t samples_available;
	uint32_t updates_done;
	uint32_t total_samples;
	uint32_t alloc_size;
	uint32_t update_threshold;
	uint32_t read_time;
	uint32_t max_read_time;
};

#endif /* __RAWPLAYER_H__ */
