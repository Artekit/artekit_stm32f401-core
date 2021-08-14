/***************************************************************************
 * Artekit Wavetooeasy
 * https://www.artekit.eu/products/devboards/wavetooeasy
 *
 * Written by Ivan Meleca
 * Copyright (c) 2021 Artekit Labs
 * https://www.artekit.eu

### Audio.h

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

#ifndef __AUDIO_H__
#define __AUDIO_H__

#include <stm32f4xx.h>
#include "ff.h"
#include "AudioSource.h"
#include "UARTClass.h"

#define Activate_PendSV() SCB->ICSR = SCB->ICSR | SCB_ICSR_PENDSVSET_Msk

#ifndef AUDIO_STATS
#define AUDIO_STATS		1
#endif

#if AUDIO_STATS
#define AUDIO_STAT(x)	x
#else
#define AUDIO_STAT(x)
#endif // AUDIO_STATS

#define TARGET_LATENCY_US		1000
#define MAX_OUTPUT_BUFFERS		2

typedef struct _output_buffer
{
	uint32_t* buffer;
	uint32_t buffer_samples;
	uint32_t mixed_samples;
	volatile bool ready;

} OUTPUT_BUFFER;

typedef struct _i2s_buffer
{
	uint8_t* buffer;
	uint32_t samples;
} I2S_BUFFER;

typedef bool (audioMixCallback)(OUTPUT_BUFFER*, AudioSource*);
typedef void (audioAnalyzeCallback)(OUTPUT_BUFFER*);

extern "C" void DMA1_Stream4_IRQHandler(void);
extern "C" void PendSV_Handler(void);

class AudioManager
{
	friend void DMA1_Stream4_IRQHandler(void);
	friend void PendSV_Handler(void);

public:
	void userConfig()
	{

	}

	bool begin(uint32_t fs = 22050, uint8_t bps = 16);
	void end();
	bool addSource(AudioSource* source);
	bool removeSource(AudioSource* source);
	bool mute();
	bool unmute();
	bool isPlaying() { return getSourcesCount() > 0; }
	bool setSpeakersVolume(float db);
	bool setHeadphoneVolume(float db);
	float getSpeakersVolume();
	float getHeadphoneVolume();
	inline bool updatePending() { return update_pending; }
	void triggerUpdate();
	void setAnalyzeCallback(audioAnalyzeCallback* analyze);
	void setMixingFunction(audioMixCallback* mix);
	inline uint32_t getOutputSamples() { return output_samples; }
	inline uint32_t getSampleRate() { return sample_rate; }
	inline uint32_t getSourcesCount() { return source_count; }

	static AudioManager& instance()
	{
		static AudioManager singleton;
		return singleton;
	}

#if AUDIO_STATS
	void printDebug(UARTClass* uart);
#endif // AUDIO_STATS

protected:
	AudioManager();
	void update();
	void mix();
	void onI2STxFinished();
	bool initI2S(uint32_t fs, uint8_t bps);
	bool initCodec();

	bool checkSourceFormat(AudioSource* source);
	inline void switchBuffers();
	inline void startDMA(OUTPUT_BUFFER* output_buffer);
	static bool mix16Multipass(OUTPUT_BUFFER* buf, AudioSource* sources);// __attribute__ ((optimize(3)));
	static bool mix24(OUTPUT_BUFFER* buf, AudioSource* sources);

	bool allocateOutputBuffers();
	void deallocateOutputBuffers();
	uint8_t* buffer1;
	uint8_t* buffer2;
	uint32_t buffer_size;

	OUTPUT_BUFFER output_buffers[2];
	OUTPUT_BUFFER* play_buffer;
	OUTPUT_BUFFER* update_buffer;
	uint8_t bits_per_sample;
	uint32_t sample_rate;
	bool initialized;
	bool sending_mclk;
	bool idling;
	volatile bool playing;
	volatile bool update_pending;

#if AUDIO_STATS
	uint32_t miss_count;
	uint32_t miss_start;
	uint32_t miss_time;
	uint32_t samples_played;
	uint32_t irq_interval_time;
	uint32_t irq_interval;
	uint32_t max_irq_interval;
#endif // AUDIO_STATS

private:
	uint32_t output_samples;

	audioMixCallback* mix_callback;
	audioAnalyzeCallback* analyze_callback;

	volatile uint8_t source_count;
	AudioSource* sources_list;
};

#ifdef __cplusplus
extern "C"
{
#endif
	
extern AudioManager Audio;

#ifdef __cplusplus
}
#endif

#endif /* __AUDIO_H__ */
