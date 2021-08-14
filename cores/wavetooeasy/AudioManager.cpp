/***************************************************************************
 * Artekit Wavetooeasy
 * https://www.artekit.eu/products/devboards/wavetooeasy
 *
 * Written by Ivan Meleca
 * Copyright (c) 2021 Artekit Labs
 * https://www.artekit.eu

### AudioManager.cpp

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

#include "Arduino.h"
#include "lm49450.h"
#include <string.h>

extern uint8_t fs_busy(void);

#define MIN(a,b) ((a) < (b) ? a : b)
#define MAX(a,b) ((a) > (b) ? a : b)

AUDIO_STAT(uint32_t mix_time = 0);
AUDIO_STAT(uint32_t mix_ticks = 0);
AUDIO_STAT(uint32_t update_time = 0);
AUDIO_STAT(uint32_t update_tick = 0);

volatile uint32_t empty_source = 0;

AudioManager::AudioManager()
{
	analyze_callback = NULL;
	mix_callback = NULL;
	sources_list = NULL;
	initialized = false;
	playing = false;
	sending_mclk = false;
	play_buffer = update_buffer = NULL;
	bits_per_sample = 0;
	sample_rate = 0;
	update_pending = false;
	idling = false;
	source_count = 0;
	output_samples = 0;

#if AUDIO_STATS
	miss_count = 0;
	miss_time = 0;
	miss_start = 0;
	samples_played = 0;
	irq_interval_time = 0;
	irq_interval = 0;
	max_irq_interval = 0;
#endif // AUDIO_STATS

	buffer1 = buffer2 = NULL;;
	buffer_size = 0;
}

bool AudioManager::begin(uint32_t fs, uint8_t bps)
{
	if (initialized)
		return true;

	sample_rate = fs;
	bits_per_sample = bps;

	if (!allocateOutputBuffers())
		return false;

	output_buffers[0].buffer = (uint32_t*) buffer1;

	if ((uint32_t) buffer1 & 0x03)
		output_buffers[0].buffer = (uint32_t*) ((uint8_t*) output_buffers[0].buffer + (4 - ((uint32_t) buffer1 & 0x03)));

	output_buffers[1].buffer = (uint32_t*) buffer2;

	if ((uint32_t) buffer2 & 0x03)
		output_buffers[1].buffer = (uint32_t*) ((uint8_t*) output_buffers[1].buffer + (4 - ((uint32_t) buffer2 & 0x03)));

	play_buffer = &output_buffers[0];
	update_buffer = &output_buffers[1];

	// Configure PendSV to the lowest priority
	NVIC_SetPriority(PendSV_IRQn, VARIANT_PRIO_PENDSV);
	NVIC_EnableIRQ(PendSV_IRQn);

	if (!initI2S(fs, bps))
		return false;

	if (!initCodec())
		return false;

	delay(100);

	onI2STxFinished();

	if (!unmute())
		return false;
	
	initialized = true;
	return true;
}

void AudioManager::end()
{
	GPIO_InitTypeDef GPIO_InitStruct;

	if (initialized)
	{
		mute();
		lm49450Disable();

		// Disable DMA1 Stream4 Transmission Complete interrupt
		DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, DISABLE);

		NVIC_DisableIRQ(DMA1_Stream4_IRQn);

		SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, DISABLE);

		I2S_Cmd(SPI2, DISABLE);

		// De-initialize SPI2
		SPI_I2S_DeInit(SPI2);

		deallocateOutputBuffers();

		// Reset pins
		// PB12, PB13, PB14, PB15 and PC6
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOB, &GPIO_InitStruct);

		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
		GPIO_Init(GPIOC, &GPIO_InitStruct);

		initialized = false;
	}
}

bool AudioManager::allocateOutputBuffers()
{
	// Calculate output samples according to the target latency
	output_samples = (TARGET_LATENCY_US * sample_rate) / 1000000;

	// Allocate two stereo buffers (and padding)
	uint32_t needed = output_samples * 2 * (bits_per_sample / 8) + 16;

	if (buffer_size == needed)
		return true;

	deallocateOutputBuffers();

	buffer1 = (uint8_t*) malloc(needed);
	if (!buffer1)
		return false;

	buffer2 = (uint8_t*) malloc(needed);
	if (!buffer2)
	{
		free(buffer1);
		return false;
	}

	buffer_size = needed;
	return true;
}

void AudioManager::deallocateOutputBuffers()
{
	if (buffer1)
		free(buffer1);

	if (buffer2)
		free(buffer2);

	buffer_size = 0;
	buffer1 = NULL;
	buffer2 = NULL;
}

bool AudioManager::initCodec()
{
	// Initialize codec
	return lm49450Initialize(sample_rate);
}

bool AudioManager::initI2S(uint32_t fs, uint8_t bps)
{
	I2S_InitTypeDef I2S_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	DMA_InitTypeDef DMA_InitStruct;
	uint16_t plli2sn;
	uint8_t plli2sr;

	if (playing)
		return false;

	// Initialize peripherals clocks
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
	RCC_I2SCLKConfig(RCC_I2S2CLKSource_PLLI2S);

	// Initialize PLLI2S clock
	switch (fs)
	{
		case 22050:
			plli2sn = 192; plli2sr = 4;
			I2S_InitStruct.I2S_AudioFreq = I2S_AudioFreq_22k;
			break;
		case 32000:
			plli2sn = 213; plli2sr = 4;
			I2S_InitStruct.I2S_AudioFreq = I2S_AudioFreq_32k;
			break;
		case 44100:
			plli2sn = 192; plli2sr = 2;
			I2S_InitStruct.I2S_AudioFreq = I2S_AudioFreq_44k;
			break;
		case 48000:
			plli2sn = 172; plli2sr = 2;
			I2S_InitStruct.I2S_AudioFreq = I2S_AudioFreq_48k;
			break;
		case 96000:
			plli2sn = 393; plli2sr = 4;
			I2S_InitStruct.I2S_AudioFreq = I2S_AudioFreq_96k;
			break;
		default:
			return false;
	}
	
	RCC_PLLI2SConfig(plli2sn, plli2sr);
	RCC_PLLI2SCmd(ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	// Deinitialize SPI2
	SPI_I2S_DeInit(SPI2);

	// Alternate function
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_SPI2);

	// Configure pins
	// PB12, PB13, PB15 and PC6
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Low_Speed;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	// Configure I2S
	I2S_InitStruct.I2S_Mode = I2S_Mode_MasterTx;
	I2S_InitStruct.I2S_CPOL = I2S_CPOL_Low;
	I2S_InitStruct.I2S_MCLKOutput = I2S_MCLKOutput_Enable;
	I2S_InitStruct.I2S_Standard = I2S_Standard_Phillips;

	switch (bps)
	{
		case 8:
		case 16:
			I2S_InitStruct.I2S_DataFormat = I2S_DataFormat_16b;
			mix_callback = mix16Multipass;
			break;
			
		case 24:
			I2S_InitStruct.I2S_DataFormat = I2S_DataFormat_24b;
			mix_callback = mix24;
			break;
		
		default:
			return false;
	}

	I2S_Init(SPI2, &I2S_InitStruct);

	// DMA
	DMA_InitStruct.DMA_Channel = DMA_Channel_0;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) &SPI2->DR;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t) 0; 		// to be filled in AudioManager::startDMA()
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStruct.DMA_BufferSize = (uint32_t) 0;			// to be filled in AudioManager::startDMA()
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_INC16;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream4, &DMA_InitStruct);

	// Enable DMA1 Stream4 Transmission Complete interrupt
	DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);

	// Enable I2S DMA transmission requests
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);

	NVIC_SetPriority(DMA1_Stream4_IRQn, VARIANT_PRIO_I2S_DMA);
	NVIC_EnableIRQ(DMA1_Stream4_IRQn);

	I2S_Cmd(SPI2, ENABLE);

	return true;
}

void AudioManager::triggerUpdate()
{
	if ((SCB->SHCSR & SCB_SHCSR_PENDSVACT_Msk) == 0)
		Activate_PendSV();
}

void AudioManager::update()
{
	AUDIO_STAT(update_tick = micros());

	// Make a list of sources, ordered by quantity of samples available
	AudioSource* ptr = sources_list;
	AudioSource* list = NULL;

	while(ptr)
	{
		if (ptr->playing())
		{
			if (list != NULL)
			{
				// Scan the list and insert
				AudioSource* list_ptr = list;
				AudioSource* prev = NULL;
				uint32_t samples_left = ptr->getSamplesLeft();

				while (list_ptr)
				{
					if (samples_left <= list_ptr->getSamplesLeft())
					{
						if (prev)
						{
							ptr->next_to_udpdate = prev->next_to_udpdate;
							prev->next_to_udpdate = ptr;
						} else {
							ptr->next_to_udpdate = list;
							list = ptr;
						}
						break;
					} else {
						prev = list_ptr;
						list_ptr = list_ptr->next_to_udpdate;
					}
				}

				// Last in the list
				if (!list_ptr)
				{
					prev->next_to_udpdate = ptr;
					ptr->next_to_udpdate = NULL;
				}
			} else {
				list = ptr;
				ptr->next_to_udpdate = NULL;
			}
		}

		ptr = ptr->next_in_list;
	}

	ptr = list;

	while (ptr)
	{
		switch (ptr->update())
		{
			case SourceIdling:
			case SourceUpdated:
				break;

			case SourceUpdatedButPending:
				break;

			case SourceRemove:
			case UpdateError:
				ptr->stop();
				break;
		}

		ptr = ptr->next_to_udpdate;
	}

	AUDIO_STAT(update_time = micros() - update_tick);
}

void AudioManager::mix()
{
	AudioSource* ptr = sources_list;
	AudioSource* mix_list = NULL;

	update_buffer->ready = false;

	while (ptr)
	{
		if (ptr->playing())// && ptr->getSamplesLeft())
		{
			ptr->setNextToMix(mix_list);
			mix_list = ptr;
		}

		ptr = ptr->getNextInList();
	}

	if (mix_list)
	{
		update_buffer->buffer_samples = output_samples;
		update_buffer->mixed_samples = 0;
		if ((mix_callback)(update_buffer, mix_list))
		{
			if (analyze_callback)
				(analyze_callback)(update_buffer);

			update_buffer->ready = true;
		}
	}
}

void AudioManager::onI2STxFinished()
{
	AUDIO_STAT(if (irq_interval_time) irq_interval = micros() - irq_interval_time);
	AUDIO_STAT(irq_interval_time = micros());
	AUDIO_STAT(if (irq_interval >= max_irq_interval) max_irq_interval = irq_interval);

	if (update_buffer->ready)
	{
		// Switch buffers
		switchBuffers();

		AUDIO_STAT(if (idling && source_count && miss_start) miss_time = GetTickCount() - miss_start);
		idling = false;
	} else {
		if (!idling)
		{
			AUDIO_STAT(if (source_count) miss_count++);
			AUDIO_STAT(if (source_count) miss_start = GetTickCount());
			memset(play_buffer->buffer, 0, (sample_rate / 1000) * (bits_per_sample / 8) * 2);
			play_buffer->mixed_samples = sample_rate / 1000;
			idling = true;
		}
	}

	// Send current playing buffer
	startDMA(play_buffer);

	// Mix the updating buffer
	mix();
}

void AudioManager::switchBuffers()
{
	OUTPUT_BUFFER* ptr;
	
	ptr = play_buffer;
	play_buffer = update_buffer;
	update_buffer = ptr;
}

void AudioManager::startDMA(OUTPUT_BUFFER* output_buffer)
{
	// Disable DMA
	DMA1_Stream4->CR = 0;
	SPI2->CR2 &= (uint16_t)~SPI_I2S_DMAReq_Tx;

	// Configure source, destination and size
	DMA1->HIFCR = 0x3D;
	DMA1_Stream4->M0AR = (uint32_t) output_buffer->buffer;
	DMA1_Stream4->PAR = (uint32_t) &SPI2->DR;
	DMA1_Stream4->NDTR = output_buffer->mixed_samples * 2;

	// Enable DMA
	DMA1_Stream4->FCR = 0;
	DMA1_Stream4->CR |= DMA_SxCR_EN | DMA_SxCR_MSIZE_0 | DMA_SxCR_DIR_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_TCIE;

	SPI2->CR2 |= SPI_I2S_DMAReq_Tx;

	AUDIO_STAT(samples_played += output_buffer->mixed_samples);
}

void AudioManager::setAnalyzeCallback(audioAnalyzeCallback* analyze)
{
	__disable_irq();
	analyze_callback = analyze;
	__enable_irq();
}

void AudioManager::setMixingFunction(audioMixCallback* mix)
{
	__disable_irq();
	mix_callback = mix;
	__enable_irq();
}

bool AudioManager::mix16Multipass(OUTPUT_BUFFER* buf, AudioSource* sources)
{
	AudioSource* source = sources;
	uint32_t count;
	uint32_t samples;

	AUDIO_STAT(mix_ticks = micros());

	while (source)
	{
		count = 0;
		samples = source->mixingStarts(Audio.getOutputSamples());
		samples = MIN(samples, buf->buffer_samples);

		if (!samples)
		{
			empty_source++;
			source->mixingEnded(0);
			source = source->getNextToMix();
			continue;
		}

		void* samples_ptr = source->getSamplesPtr();

		if (source->isStereo())
		{
			uint32_t* ptr = (uint32_t*) samples_ptr;

			if (buf->mixed_samples == 0)
			{
				while (samples)
				{
					buf->buffer[count++] = *(ptr++);
					samples--;
				}
			} else {
				while (count < samples && count < buf->mixed_samples)
				{
					buf->buffer[count] = __QADD16(buf->buffer[count], *(ptr++));
					count++;
				}

				while (count < samples)
					buf->buffer[count++] = *(ptr++);
			}
		} else {
			uint16_t* ptr = (uint16_t*) samples_ptr;

			if (buf->mixed_samples == 0)
			{
				while (samples--)
				{
					buf->buffer[count++] = __PKHBT(*ptr, *ptr, 16);
					ptr++;
				}
			} else {
				while (count < samples && count < buf->mixed_samples)
				{
					buf->buffer[count] = __QADD16(buf->buffer[count], __PKHBT(*ptr, *ptr, 16));
					ptr++;
					count++;
				}

				while (count < samples)
				{
					buf->buffer[count++] = __PKHBT(*ptr, *ptr, 16);
					ptr++;
				}
			}
		}

		source->mixingEnded(count);

		if (buf->mixed_samples < count)
			buf->mixed_samples = count;

		source = source->getNextToMix();
	}

	AUDIO_STAT(mix_time = micros() - mix_ticks);
	return buf->mixed_samples > 0;
}

bool AudioManager::mix24(OUTPUT_BUFFER* buf, AudioSource* pb)
{
	/* TBD */
	UNUSED(buf);
	UNUSED(pb);
	return false;
}

bool AudioManager::checkSourceFormat(AudioSource* source)
{
	if (source->sampleRate() != sample_rate)
		return false;
	
	if (source->bitsPerSample() != bits_per_sample)
		return false;

	return true;
}

bool AudioManager::addSource(AudioSource* source)
{
	if (!checkSourceFormat(source))
		return false;
	
	__disable_irq();

	if (sources_list == NULL)
	{
		sources_list = source;
		sources_list->setNextInList(NULL);
	} else {
		AudioSource* ptr = sources_list;

		while (ptr)
		{
			if (ptr->getNextInList() == NULL)
			{
				ptr->setNextInList(source);
				break;
			}
			ptr = ptr->getNextInList();
		}
	}
	
	source_count++;
	__enable_irq();

	return true;
}

bool AudioManager::removeSource(AudioSource* source)
{
	if (!source_count)
		return false;
	
	__disable_irq();

	if (sources_list == source)
	{
		sources_list = source->getNextInList();
		source->setNextInList(NULL);
	} else {
		AudioSource* ptr = sources_list;

		while (ptr)
		{
			if (ptr->getNextInList() == source)
			{
				ptr->setNextInList(source->getNextInList());
				source->setNextInList(NULL);
				break;
			}
			ptr = ptr->getNextInList();
		}

		if (ptr == NULL)
		{
			__enable_irq();
			return false;
		}
	}

	source_count--;
	__enable_irq();

	return true;
}

bool AudioManager::mute()
{
	return lm49450Mute();
}

bool AudioManager::unmute()
{
	return lm49450Unmute();
}

bool AudioManager::setSpeakersVolume(float db)
{
	return lm49450SetSpeakersVolume(db);
}

bool AudioManager::setHeadphoneVolume(float db)
{
	return lm49450SetHeadphoneVolume(db);
}

float AudioManager::getSpeakersVolume()
{
	return lm49450GetSpeakersVolume();
}

float AudioManager::getHeadphoneVolume()
{
	return lm49450GetHeadphoneVolume();
}

#if AUDIO_STATS
void AudioManager::printDebug(UARTClass* uart)
{
	uart->println("\r\nAudio statistics");
	uart->println("----------------\r\n");

	uart->print("Samples played: ");
	uart->println(samples_played);
	uart->print("Buffers not ready: ");
	uart->println(miss_count);
	uart->print("Buffers not ready (time): ");
	uart->print(miss_time);
	uart->println(" ms");
	uart->print("DMA IRQ interval: ");
	uart->print(irq_interval);
	uart->println(" uS");
	uart->print("Max. DMA IRQ interval: ");
	uart->print(max_irq_interval);
	uart->println(" uS");
	uart->print("Update() time: ");
	uart->print(update_time);
	uart->println(" uS");
}
#endif // AUDIO_STATS

void DMA1_Stream4_IRQHandler(void)
{
	Audio.onI2STxFinished();
}

void PendSV_Handler(void)
{
	// For now, this guards AudioSources that depend on the file system to update.
	// TBD: move this calls to each type of AudioSource and return a coherent value indicating that
	// the update is still pending.
	if (fs_busy() || sdIsBusy())
	{
		Audio.update_pending = true;
		return;
	}

	Audio.update_pending = false;
	Audio.update();
}
