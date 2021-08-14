/***************************************************************************
 * Artekit STM32F401 bootloader
 *
 * Written by Ivan Meleca
 * Copyright (c) 2021 Artekit Labs
 * https://www.artekit.eu
 *
 * Based on Bootloader_D21
 * Copyright Arduino srl (c) 2015
 *

### main.c

#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 3 of the License.
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

#include <stm32f4xx.h>
#include <string.h>
#include <ff.h>
#include <sdcard.h>

#define FLASH_STATUS_READY			0x01
#define FLASH_STATUS_ERROR			0x02
#define FLASH_STATUS_BUSY			0x03

#define CMD_SIGN_ON					0x01
#define CMD_SET_PARAMETER			0x02
#define CMD_GET_PARAMETER			0x03
#define CMD_LOAD_ADDRESS				0x06
#define CMD_ENTER_PROGMODE			0x10
#define CMD_LEAVE_PROGMODE			0x11
#define CMD_CHIP_ERASE				0x12
#define CMD_PROGRAM_FLASH			0x13
#define CMD_READ_FLASH				0x14
#define STATUS_CMD_OK 				0x00
#define STATUS_CMD_TOUT 				0x80

#define MESSAGE_START				0x1B
#define MESSAGE_MAX_LENGTH			512
#define USART_BUFFER_MAX_LENGTH		2048

#define BOOTLOADER_END_ADDR			0x08003FFF
#define FLASH_START_ADDR				0x08000000
#define FLASH_END_ADDR				0x0803FFFF
#define MAIN_PROGRAM_ADDR			0x08004000

#define FILE_BUFFER_SIZE				0x800
#define FIRMWARE_MAX_SIZE			(FLASH_END_ADDR - MAIN_PROGRAM_ADDR)

#if defined(BOOTLOADER_PROPBOARD)
#define SIGNATURE 					"AAKK-60E2-485F-B5B2-4F593B57FF21"
#elif defined(BOOTLOADER_WAVETOOEASY)
#define SIGNATURE 					"AAKK-71EA-EE5F-8F4E-88FFE568BCAE"
#else
#error "Please define a board"
#endif

static uint8_t tx_buffer[USART_BUFFER_MAX_LENGTH];
static uint32_t txwr = 0;
static uint32_t txrd = 0;
static volatile uint32_t txsize = 0;

static uint8_t rx_buffer[USART_BUFFER_MAX_LENGTH];
static uint32_t rxwr = 0;
static uint32_t rxrd = 0;
static volatile uint32_t rxsize = 0;

static uint8_t message_buffer[MESSAGE_MAX_LENGTH];
static uint32_t message_offset = 0;
static uint8_t next_seq = 1;
static uint8_t parameters[15] = { 0,1,1,1,0,3,3,0,0,0,0,0,0,1,0 };
static uint8_t signature_sector_erase = 0;
static uint8_t __attribute__ ((aligned (4))) file_buffer[FILE_BUFFER_SIZE];

typedef void *(pAppEntryPoint)(void);

volatile uint32_t watchdog_timeout = 0;
uint32_t watchdog_reload = 0;

volatile uint32_t ticks = 0;

static FATFS fs;
static FIL fwfile;

#if defined(BOOTLOADER_WAVETOOEASY)
#define WAVETOOEASY_LED1(x) {if (x) GPIOB->BSRRH = GPIO_Pin_0; else GPIOB->BSRRL = GPIO_Pin_0;}
#define WAVETOOEASY_LED2(x) {if (x) GPIOB->BSRRH = GPIO_Pin_1; else GPIOB->BSRRL = GPIO_Pin_1;}
#else
#define WAVETOOEASY_LED1(x)
#define WAVETOOEASY_LED2(x)
#endif

#define BL_DEBUG_FORCE_PROG	1
// #define BL_DEBUG

#ifdef BL_DEBUG
#include <stdio.h>
char debug_buff[256];

#define DEBUG_PRINTF(x, ...) \
	sprintf(debug_buff, x, __VA_ARGS__); \
	debug(debug_buff)

static void debug(const char* str)
{
	while (*str)
	{
		while (!(USART6->SR & USART_SR_TXE));
		USART6->DR = *(str++);
	}
}

static void init_debug(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_USART6EN;

	// PA11 AF8
	GPIOA->AFR[1] |= 0x08 << 12;
	GPIOA->MODER |= 0x02 << 22;

	USART6->BRR = 84000000 / 115200;
	USART6->CR1 = USART_CR1_TE | USART_CR1_UE;
}

static void deinit_debug(void)
{
	USART6->CR1 = 0;

	// Reset USART6 clock
	RCC->APB2RSTR |= RCC_APB2RSTR_USART6RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_USART6RST;

	GPIOA->AFR[1] &= ~(0x0F << 12);
	GPIOA->MODER &= ~(0x03 << 22);
}

#else

#define debug(x)
#define DEBUG_PRINTF(x, ...)
#define init_debug()
#define deinit_debug()

#endif /* BL_DEBUG */

void SysTick_Handler(void)
{
	ticks++;
}

void wait(uint32_t ms)
{
	uint32_t start = ticks;

	while (ticks - start < ms);
}

uint32_t get_ticks()
{
	return ticks;
}

static inline void init_uart(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	// PA9 & PA10 AF7
	GPIOA->AFR[1] |= 0x07 << 4;
	GPIOA->AFR[1] |= 0x07 << 8;
	GPIOA->MODER |= 0x02 << 18;
	GPIOA->MODER |= 0x02 << 20;

	USART1->BRR = 84000000 / 230400;
	USART1->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | USART_CR1_RXNEIE;

	NVIC_EnableIRQ(USART1_IRQn);
}

static inline void deinit_uart(void)
{
	NVIC_DisableIRQ(USART1_IRQn);
	USART1->CR1 = 0;

	// Reset USART1 clock
	RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;

	GPIOA->AFR[1] &= ~(0x0F << 4);
	GPIOA->AFR[1] &= ~(0x0F << 8);
	GPIOA->MODER &= ~(0x03 << 18);
	GPIOA->MODER &= ~(0x03 << 20);
}

static void uart_flush(void)
{
	USART1->CR1 |= USART_CR1_TXEIE;
	while (txsize && !(USART1->SR & USART_SR_TC));
}

static void uart_write(uint8_t* data, uint32_t len)
{
	while (len)
	{
		if (txsize == USART_BUFFER_MAX_LENGTH)
			uart_flush();

		tx_buffer[txwr++] = *data++;
		if (txwr == USART_BUFFER_MAX_LENGTH)
			txwr = 0;

		len--;
		txsize++;
	}
}

static uint32_t uart_read(uint8_t* data, uint32_t len)
{
	uint32_t copied = 0;
	uint32_t in_buffer = 0;

	__disable_irq();
	in_buffer = rxsize;
	__enable_irq();

	while (in_buffer && len)
	{
		*data++ = rx_buffer[rxrd++];
		if (rxrd == USART_BUFFER_MAX_LENGTH)
			rxrd = 0;

		len--;
		in_buffer--;
		copied++;
	}

	__disable_irq();
	rxsize -= copied;
	__enable_irq();

	return copied;
}

void USART1_IRQHandler(void)
{
	uint8_t c;

	if (USART1->SR & USART_SR_RXNE)
	{
		c = USART1->DR;
		if (rxsize < USART_BUFFER_MAX_LENGTH)
		{
			rx_buffer[rxwr++] = c;
			if (rxwr == USART_BUFFER_MAX_LENGTH)
				rxwr = 0;

			rxsize++;
		}
	}

	if (USART1->SR & USART_SR_TXE)
	{
		if (txsize)
		{
			USART1->DR = tx_buffer[txrd++];
			if (txrd == USART_BUFFER_MAX_LENGTH)
				txrd = 0;

			txsize--;
		} else {
			USART1->CR1 &= ~USART_CR1_TXEIE;
		}
	}

	if (USART1->SR & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE))
	{
		/* It is cleared by a software sequence (a read to the
		 * USART_SR register followed by a read to the USART_DR register).
		 */
		c = USART1->DR;
	}
}

static uint8_t getch(uint8_t* c, uint32_t timeout)
{
	uint32_t counter = ticks;
	while (!rxsize)
	{
		if (ticks - counter > timeout)
			return 0;
	}

	return uart_read(c, 1);
}

static uint8_t flash_status(void)
{
	if (FLASH->SR & FLASH_SR_BSY)
		return FLASH_STATUS_BUSY;

	if (FLASH->SR & (FLASH_SR_WRPERR | FLASH_SR_PGAERR |
					 FLASH_SR_PGPERR | FLASH_SR_PGSERR))
		return FLASH_STATUS_ERROR;

	return FLASH_STATUS_READY;
}

static inline uint8_t flash_is_busy(void)
{
	return (flash_status() == FLASH_STATUS_BUSY);
}

static inline void unlock_flash(void)
{
	FLASH->KEYR = 0x45670123;
	FLASH->KEYR = 0xCDEF89AB;
}

static inline void lock_flash(void)
{
	FLASH->CR |= FLASH_CR_LOCK;
}

static inline uint8_t get_sector_from_address(uint32_t address)
{

	if (address <= 0x08003FFF)
		return 0;
	else if (address <= 0x08007FFF)
		return 1;
	else if (address <= 0x0800BFFF)
		return 2;
	else if (address <= 0x0800FFFF)
		return 3;
	else if (address <= 0x0801FFFF)
		return 4;
	else if (address <= 0x0803FFFF)
		return 5;
	else if (address <= 0x0805FFFF)
		return 6;
	else if (address <= 0x0807FFFF)
		return 7;

	return 0;
}

static inline uint8_t get_sector_from_boundary(uint32_t address)
{
	if (address & 0x3FFF)
		return 0;

	switch (address)
	{
		case 0x08004000:
			return 1;
		case 0x08008000:
			return 2;
		case 0x0800C000:
			return 3;
		case 0x08010000:
			return 4;
		case 0x08020000:
			return 5;
	}

	return 0;
}

static uint8_t erase_sector(uint8_t sector)
{
	DEBUG_PRINTF("Erasing sector %i\r\n", sector);

	// Countermeasure
	if (sector == 0)
		return 0;

	while (flash_is_busy());

	if (flash_status() != FLASH_STATUS_READY)
	{
		DEBUG_PRINTF("Error before sector erase. FLASH->SR = 0x%08x\r\n", FLASH->SR);
		return 0;
	}

	__disable_irq();
	unlock_flash();

	FLASH->CR &= ~(FLASH_CR_PSIZE | FLASH_CR_SNB);
	FLASH->CR |= FLASH_CR_PSIZE_1; /* x32, no external VPP */
	FLASH->CR |= (sector << 3);
	FLASH->CR |= FLASH_CR_SER;
	FLASH->CR |= FLASH_CR_STRT;

	while (flash_is_busy());

	FLASH->CR &= (~FLASH_CR_SER);
	FLASH->CR &= ~FLASH_CR_SNB;

	lock_flash();
	__enable_irq();

	if (flash_status() != FLASH_STATUS_READY)
	{
		DEBUG_PRINTF("Error after sector erase. FLASH->SR = 0x%08x\r\n", FLASH->SR);
		return 0;
	}

	DEBUG_PRINTF("Sector %i erased\r\n", sector);
	return 1;
}

static uint8_t flash_verify(uint32_t address, uint8_t* data, uint16_t len)
{
	uint8_t* ptr = (uint8_t*) address;
	uint32_t i;

	for (i = 0; i < len; i++)
	{
		if (ptr[i] != data[i])
			return 0;
	}

	return 1;
}

static inline uint16_t program_flash(uint32_t address, uint8_t* data, uint16_t len)
{
	uint32_t access_type;
	uint8_t sector;
	uint8_t* ptr8 = NULL;
	uint16_t* ptr16 = NULL;
	uint32_t* ptr32 = NULL;

	if (address & 0xFF)
	{
		debug("Error! Data not aligned\r\n");
		return 0;
	}

	/* Check that the address is beyond bootloader area */
	if (address < MAIN_PROGRAM_ADDR)
	{
		debug("Error: trying to program address in bootloader sector\r\n");
		return 0;
	}

	/* Check that the address is a valid flash address */
	if (address > FLASH_END_ADDR)
	{
		debug("Error: trying to program address beyond the end of flash\r\n");
		return 0;
	}

	DEBUG_PRINTF("Programming at address 0x%08x, length %i bytes\r\n", address, len);

	while (len)
	{
		/* Flash is divided in 8 sectors. In the first sector (16kB) resides
		 * the bootloader. For the rest, we check if the provided address and the
		 * start address match and erase if necessary.
		 */
		sector = get_sector_from_boundary(address);

		if (sector)
		{
			/* If we already erased the signature sector, avoid erasing it twice */
			if (sector != signature_sector_erase)
			{
				if (!erase_sector(sector))
					return 0;
			}
		}

		while (len)
		{
			if (len >= 4)
			{
				access_type = 2;
				ptr32 = (uint32_t*) address;
			} else if (len >= 2)
			{
				access_type = 1;
				ptr16 = (uint16_t*) address;
			} else {
				access_type = 0;
				ptr8 = (uint8_t*) address;
			}

			while (flash_is_busy());

			__disable_irq();
			unlock_flash();

			FLASH->CR &= ~FLASH_CR_PSIZE;
			FLASH->CR |= access_type << 8;
			FLASH->CR |= FLASH_CR_PG;

			if (access_type == 2)
			{
				*ptr32 = *((uint32_t*) data);
				len -= 4;
				address += 4;
				data += 4;
			} else if (access_type == 1)
			{
				*ptr16 = *((uint16_t*) data);
				len -= 2;
				address += 2;
				data += 2;
			} else {
				*ptr8 = *data;
				len--;
				address++;
				data++;
			}

			while (flash_is_busy());

			lock_flash();
			__enable_irq();

			/* Check for errors */
			if (FLASH->SR & 0x1F0)
			{
				DEBUG_PRINTF("Error programming. FLASH->SR = 0x%08x\r\n", FLASH->SR);
				return 0;
			}
		}
	}

	return 1;
}

static inline void jump_to_app(uint32_t addr)
{
	pAppEntryPoint *pFunc = (pAppEntryPoint*) *((uint32_t*)(addr+4));
	__set_MSP(*((uint32_t*)addr));
	pFunc();
}

#define STATE_START					0
#define STATE_GET_SEQUENCE_NUMBER	1
#define STATE_GET_MESSAGE_SIZE_1		2
#define STATE_GET_MESSAGE_SIZE_2		3
#define STATE_GET_TOKEN				4
#define STATE_GET_DATA				5
#define STATE_GET_CHECKSUM			6

static uint8_t receive_message(uint32_t timeout)
{
	uint8_t state = STATE_START;
	uint16_t msg_size = 0;
	uint8_t c = 0;
	uint8_t chksum = 0;

	while (1)
	{
		if (!getch(&c, timeout))
			return 0;

		if (state != STATE_GET_CHECKSUM)
			chksum ^= c;

		switch(state)
		{
			case STATE_START:
				if (c == MESSAGE_START)
				{
					chksum = c;
					state = STATE_GET_SEQUENCE_NUMBER;
				}
				break;

			case STATE_GET_SEQUENCE_NUMBER:
				next_seq = c;
				state = STATE_GET_MESSAGE_SIZE_1;
				break;

			case STATE_GET_MESSAGE_SIZE_1:
				msg_size = c << 8;
				state = STATE_GET_MESSAGE_SIZE_2;
				break;

			case STATE_GET_MESSAGE_SIZE_2:
				msg_size |= c;

				if (!msg_size || (msg_size > MESSAGE_MAX_LENGTH))
					state = STATE_START;
				else
					state = STATE_GET_TOKEN;
				break;

			case STATE_GET_TOKEN:
				if (c != 0x0E)
				{
					state = STATE_START;
				} else {
					message_offset = 0;
					state = STATE_GET_DATA;
				}
				break;

			case STATE_GET_DATA:
				message_buffer[message_offset++] = c;
				if (message_offset == msg_size)
					state = STATE_GET_CHECKSUM;
				break;

			case STATE_GET_CHECKSUM:
				if (chksum == c)
					return 1;

				state = STATE_START;
				chksum = 0;
		}
	}

	return 0;
}

static void send_message(uint8_t* data, uint16_t data_len)
{
	uint8_t chksum = MESSAGE_START;
	uint16_t i;
	uint8_t c = MESSAGE_START;

	uart_write(&c, 1);

	uart_write(&next_seq, 1);
	chksum ^= next_seq;
	next_seq++;

	c = data_len >> 8;
	uart_write(&c, 1);
	chksum ^= c;

	c = data_len & 0x0F;
	uart_write(&c, 1);
	chksum ^= c;

	c = 0x0E;
	uart_write(&c, 1);
	chksum ^= 0x0E;

	uart_write(data, data_len);
	for (i = 0; i < data_len; i++)
		chksum ^= *data++;

	uart_write(&chksum, 1);
	uart_flush();
}

static uint32_t find_signature(void)
{
	uint32_t* ptr = (uint32_t*) MAIN_PROGRAM_ADDR;
#ifdef BL_DEBUG
	uint32_t time = ticks;
#endif
	while (ptr < (uint32_t*) FLASH_END_ADDR)
	{
		if (*ptr == 0x4B4B4141) // AAKK
		{
			if (memcmp(ptr, SIGNATURE, 32) == 0)
			{
#ifdef BL_DEBUG
				DEBUG_PRINTF("Signature found at 0x%08x\r\n", (uint32_t) ptr);
				DEBUG_PRINTF("Time to find signature: %ims\r\n", ticks - time);
#endif
				return (uint32_t) ptr;
			}
		}

		ptr++;
	}

	debug("Signature not found\r\n");
	return 0;
}

static uint8_t find_signature_in_file(FIL* file)
{
	uint32_t size;
	UINT read;
	uint32_t* ptr = (uint32_t*) file_buffer;
	uint32_t offs = 0;

	size = f_size(file);

	if (size <= FILE_BUFFER_SIZE || size >= FIRMWARE_MAX_SIZE)
		return 0;

	// Read FILE_BUFFER_SIZE bytes backwards, the signature should be somewhere at the end
	if (f_lseek(file, size - FILE_BUFFER_SIZE) != FR_OK)
		return 0;

	if (f_read(file, file_buffer, FILE_BUFFER_SIZE, &read) != FR_OK || read != FILE_BUFFER_SIZE)
		return 0;

	while (offs < FILE_BUFFER_SIZE)
	{
		if (*ptr == 0x4B4B4141) // AAKK
		{
			if (memcmp(ptr, SIGNATURE, 32) == 0)
				return 1;
		}

		offs += 4;
		ptr++;
	}

	return 0;
}

static void invalidate_signature()
{
	uint32_t signature_sector;
	uint32_t signature_address = find_signature();
	if (signature_address)
	{
		/* Invalidate signature */
		debug("Invalidating signature\r\n");
		signature_sector = get_sector_from_address(signature_address);
		if (signature_sector)
		{
			if (erase_sector(signature_sector))
			{
				signature_sector_erase = signature_sector;
				message_buffer[1] = STATUS_CMD_OK;
			} else {
				debug("Error erasing signature address\r\n");
				message_buffer[1] = STATUS_CMD_TOUT;
				return;
			}
		}
	} else {
		signature_sector_erase = 0;
	}
}

static void receive_sketch(void)
{
	uint32_t next_addr = 0;
	uint16_t data_len;
	uint32_t base_address = FLASH_START_ADDR;
	uint8_t ext_addr = 0;
	uint32_t rd_idx = 0;
	uint8_t* ptr;

	for(;;)
	{
		if (receive_message(1000))
		{
			switch (message_buffer[0])
			{
				case CMD_SIGN_ON:
					debug("CMD_SIGN_ON\r\n");
					message_buffer[1] = STATUS_CMD_OK;
					message_buffer[2] = 8;
					memcpy(&message_buffer[3], "STK500_2", 8);
					send_message(message_buffer, 11);
					break;

				case CMD_SET_PARAMETER:
					DEBUG_PRINTF("CMD_SET_PARAMETER %i = %i\r\n", message_buffer[1], message_buffer[2]);

					if (message_buffer[1] < sizeof(parameters))
						parameters[message_buffer[1]] = message_buffer[2];

					message_buffer[1] = STATUS_CMD_OK;
					send_message(message_buffer, 2);
					break;

				case CMD_GET_PARAMETER:

					if (message_buffer[1] < sizeof(parameters))
						message_buffer[2] = parameters[message_buffer[1]];
					else
						message_buffer[2] = 0;

					DEBUG_PRINTF("CMD_GET_PARAMETER %i = %i\r\n", message_buffer[1], message_buffer[2]);

					message_buffer[1] = STATUS_CMD_OK;
					send_message(message_buffer, 3);
					break;

				case CMD_LOAD_ADDRESS:
					next_addr = (message_buffer[1] << 24) | (message_buffer[2] << 16) |
								(message_buffer[3] <<8 ) | (message_buffer[4]);

					if (next_addr & 0x80000000)
						ext_addr = 1;

					next_addr <<= 1;
					next_addr += base_address;

					DEBUG_PRINTF("CMD_LOAD_ADDRESS address = 0x%08x %s\r\n", next_addr, ext_addr ? "EXTENDED" : "");

					if (ext_addr)
						next_addr += 0x10000;

					message_buffer[1] = STATUS_CMD_OK;
					send_message(message_buffer, 2);
					break;

				case CMD_ENTER_PROGMODE:
					debug("CMD_ENTER_PROGMODE\r\n");

					invalidate_signature();

					message_buffer[1] = STATUS_CMD_OK;
					send_message(message_buffer, 2);
					break;

				case CMD_LEAVE_PROGMODE:
					debug("CMD_LEAVE_PROGMODE\r\n");
					message_buffer[1] = STATUS_CMD_OK;
					send_message(message_buffer, 2);
					wait(100);
					NVIC_SystemReset();
					return;

				case CMD_CHIP_ERASE:
					debug("CMD_CHIP_ERASE\r\n");
					message_buffer[1] = STATUS_CMD_OK;
					send_message(message_buffer, 2);
					break;

				case CMD_PROGRAM_FLASH:
					debug("CMD_PROGRAM_FLASH\r\n");

					data_len = (message_buffer[1] << 8) | message_buffer[2];

					if (program_flash(next_addr, &message_buffer[10], data_len) &&
						flash_verify(next_addr, &message_buffer[10], data_len))
					{
						message_buffer[1] = STATUS_CMD_OK;
					} else {
						message_buffer[1] = STATUS_CMD_TOUT;
					}
					send_message(message_buffer, 2);
					break;

				case CMD_READ_FLASH:
					debug("CMD_READ_FLASH\r\n");

					data_len = (message_buffer[1] << 8) | message_buffer[2];
					message_buffer[1] = STATUS_CMD_OK;
					rd_idx = 2;
					ptr = (uint8_t*) ((uint32_t) next_addr);
					while (data_len)
					{
						message_buffer[rd_idx++] = *ptr++;
						data_len--;
					}
					message_buffer[rd_idx++] = STATUS_CMD_OK;
					send_message(message_buffer, rd_idx);
					break;
			}
		} else {
			debug("Timeout\r\n");
			break;
		}
	}
}

static uint8_t copy_file(FIL* file)
{
	uint32_t address = MAIN_PROGRAM_ADDR;
	uint32_t left;
	uint32_t to_read;
	UINT read;

	if (f_lseek(file, 0) != FR_OK)
		return 0;

	// File length was checked before
	left = f_size(file);

	while (left)
	{
		WAVETOOEASY_LED2(1);

		if (left < FILE_BUFFER_SIZE)
			to_read = left;
		else
			to_read = FILE_BUFFER_SIZE;

		if (f_read(file, file_buffer, to_read, &read) != FR_OK || read != to_read)
			return 0;

		if (!program_flash(address, file_buffer, read))
			return 0;

		left -= read;
		address += read;

		WAVETOOEASY_LED2(0);
		wait(25);
	}

	return 1;
}

static uint8_t verify_copy(FIL* file)
{
	uint32_t address = MAIN_PROGRAM_ADDR;
	uint32_t left;
	uint32_t to_read;
	uint8_t* ptr = (uint8_t*) MAIN_PROGRAM_ADDR;
	UINT read;

	if (f_lseek(file, 0) != FR_OK)
		return 0;

	// File length was checked before
	left = f_size(file);

	while (left)
	{
		WAVETOOEASY_LED2(1);

		if (left < FILE_BUFFER_SIZE)
			to_read = left;
		else
			to_read = FILE_BUFFER_SIZE;

		if (f_read(file, file_buffer, to_read, &read) != FR_OK || read != to_read)
			return 0;

		if (memcmp(file_buffer, ptr, read) != 0)
			return 0;

		left -= read;
		address += read;
		ptr += read;

		WAVETOOEASY_LED2(0);
		wait(25);
	}

	return 1;
}

#if defined (BOOTLOADER_PROPBOARD)
static void set_periph_power_on()
{
	GPIO_InitTypeDef GPIO_InitStruct;

	// Configure 5V and 3.3V power enable pins
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	// Configure audio mute
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIOC->BSRRL = GPIO_Pin_1 | GPIO_Pin_2;
	GPIOA->BSRRH = GPIO_Pin_15;
}

static void set_periph_power_off()
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIOC->BSRRH = GPIO_Pin_1 | GPIO_Pin_2;

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
}
#endif

static uint8_t do_file_update()
{
	uint8_t res = 0;

#if defined (BOOTLOADER_PROPBOARD)
	const char* fw_name = "pb.bin";
	set_periph_power_on();
	wait(250);
#else
	const char* fw_name = "wte.bin";
#endif

	f_mount(&fs, "", 0);

	if (f_open(&fwfile, fw_name, FA_OPEN_EXISTING | FA_READ) == FR_OK)
	{
		// Check if we can find the signature.
		// This means the file was copied into the SD entirely
		if (find_signature_in_file(&fwfile))
		{
			// Remove any flash errors
			FLASH->SR = 0xC0;

			invalidate_signature();

			if (!copy_file(&fwfile) || !verify_copy(&fwfile))
			{
#if defined (BOOTLOADER_WAVETOOEASY)
				// On WaveTooEasy we loop forever while
				// blinking the LEDs
				while(1)
				{
					WAVETOOEASY_LED1(0);
					WAVETOOEASY_LED2(0);
					wait(250);
					WAVETOOEASY_LED1(1);
					WAVETOOEASY_LED2(1);
					wait(250);
				}
#endif
			} else {
				res = 1;
			}
		}

		// We can now close and delete the file
		f_close(&fwfile);

		if (res)
			f_unlink(fw_name);
	}

	f_unmount("");

	// Explicitly de-initialize the SD
	sdDeinitialize();

#if defined (BOOTLOADER_PROPBOARD)
	set_periph_power_off();
	wait(250);
#endif

	return res;
}

#if defined (BOOTLOADER_WAVETOOEASY)
void configure_leds()
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	WAVETOOEASY_LED1(0);
	WAVETOOEASY_LED2(0);
}
#endif

int main (void)
{
	uint32_t csr;

	init_debug();
	debug("Entered Bootloader\r\n");

	SysTick_Config((SystemCoreClock / 1000) - 1);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB |
						   RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD,
						   ENABLE);

#if defined (BOOTLOADER_WAVETOOEASY)
	configure_leds();
#endif

	/* Get the cause of reset */
	csr = RCC->CSR;

	/* Clear reset flags */
	RCC->CSR |= RCC_CSR_RMVF;

	/* Try to update from file on SD */
	if (!do_file_update())
	{
		/* Run the programmer only if reset came from external reset pin, POR or BOR */
		if (csr & (RCC_CSR_PADRSTF | RCC_CSR_PORRSTF | RCC_CSR_BORRSTF))
		{
			init_uart();

			debug("Waiting for data...\r\n");
			receive_sketch();
		} else {
			debug("Reason: other than reset pin/POR/BOR\r\n");
		}
	}

	if (find_signature())
	{
		debug("Jumping to app\r\n");

		SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
		deinit_uart();
		deinit_debug();

		jump_to_app(MAIN_PROGRAM_ADDR);
	} else {
		// Wait for some reset
		debug("Idling...\r\n");
		while (1);
	}

	return 0;
}
