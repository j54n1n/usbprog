/*
 * Copyright (c) 2006 - 2010 by Hartmut Birr
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA. 
 *
 */

#ifdef DEBUG

#include <avr/io.h>
#include <avr/interrupt.h>

#include "uart.h"

#ifndef F_CPU
#error Please define F_CPU
#endif /* !F_CPU */

//#define UART_BAUDRATE		9600
//#define UART_BAUDRATE		38400
//#define UART_BAUDRATE		57600
//#define UART_BAUDRATE		115200
#define UART_BAUDRATE		230400
//#define UART_BAUDRATE		460800
//#define UART_BAUDRATE     500000
//#define UART_BAUDRATE		921600

#define USE_U2X 


#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega32__) ||  defined(__AVR_ATmega16__)

#define UART_UDR		UDR
#define UART_UCSRA		UCSRA
#define UART_UCSRB		UCSRB
#define UART_UCSRC		UCSRC
#define UART_UBRRH		UBRRH
#define UART_UBRRL		UBRRL

#define UART_U2X        U2X

#define UART_PE			PE
#define UART_DOR		DOR
#define UART_FE			FE
#define UART_UDRE		UDRE
#define UART_TXC		TXC
#define UART_RXC		RXC

#define UART_TXEN		TXEN
#define UART_RXEN		RXEN
#define UART_UDRIE		UDRIE
#define UART_TXCIE		TXCIE
#define UART_RXCIE		RXCIE

#define UART_RX_vect	USART_RXC_vect
#define UART_TX_vect	USART_TXC_vect
#define UART_UDRE_vect	USART_UDRE_vect

#define UART_UCSRC_INIT	((1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0))

#elif defined(__AVR_ATmega88__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega644__)

#define UART_UDR		UDR0
#define UART_UCSRA		UCSR0A
#define UART_UCSRB		UCSR0B
#define UART_UCSRC		UCSR0C
#define UART_UBRRH		UBRR0H
#define UART_UBRRL		UBRR0L

#define UART_U2X        U2X0

#define UART_PE			UPE0
#define UART_DOR		DOR0
#define UART_FE			FE0
#define UART_UDRE		UDRE0
#define UART_TXC		TXC0
#define UART_RXC		RXC0

#define UART_TXEN		TXEN0
#define UART_RXEN		RXEN0
#define UART_UDRIE		UDRIE0
#define UART_TXCIE		TXCIE0
#define UART_RXCIE		RXCIE0

#define UART_UCSRC_INIT	((0<<UMSEL01)|(0<<UMSEL00)|(0<<UPM01)|(0<<UPM00)|(0<<USBS0)|(1<<UCSZ01)|(1<<UCSZ00)|(0<<UCPOL0))

#define UART_RX_vect	USART_RX_vect
#define UART_TX_vect	USART_TX_vect

#else
#error Please define your UART code
#endif


#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE 128
#endif

#ifndef UART_TX_BUFFER_SIZE
#define UART_TX_BUFFER_SIZE (512)
#endif

static uint8_t UartRxBuffer[UART_RX_BUFFER_SIZE];
#if UART_RX_BUFFER_SIZE >= 256
static volatile uint16_t UartRxCount;
static volatile uint16_t UartRxCurrent;
#else
static volatile uint8_t UartRxCount;
static volatile uint8_t UartRxCurrent;
#endif

static uint8_t UartTxBuffer[UART_TX_BUFFER_SIZE];
#if UART_TX_BUFFER_SIZE >= 256
static volatile uint16_t UartTxCount;
static volatile uint16_t UartTxCurrent;
#else
static volatile uint8_t UartTxCount;
static volatile uint8_t UartTxCurrent;
#endif
static volatile uint8_t UartStatus;

void initUart(void)
{
    uint8_t sreg = SREG;

	/* disable interrupts during initialisation */
	cli();

	/* disable the UART */
	UART_UCSRA = 0;
	UART_UCSRB = 0;
	UART_UCSRC = 0;


	/* set the baudrate */
#ifdef USE_U2X
	UART_UBRRH =
#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega32__) ||  defined(__AVR_ATmega16__)
				 (1<<URSEL) | 
#endif
	                          (((F_CPU + 4 * UART_BAUDRATE) / 8 / UART_BAUDRATE - 1) / 256);
	UART_UBRRL = ((F_CPU + 4 * UART_BAUDRATE) / 8 / UART_BAUDRATE - 1) % 256;
	UART_UCSRA = (1<<UART_U2X);
#else
	UART_UBRRH =
#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega32__)  ||  defined(__AVR_ATmega16__)
				 (1<<URSEL) | 
#endif
 							  (((F_CPU + 8 * UART_BAUDRATE) / 16 / UART_BAUDRATE - 1) / 256);
	UART_UBRRL = ((F_CPU + 8 * UART_BAUDRATE) / 16 / UART_BAUDRATE - 1) % 256;
#endif

	/* set paity, stop bits and data length */
	UART_UCSRC = UART_UCSRC_INIT;

	resetUartRx();
	resetUartTx();

	/* restore the state of the interrupt */
	SREG = sreg;
}

void resetUartRx(void)
{
	uint8_t sreg = SREG;

	cli();
	UART_UCSRB &= ~((1<<UART_RXCIE)|(1<<UART_RXEN));


	UartRxCount = 0;
	UartRxCurrent = 0;

	UartStatus &= ~(UART_RX_BUFFER_OVERFLOW|UART_PARITY_ERROR|UART_DATA_OVERRUN|UART_FRAME_ERROR);

	UART_UCSRB |= (1<<UART_RXCIE)|(1<<UART_RXEN);
	SREG = sreg;
}

void resetUartTx(void)
{
	uint8_t sreg = SREG;
		
	cli();

	UART_UCSRB &= ~((1<<UART_UDRIE)|(1<<UART_TXEN));
	UartTxCount = 0;
	UartTxCurrent = 0;
    UartStatus |= UART_TX_BUFFER_EMPTY;


	UART_UCSRB |= (1<<UART_TXEN);

	SREG = sreg;
}

ISR(UART_RX_vect)
{
	uint8_t ucsra;
    
	ucsra = UART_UCSRA;
	UartRxBuffer[UartRxCurrent] = UART_UDR;
	if (UartRxCurrent < UART_RX_BUFFER_SIZE - 1)
	{
		UartRxCurrent++;
	}
	else
	{
		UartRxCurrent = 0;
	}
	if (UartRxCount >= UART_RX_BUFFER_SIZE)
	{
		UartStatus |= UART_RX_BUFFER_OVERFLOW;
	}
	else
	{
		UartRxCount++;
	}
	if (ucsra & (1<<UART_DOR))
	{
		UartStatus |= UART_DATA_OVERRUN;
	}
	if (ucsra & (1<<UART_FE))
	{
		UartStatus |= UART_FRAME_ERROR;
	}
	if (ucsra & (1<<UART_PE))
	{
		UartStatus |= UART_PARITY_ERROR;
	}
}

uint8_t getUartRxData(uint8_t *buffer, uint8_t size)
{
	uint8_t sreg;
	uint8_t count = 0;


    sreg = SREG;
	cli();
	while (UartRxCount > 0 && count < size)
	{
		if (UartRxCurrent >= UartRxCount)
		{
			*buffer = UartRxBuffer[UartRxCurrent - UartRxCount];
		}
		else
		{
			*buffer = UartRxBuffer[UART_RX_BUFFER_SIZE + UartRxCurrent - UartRxCount];
		}
		UartRxCount--;

		SREG = sreg;

		count++;
		buffer++;

		sreg = SREG;
		cli();
	}
	SREG = sreg;

	return count;
}

ISR(UART_UDRE_vect)
{
    if (UART_UCSRA & (1<<UART_UDRE))	
    {
		if (UartTxCount)
		{
			UART_UDR = UartTxBuffer[UartTxCurrent];
			if (UartTxCurrent < UART_TX_BUFFER_SIZE - 1)
			{
				UartTxCurrent++;
			}
			else
			{
				UartTxCurrent=0;
			}
			UartTxCount--;
		}
        else
		{
			UART_UCSRB &= ~(1<<UART_UDRIE);
            UartStatus |= UART_TX_BUFFER_EMPTY;
		}
	}
}

void flashUartTxData    (void)
{
	uint8_t sreg;

    sreg = SREG;
    while (UartTxCount)
    {
        if (!(sreg & (1<<SREG_I)) &&
            UART_UCSRA & (1<<UART_UDRE) && 
            UartTxCount)
        {
		    UART_UDR = UartTxBuffer[UartTxCurrent];
		    if (UartTxCurrent < sizeof(UartTxBuffer) - 1)
		    {
			    UartTxCurrent++;
		    }
		    else
		    {
			    UartTxCurrent=0;
		    }
		    UartTxCount--;
        }
        else
        {
            SREG = sreg;
            asm("nop");
            asm("nop");
            sreg = SREG;
            cli();
        }
    }
    SREG = sreg;
}



uint8_t setUartTxData(uint8_t *buffer, uint8_t size, uint8_t wait)
{
	uint8_t current = size;
	uint8_t sreg;
    uint8_t flag = 1;

	while (current && (flag || wait))
	{
		sreg = SREG;
		cli();

        if (!(sreg & (1<<SREG_I)) &&
            UART_UCSRA & (1<<UART_UDRE) && 
            UartTxCount)
        {
		    UART_UDR = UartTxBuffer[UartTxCurrent];
		    if (UartTxCurrent < sizeof(UartTxBuffer) - 1)
		    {
			    UartTxCurrent++;
		    }
		    else
		    {
			    UartTxCurrent=0;
		    }
		    UartTxCount--;
        }

        if (UartTxCount < sizeof(UartTxBuffer))
        {
			if (UartTxCurrent + UartTxCount >= sizeof(UartTxBuffer))
			{
				UartTxBuffer[UartTxCurrent + UartTxCount - sizeof(UartTxBuffer)] = *buffer;
			}
			else
			{
				UartTxBuffer[UartTxCurrent + UartTxCount] = *buffer;
			}
		    UartTxCount++;
            UartStatus &= ~UART_TX_BUFFER_EMPTY;
			UART_UCSRB |= (1<<UART_UDRIE);

            flag = 1;
        }
        else
        {
            flag = 0;
        }
    
        SREG = sreg;

        if (flag)
        {
		    buffer++;
		    current--;
        }
	}
	return size - current;
}

uint8_t getUartStatus(void)
{
	return UartStatus;
}

uint16_t getUartRxCount(void)
{
#if UART_RX_BUFFER_SIZE >= 256
    uint16_t count;
    uint8_t sreg = SREG;

    cli();
    count = UartRxCount;
    SREG = sreg;
    return count;
#else
	return UartRxCount;
#endif
}

uint16_t getUartTxCount(void)
{
#if UART_TX_BUFFER_SIZE >= 256
    uint16_t count;
    uint8_t sreg = SREG;

    cli();
    count = UartTxCount;
    SREG = sreg;
    return count;
#else
	return UartTxCount;
#endif
}

#endif
