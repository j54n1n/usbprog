/*
 * Copyright (c) 2007 - 2010 by Hartmut Birr
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


#include <inttypes.h>
#include <avr/pgmspace.h>

#include <stdio.h>

#ifdef DEBUG

#include "uart.h"

prog_char DbgMsg1[] = "(%lu.%03u, %s:%d) ";
prog_char DbgMsg2[] = "(%lu.%03u, %s:%d)\n";

int uart_putchar(char c, FILE* stream)
{
    if (c == '\n')
    {   
        char cr = '\r';
        setUartTxData((uint8_t*)&cr, 1, 1);
    }
    setUartTxData((uint8_t*)&c, 1, 1);
	return 0;
}

static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

void initDebug(void)
{
   	// printf auf UART verbiegen
	stdout = &mystdout;
}

#else

void initDebug(void)
{
}

#endif
