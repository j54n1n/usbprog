/*
 * Copyright (c) by Hartmut Birr
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

#ifndef _UART_H_
#define _UART_H_


#define UART_RX_BUFFER_OVERFLOW 0x01
#define UART_PARITY_ERROR		0x02
#define UART_DATA_OVERRUN		0x04
#define UART_FRAME_ERROR		0x08
#define UART_TX_BUFFER_EMPTY	0x80

#ifdef __cplusplus
extern "C" {
#endif

extern void initUart(void);
extern uint8_t getUartRxData(uint8_t *buffer, uint8_t size);
extern uint8_t setUartTxData(uint8_t *buffer, uint8_t size, uint8_t wait);
extern uint8_t getUartStatus(void);
extern uint16_t getUartRxCount(void);
extern uint16_t getUartTxCount(void);
extern void resetUartRx(void);
extern void resetUartTx(void);

#ifdef __cplusplus
}
#endif

#endif
