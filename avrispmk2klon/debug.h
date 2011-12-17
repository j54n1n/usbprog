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

#ifndef _DEBUG_H_
#define _DEBUG_H_

#include <inttypes.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif


#define DbgPrint(...) 	fprintf_P(stdout, __VA_ARGS__)

void initDebug(void);

#ifdef DEBUG

uint32_t getTicker(void);

extern prog_char DbgMsg1[];     // "(%s:%d) ";
extern prog_char DbgMsg2[];     // "(%s:%d)\n";

#ifndef NDEBUG
#define DPRINT(...)     DPRINT1(__VA_ARGS__)
#define CHECKPOINT      CHECKPOINT1
#else
#define DPRINT(...)
#define CHECKPOINT
#endif

#define DPRINT1(...)    do { uint32_t ticker = getTicker(); uint8_t sreg = SREG; cli(); DbgPrint(DbgMsg1, (uint32_t)(ticker / 1000), (uint16_t)(ticker % 1000), __FILE__, __LINE__); DbgPrint(__VA_ARGS__); SREG = sreg; } while(0)
#define CHECKPOINT1     do { uint32_t ticker = getTicker(); uint8_t sreg = SREG; cli(); DbgPrint(DbgMsg2, (uint32_t)(ticker / 1000), (uint16_t)(ticker % 1000), __FILE__, __LINE__); SREG = sreg; } while(0)

#else

#define DPRINT(...)
#define CHECKPOINT

#define DPRINT1(...)
#define CHECKPOINT1
#endif

#ifdef __cplusplus
}
#endif

#endif
