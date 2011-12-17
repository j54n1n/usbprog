/* usbn960x.c
* Copyright (C) 2005  Benedikt Sauter
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "usbn2mc.h"
//#include "uart.h"

// ********************************************************************
// This subroutine handles the communication with usbn9604          
// ********************************************************************


// Read data from usbn96x register

void USBNInitMC(void)
{
  uint8_t sreg = SREG;
  cli();

  // INT 0 fallende Flanke
  MCUCR &= ~(1 << ISC00);
  MCUCR |=  (1 << ISC01);
  GICR |= (1 << INT0);

  USB_CTRL_DDR &= ~PF_INT;
  USB_CTRL_DDR |= (PF_RD | PF_WR | PF_CS | PF_A0);

  USB_CTRL_PORT &= ~(PF_A0 | PF_INT);
  USB_CTRL_PORT |= (PF_RD | PF_WR | PF_CS);

  SREG = sreg;
}



inline unsigned char USBNBurstRead(void)
{
  uint8_t result;

  USB_CTRL_PORT &= ~PF_A0;
  USB_CTRL_PORT &= ~PF_CS;
  USB_CTRL_PORT &= ~PF_RD;
  asm("nop");              // pause for data to get to bus
  asm("nop");
  result = USB_DATA_IN;
  USB_CTRL_PORT |= PF_RD;
  USB_CTRL_PORT |= PF_CS;
  return result;
}

unsigned char USBNRead(unsigned char Adr)
{
  uint8_t sreg;
  uint8_t result;

  sreg = SREG;
  cli();

  USB_DATA_DDR = 0xff;      // set for output
  USB_DATA_OUT = Adr;       // load address

  USB_CTRL_PORT |= PF_A0;
  USB_CTRL_PORT &= ~PF_CS;
  USB_CTRL_PORT &= ~PF_WR;  // strobe the CS, WR, and A0 pins
  asm("nop");
  asm("nop");
  USB_CTRL_PORT |= PF_WR;
  USB_CTRL_PORT |= PF_CS;
  USB_DATA_DDR = 0x00;      // set PortD for input
  asm("nop");
  asm("nop");
  result = USBNBurstRead();

  SREG = sreg;

  return result;
}

void USBNReadBlock(unsigned char Addr, unsigned char* Buffer, unsigned char Size)
{
  uint8_t sreg;

  sreg = SREG;
  cli();

  USB_DATA_DDR = 0xff;      // set for output
  USB_DATA_OUT = Addr;      // load address

  USB_CTRL_PORT |= PF_A0;
  USB_CTRL_PORT &= ~PF_CS;
  USB_CTRL_PORT &= ~PF_WR;  // strobe the CS, WR, and A0 pins
  asm("nop");
  USB_CTRL_PORT |= PF_WR;
  USB_CTRL_PORT |= PF_CS;
  USB_DATA_DDR = 0x00;      // set PortD for input

  USB_CTRL_PORT &= ~PF_A0;

  while (Size--)
  {
    USB_CTRL_PORT &= ~PF_CS;
    USB_CTRL_PORT &= ~PF_RD;
    asm("nop");             // pause for data to get to bus
    *Buffer++ = USB_DATA_IN;
    USB_CTRL_PORT |= PF_RD;
    USB_CTRL_PORT |= PF_CS;
  }

  SREG = sreg;
}



// Write data to usbn96x register
void USBNWrite(unsigned char Adr, unsigned char Data)
{
  uint8_t sreg;

  sreg = SREG;
  cli();

  USB_DATA_OUT = Adr;        // put the address on the bus
  USB_DATA_DDR = 0xff;         // set for output

  USB_CTRL_PORT |= PF_A0;
  USB_CTRL_PORT &= ~PF_CS;
  USB_CTRL_PORT &= ~PF_WR;  // strobe the CS, WR, and A0 pins
  asm("nop");
  asm("nop");
  USB_CTRL_PORT |= PF_WR;
  USB_CTRL_PORT |= PF_CS;
  asm("nop");
  asm("nop");
  USBNBurstWrite(Data);
  asm("nop");
  asm("nop");
  SREG = sreg;
}

void USBNWriteBlock(unsigned char Addr, unsigned char* Buffer, unsigned char Size, unsigned char isPgmSpace)
{
  uint8_t sreg;

  sreg = SREG;
  cli();

  USB_DATA_DDR = 0xff;          // set for output
  USB_DATA_OUT = Addr;          // put the address on the bus

  USB_CTRL_PORT |= PF_A0;
  USB_CTRL_PORT &= ~PF_CS;
  USB_CTRL_PORT &= ~PF_WR;      // strobe the CS, WR, and A0 pins
  asm("nop");
  USB_CTRL_PORT |= PF_WR;
  USB_CTRL_PORT |= PF_CS;

  USB_CTRL_PORT &= ~PF_A0;

  if (isPgmSpace)
  {
    while(Size--)
    {
      USB_DATA_OUT = pgm_read_byte(Buffer);   // put data on the bus

      USB_CTRL_PORT &= ~PF_CS;
      USB_CTRL_PORT &= ~PF_WR;  // strobe the CS and WR

      Buffer++;                 // increment buffer ptr (this is also a delay)

      USB_CTRL_PORT |= PF_WR;
      USB_CTRL_PORT |= PF_CS;
    }
  }
  else
  {
    while(Size--)
    {
      USB_DATA_OUT = *Buffer++;   // put data on the bus

      USB_CTRL_PORT &= ~PF_CS;
      USB_CTRL_PORT &= ~PF_WR;    // strobe the CS and WR

      asm("nop");

      USB_CTRL_PORT |= PF_WR;
      USB_CTRL_PORT |= PF_CS;
    }
  }

  SREG = sreg;
}

void USBNBurstWrite(unsigned char Data)
{
  USB_DATA_OUT = Data;       // put data on the bus
  USB_CTRL_PORT &= ~PF_A0;
  USB_CTRL_PORT &= ~PF_CS;
  USB_CTRL_PORT &= ~PF_WR;  // strobe the CS, WR, and A0 pins
  asm("nop");
  asm("nop");
  USB_CTRL_PORT |= PF_WR;
  USB_CTRL_PORT |= PF_CS;
}


