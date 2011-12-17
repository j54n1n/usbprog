/* mciface.h
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
#ifndef _MCIFACE_H_
#define _MCIFACE_H_


#include "../usbn2mc/tiny/usbnapi.h"

unsigned char USBNRead(unsigned char Adr);
void USBNReadBlock(uint8_t Addr, uint8_t* Data, uint8_t Size);
void USBNWrite(unsigned char Adr,unsigned char Data);
void USBNWriteBlock(uint8_t Addr, const uint8_t* Data, uint8_t Size, uint8_t isPgmSpace);

void USBNInitMC(void);

// print debug messages
void USBNDebug(char *msg);

//void USBNInterfaceRequests(DeviceRequest *req,EPInfo* ep);

//void USBNDecodeVendorRequest(DeviceRequest *req);
//void USBNDecodeClassRequest(DeviceRequest *req);


/// The Atmega register used to send data/address to the USBN9604
#define USB_DATA_OUT		PORTC

/// The Atmega register used to receive data from the USBN9604
#define USB_DATA_IN		PINC	

/// The Atmega register that controls the i/o direction of the USB_DATA_OUT
#define USB_DATA_DDR		DDRC

/// The Atmega port used to send control signals to the USBN9604
#define USB_CTRL_PORT		PORTD

/// The Atmega register that controls the i/o direction of USB_CTRL_PORT
#define USB_CTRL_DDR		DDRD

/// The pin address of the chip select signal
#define  PF_CS    (1<<PD3)

/// The pin address of the Address enable signal
#define  PF_A0    (1<<PD6)

/// The pin address of the write strobe signal
#define  PF_WR    (1<<PD5)

/// The pin address of the read strobe signal
#define  PF_RD    (1<<PD4)

#define  PF_INT   (1<<PD2)

#endif /* _MCIFACE_H_ */
