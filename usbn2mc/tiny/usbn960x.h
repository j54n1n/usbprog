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

#ifndef _USBN960X_H
#define _USBN960X_H

#include "../usbn960xreg.h"
#include "../usb11spec.h"

struct _tx_ctrl
{
    uint8_t tx_fifo;
    uint8_t txc;
    uint8_t txs;
    uint8_t txd;
};

struct _tx_info
{
    uint8_t FifoSize;
    uint8_t* Buffer;
    uint16_t BufferIndex;
    uint16_t BufferSize;
    void (*func)(void);
    uint8_t DataPid       : 1;
    uint8_t isPgmSpace    : 1;
    uint8_t zeroLengthPkt : 1;
};

struct _rx_ctrl
{
    uint8_t rx_fifo;
    uint8_t rxs;
    uint8_t rxd;
    uint8_t rxc;
};

struct _rx_info
{
    uint8_t FifoSize;
    uint8_t* Buffer;
    uint16_t BufferIndex;
    uint16_t BufferSize;
    void (*func)(void);
};



extern struct _tx_ctrl tx_ctrl[4];
extern struct _tx_info tx_info[4];
extern struct _rx_ctrl rx_ctrl[4];
extern struct _rx_info rx_info[4];


struct list_entry
{
  void *data;   uint8_t type;
  uint8_t len;
  uint8_t conf;
  uint8_t interf;
  uint8_t index;
  struct list_entry *next;
};

struct string_entry
{   void *data;
  uint8_t index;
  struct string_entry *next;
};



/*-------------------------------------------
 * global data structs
 * ------------------------------------------*/

typedef struct functioninfo FunctionInfo;
struct functioninfo {
  unsigned char Address;
  unsigned char ConfigurationIndex;
};


typedef struct devicereq DeviceRequest;
struct devicereq {
  unsigned char	  bmRequestType;
  unsigned char	  bRequest;
  unsigned short  wValue;
  unsigned short  wIndex;
  unsigned short  wLength;
};

unsigned char EP0RXBuf[8];



// system functions

void _USBNReceiveEvent(void);
void _USBNTransmitEvent(void);
void _USBNNackEvent(void);
void _USBNAlternateEvent(void);


/// usb default requests set address
void _USBNSetAddress(DeviceRequest *req);
void _USBNGetDescriptor(DeviceRequest *req);
void _USBNSetConfiguration(DeviceRequest *req);
void _USBNGetStatus(DeviceRequest *req);
void _USBNClearFeature(DeviceRequest *req);
void _USBNSetFeature(DeviceRequest *req);

//void _USBNToggle(EPInfo* ep);
void _USBNTransmit(uint8_t ep);
void _USBNReceive(uint8_t ep);


void _USBNTransmitFIFO0(void);
void _USBNReceiveFIFO0(void);

//only for compiler
void USBNDecodeVendorRequest(DeviceRequest *req);
void USBNDecodeClassRequest(DeviceRequest *req /*,EPInfo* ep*/);

uint8_t _USBNGetRxData(uint8_t ep, uint8_t *buffer, uint8_t size);

#endif /* __USBN960X_H__ */
