/* usbn960x.c
* Copyright (C) 2006  Benedikt Sauter
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "usbnapi.h"

#define NDEBUG
#include "debug.h"

struct usb_device_descriptor* DeviceDescriptor;
struct usb_configuration_descriptor_tab* ConfigurationDescriptorTab;
struct usb_wstring_descriptor_tab* StringTab;

// setup global datastructure
void USBNInit(struct usb_device_descriptor* _DeviceDescriptor, 
              struct usb_configuration_descriptor_tab* _ConfigurationDescriptorTab, 
              struct usb_wstring_descriptor_tab* _StringTab)
{

    USBNWrite(CCONF, 0x02);           // clock to 16 MHz


    DeviceDescriptor = _DeviceDescriptor;
    ConfigurationDescriptorTab = _ConfigurationDescriptorTab;
    StringTab = _StringTab;
}

uint16_t USBNGetDescriptor(const uint16_t Value, void** const DescriptorAddress)
{
    const uint8_t Type = Value >> 8;
    uint8_t Number = Value & 0xff;
    uint16_t Size = 0;
    void *Address = NULL;

    DPRINT(PSTR("USB_GetDescriptor(Value=%04x, DescriptorAddress=%04x)\n"), Value, DescriptorAddress);

    switch(Type)
    {
        case DEVICE:
            Address = (void*)DeviceDescriptor;
			Size    = sizeof(struct usb_device_descriptor);
			break;

        case CONFIGURATION:
            if (Number < pgm_read_byte(&ConfigurationDescriptorTab->NumberOfConfigurations))
            {   
                Address = (void*)pgm_read_word(&ConfigurationDescriptorTab->Configurations[Number]);
			    Size    = pgm_read_word(&((struct usb_configuration_descriptor*)Address)->wTotalLength);
            }
			break;

        case STRING:
            if (Number < pgm_read_byte(&StringTab->NumberOfStrings))
            {
                Address = (void*)pgm_read_word(&StringTab->Strings[Number]);
                Size = pgm_read_byte(&((struct usb_wstring_descriptor*)Address)->bLength);
            }
            break;

    }

    *DescriptorAddress = Address;
    return Size;
}

void USBNStart(void)
{
  DPRINT1(PSTR("USBN960x revision: %u\n"), USBNRead(RID) & 0x0f);
  DPRINT1(PSTR("CCONF=%02x\n"), USBNRead(CCONF));

  USBNWrite(MCNTRL,SRST);           // clear all registers
  while(USBNRead(MCNTRL)&SRST);

  USBNWrite(FAR,AD_EN+0x00);        // set default address
  USBNWrite(EPC0,DEF);
  USBNWrite(TXC0,FLUSH);            // FLUSHTX0;

  USBNWrite(RXC0,RX_EN+FLUSH);      // enable EP0 receive

  USBNWrite(RXMSK, RX_FIFO0+RX_FIFO1+RX_FIFO2+RX_FIFO3);            // data incoming EP0
  USBNWrite(TXMSK, TX_FIFO0+TX_FIFO1+TX_FIFO2+TX_FIFO3);            // data incoming EP0
 
  USBNWrite(ALTMSK, ALT_RESET+ALT_SD3+ALT_EOP+ALT_RESUME);

  USBNWrite(NAKMSK, /*NAK_OUT1*//*|NAK_IN1*//*|*/NAK_OUT0);

  USBNWrite(MAMSK, INTR_E|RX_EV|ALT|TX_EV|NAK);

  USBNWrite(NFSR,OPR_ST);
  USBNWrite(MCNTRL, VGE+NAT+INT_L_P);     // VGE, no NAT, interrupt on high

}

uint8_t USBNGetRxData(uint8_t ep, uint8_t *buffer, uint8_t size)
{
    DPRINT(PSTR("USBNGetRxData(ep=%u, buffer=%04x, size=%u)\n"), ep, buffer, size);

    if (ep >= 1 && ep <= 3)
    {
        return _USBNGetRxData(ep, buffer, size);
    }
    return 0;
}

// ********************************************************************
// Interrupt Routine for USBN960x
// ********************************************************************


void USBNInterrupt(void)
{
    uint8_t maev,mask;
  
    while ((maev = USBNRead(MAEV)) & (RX_EV|TX_EV|ALT|/*WARN|*/NAK))
    {
        if(maev & NAK) _USBNNackEvent(); 
        if(maev & RX_EV) _USBNReceiveEvent();
        if(maev & TX_EV) _USBNTransmitEvent();
        if(maev & ALT) _USBNAlternateEvent();
//        if(maev & WARN) _USBNWarnEvent();
    }
  
    mask = USBNRead(MAMSK);
    USBNWrite(MAMSK,0x00);                  // disable irq
    USBNWrite(MAMSK, mask);
}

void USBNAddInEndpointCallback(uint8_t epnr, void (*fkt)(void))
{
    if (epnr >= 1 && epnr <= 3)
    {
        tx_info[epnr].func = fkt;
    }
}

void USBNAddOutEndpointCallback(uint8_t epnr, void (*fkt)(void))
{
    if (epnr >= 1 && epnr <= 3)
    {
        rx_info[epnr].func = fkt;
    }
}
