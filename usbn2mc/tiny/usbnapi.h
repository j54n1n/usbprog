/* usbnapi.c
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

#ifndef _USBNAPI_H
#define _USBNAPI_H

#include <stdlib.h>

#include "usbn960x.h"
#include "usbn2mc.h"

struct usb_configuration_descriptor_tab
{
    uint8_t NumberOfConfigurations;
    struct usb_configuration_descriptor* Configurations[];
};    

struct usb_wstring_descriptor 
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    wchar_t wString[];
};

struct usb_wstring_descriptor_tab
{
    uint8_t NumberOfStrings;
    struct usb_wstring_descriptor* Strings[];
};

void USBNInit(struct usb_device_descriptor* _DeviceDescriptor, 
              struct usb_configuration_descriptor_tab* _ConfigurationDescriptorTab, 
              struct usb_wstring_descriptor_tab* _StringTab);

/// start usb system after configuration
void USBNStart(void);

/// handle usb chip interrupt
void USBNInterrupt(void);

uint16_t USBNGetDescriptor(const uint16_t Value, void** const DescriptorAddress);

void USBNAddInEndpointCallback(uint8_t epnr, void (*fkt)(void));
void USBNAddOutEndpointCallback(uint8_t epnr, void (*fkt)(void));

uint8_t USBNGetRxData(uint8_t ep, uint8_t *buffer, uint8_t size);

#endif /* __USBNAPI_H__ */

