/*
 * usbprog-jtag - OpenOCD-Firmware for USBProg
 * Copyright (C) 2010 Cahya Wirawan 
 *
 * Released under the MIT Licence.
 */

#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <inttypes.h>

#include "usbprog-jtag.h"
#include "jtag_defs.h"
#include "jtag_functions.h"
#include "wait.h"
#include "uart.h"
#include "usbn2mc.h"

int datatogl = 0;

uint8_t  usbBuffer[USBPROG_USB_BUFFER_SIZE+USBPROG_USB_BUFFER_OFFSET];
uint8_t  *dataFromHost = usbBuffer+USBPROG_USB_BUFFER_OFFSET;
uint8_t  *dataToHost = usbBuffer;
uint16_t dataFromHostSize=0;
uint16_t dataToHostSize=0;

volatile uint8_t resetJtagTransfers=0;

SIGNAL(USART_RXC_vect)
{
 //Terminal(UARTGetChar());
 //UARTWrite("usbn>");
}


SIGNAL(INT0_vect)
{
  USBNInterrupt();
}


void USBNDecodeVendorRequest(DeviceRequest *req)
{
}

void USBToglAndSend(void)
{
  if(datatogl == 1) {
    USBNWrite(TXC1, TX_LAST+TX_EN+TX_TOGL);
    datatogl = 0;
  } else {
    USBNWrite(TXC1, TX_LAST+TX_EN);
    datatogl = 1;
  }
}

void USBMessageSend(uint8_t *message, uint16_t messageLength) {
  int i, pos, is64;
  unsigned char res;

  is64 = (messageLength%FRAME_SIZE);
  pos = 0;
  USBNWrite(TXC1, FLUSH);
  while (messageLength)  {
		do {
			res = USBNRead(TXC1);
		} while (res & TX_EN);
		USBNWrite(TXD1, message[pos++]);
		for (i = 1; (i < FRAME_SIZE) && (i < messageLength); i++) {
			USBNBurstWrite(message[pos++]);
		}
		USBToglAndSend();
		messageLength -= i;
  }
  if(!is64) {
    do {
			res = USBNRead(TXC1);
		} while (res & TX_EN);
    USBToglAndSend();
  }
}

void MainTask(uint8_t  *usb_out)
{
  uint16_t i, bufferLength;
  static uint16_t bufferIndex=0;

  if(dataFromHostSize == 0) {
  	dataFromHostSize = *(uint16_t*)&usb_out[0];
  	bufferLength = ((dataFromHostSize+sizeof(uint16_t))<=FRAME_SIZE)? dataFromHostSize+sizeof(uint16_t):FRAME_SIZE; 
  	for(i = sizeof(uint16_t); i < bufferLength; i++) {
  	  dataFromHost[bufferIndex++] = usb_out[i];
    }
  }
  else {
  	bufferLength = ((dataFromHostSize-bufferIndex)<=FRAME_SIZE)? dataFromHostSize-bufferIndex:FRAME_SIZE; 
  	for(i = 0; i < bufferLength; i++) {
  	  dataFromHost[bufferIndex++] = usb_out[i];
    }
  }  
  if(bufferIndex==dataFromHostSize) {
    ProcessData();
    dataFromHostSize=0;
    bufferIndex=0;
  }
}

void ProcessData(void)
{
  if(dataFromHostSize>0) {        
    //first byte is always the command
    dataFromHostSize--;
    
    dataToHostSize=0;
    
    switch(dataFromHost[0] & JTAG_CMD_MASK) 
    {      
    case JTAG_CMD_TAP_OUTPUT:
      
      dataFromHostSize*=4;

      if( dataFromHost[0] & JTAG_DATA_MASK )
        dataFromHostSize-= (4- ((dataFromHost[0] & JTAG_DATA_MASK)>>4));
      if(jtag_delay)
        dataToHostSize= jtag_tap_output_with_delay( &dataFromHost[1] , dataFromHostSize, dataToHost);
      else
        dataToHostSize= jtag_tap_output_max_speed( &dataFromHost[1] , dataFromHostSize, dataToHost);
      break;
      
    case JTAG_CMD_TAP_OUTPUT_EMU:
      dataFromHostSize*=4;
      if(dataFromHost[0]&JTAG_DATA_MASK)
        dataFromHostSize-=(4- ((dataFromHost[0]&JTAG_DATA_MASK)>>4));
      
      dataToHostSize=jtag_tap_output_emu(&dataFromHost[1], dataFromHostSize, dataToHost);
      
      break;
      
    case JTAG_CMD_READ_INPUT:
      dataToHost[0]=jtag_read_input();
      dataToHostSize=1;
      break;
    
    case JTAG_CMD_SET_SRST:
      jtag_set_srst(dataFromHost[1]&1);
      dataToHost[0]=0;//TODO: what to output here?
      dataToHostSize=1;
      break;
    
    case JTAG_CMD_SET_TRST:
      jtag_set_trst(dataFromHost[1]&1);
      dataToHost[0]=0;//TODO: what to output here?
      dataToHostSize=1;
      break;
    
    case JTAG_CMD_SET_DELAY:
      jtag_delay=dataFromHost[1]*256;
      dataToHost[0]=0;//TODO: what to output here?
      dataToHostSize=1;

    case JTAG_CMD_SET_SRST_TRST:
      jtag_set_trst_srst(dataFromHost[1]&2?1:0,dataFromHost[1]&1);
      dataToHost[0]=0;//TODO: what to output here?
      dataToHostSize=1;
    
    default: //REPORT ERROR?
      break;
    }
  }

  if (dataToHostSize)
  {
    if(dataToHostSize)
      USBMessageSend(dataToHost, dataToHostSize);
    
    dataToHostSize=0;
  }
}

int main(void)
{
  int conf, interf;
	uint16_t i = 0;
  struct list_entry *tmp;

  UARTInit();

  USBNInit();   
  
  DDRA = (1 << DDA4);
  //PORTA |= (1<<PA4);	//on
  PORTA &= ~(1<<PA4); //off
	DDRB = 0;
	PORTB = 0;

  jtag_init();

	// initialize the send and receive buffers
	for (i = 0; i < USBPROG_OUT_BUFFER_SIZE; i++) {
		dataFromHost[i] = 0;
	}

	for (i = 0; i < USBPROG_IN_BUFFER_SIZE; i++) {
		dataToHost[i] = 0;
	}
  dataFromHostSize=0;
  dataToHostSize=0;
  resetJtagTransfers=0;


  USBNDeviceVendorID(VID);
  USBNDeviceProductID(PID);
  
  USBNDeviceBCDDevice(0x0001);


  char lang[]={0x09,0x04};
  _USBNAddStringDescriptor(lang); // language descriptor

  
  USBNDeviceManufacture ("Cahya Wirawan");
  USBNDeviceProduct	("usbprog-jtag");
  //USBNDeviceSerialNumber("1");

  conf = USBNAddConfiguration();

  USBNConfigurationPower(conf,50);

  interf = USBNAddInterface(conf,0);
  USBNAlternateSetting(conf,interf,0);

  USBNAddInEndpoint(conf,interf,1,0x02,BULK,64,0,NULL);
  USBNAddOutEndpoint(conf,interf,1,0x02,BULK,64,0,&MainTask);
  
  USBNInitMC();
  // start usb chip
  USBNStart();
  sei();
  wait_ms(1000);
  while(1){
		PORTA |= (1<<PA4);  //on
		wait_ms(100);
		PORTA &= ~(1<<PA4); //off
		wait_ms(50);
		PORTA |= (1<<PA4);  //on
		wait_ms(100);
		PORTA &= ~(1<<PA4); //off
		wait_ms(5000);
#if 1
		UARTWrite("\n\rAAAA:\n\r");  
		for (i=0;i<18;i++)
		{
			SendHex((*((char*)&DeviceDescriptor + i)));
		}
		UARTWrite("\n\rBBBB\n\r");
		
		tmp = DescriptorList;
		while(tmp != NULL) {
		  for (i=0;i<tmp->len;i++)
		  {
		    SendHex((*((char*)tmp->data + i)));
		  }
		  tmp=tmp->next;
		  UARTWrite("\n\r");
	        }
		UARTWrite("\n\rCCCC\n\r");
#endif
	}
}


