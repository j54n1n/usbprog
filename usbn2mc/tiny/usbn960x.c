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

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "usbn960x.h"
#include "usbn2mc.h"

//#define NDEBUG
#include "debug.h"

struct _tx_ctrl tx_ctrl[4] = 
{
    {.tx_fifo = TX_FIFO0, .txc = TXC0, .txs = TXS0, .txd = TXD0},
    {.tx_fifo = TX_FIFO1, .txc = TXC1, .txs = TXS1, .txd = TXD1},
    {.tx_fifo = TX_FIFO2, .txc = TXC2, .txs = TXS2, .txd = TXD2},
    {.tx_fifo = TX_FIFO3, .txc = TXC3, .txs = TXS3, .txd = TXD3},
};

struct _tx_info tx_info[4] =
{
    {.FifoSize = 8, .func = _USBNTransmitFIFO0},
    {.FifoSize = 64},
    {.FifoSize = 64},
    {.FifoSize = 64},
};

struct _rx_ctrl rx_ctrl[] = 
{
    {.rx_fifo = RX_FIFO0, .rxs = RXS0, .rxd = RXD0, .rxc = RXC0},
    {.rx_fifo = RX_FIFO1, .rxs = RXS1, .rxd = RXD1, .rxc = RXC1},
    {.rx_fifo = RX_FIFO2, .rxs = RXS2, .rxd = RXD2, .rxc = RXC2},
    {.rx_fifo = RX_FIFO3, .rxs = RXS3, .rxd = RXD3, .rxc = RXC3},
};

struct _rx_info rx_info[] =
{
    {.FifoSize = 8, .func = _USBNReceiveFIFO0},
    {.FifoSize = 64,},
    {.FifoSize = 64,},
    {.FifoSize = 64,},
};

FunctionInfo  USBNFunctionInfo;

// ********************************************************************
// Interrupt Event Handler
// ********************************************************************


void _USBNNackEvent(void)
{
    uint8_t event;
  
    event = USBNRead(NAKEV);

//    DPRINT(PSTR("_USBNNackEvent(), nakev=%02x, nakmsk=%02x, epc0=%02x, txev=%02x, rxev=%02x\n"), 
//           event, USBNRead(NAKMSK), USBNRead(EPC0), USBNRead(TXEV), USBNRead(RXEV));


    if ((event & NAK_OUT0) && tx_info[0].BufferSize >= tx_info[0].FifoSize)
    {
    	DPRINT(PSTR("  NAK_OUT0\n"));
        DPRINT(PSTR("  re-enable the receiver\n"));
        USBNWrite(TXC0,FLUSH);
        USBNWrite(RXC0,RX_EN);	//re-enable the receiver  
        tx_info[0].BufferSize = 0;
    }

    if ((event & NAK_OUT1) && (USBNRead(RXC1) & RX_EN))
    {
        DPRINT(PSTR("  NAK_OUT1\n"));
    }
    if ((event & NAK_IN1) && (USBNRead(TXC1) & TX_EN))
    {
        DPRINT(PSTR("  NAK_IN1\n"));
    }
        
    if(event & NAK_IN0)
    {
        DPRINT(PSTR("  NAK_IN0\n"));
    }

    if(event & NAK_IN1)
    {
        DPRINT(PSTR("  NAK_IN1\n"));
    }
}

volatile uint8_t usbnRxEvent;
volatile uint8_t usbnTxEvent;
volatile uint8_t usbnTxStatus[4];

#ifdef BOOTLDR
#else
uint8_t _USBNGetRxStatus(uint8_t ep)
{
    uint8_t status = 0;

    uint8_t sreg = SREG;
    cli();

    status = usbnRxEvent & rx_ctrl[ep].rx_fifo ? 1 : 0;

    SREG = sreg;

    return status;
}

uint8_t USBNGetRxStatus(uint8_t ep)
{
    if (ep >= 1 && ep <= 3)
    {
        return _USBNGetRxStatus(ep);
    }

    return 0;
}
#endif

uint8_t _USBNGetRxData(uint8_t ep, uint8_t *buffer, uint8_t size)
{ 
    uint8_t sreg;
    uint8_t result = 0;
    struct _rx_info* info = &rx_info[ep];
    struct _rx_ctrl* ctrl = &rx_ctrl[ep];

    sreg = SREG;
    cli();

    if (usbnRxEvent & ctrl->rx_fifo)
    {

        info->Buffer = buffer;
        info->BufferSize = size;
        info->BufferIndex = 0;

        _USBNReceive(ep);

        result = info->BufferIndex;

        usbnRxEvent &= ~ctrl->rx_fifo;

        // flush the FIFO
        USBNWrite(ctrl->rxc, FLUSH);
 
        // enable the FIFO
        USBNWrite(ctrl->rxc, IGN_SETUP|RX_EN);
    }

    SREG = sreg;

    return result;
}

#ifdef BOOTLDR
#else
uint8_t Endpoint_IsINReady(uint8_t ep)
{
    uint8_t result = 0;

    if (ep >= 1 && ep <= 3)
    {
        uint8_t sreg = SREG;
        cli();

        result = usbnTxEvent & tx_ctrl[ep].tx_fifo ? 1 : 0;

        SREG = sreg;
    }

    return result;
}

void Endpoint_Write_Byte(uint8_t ep, uint8_t data)
{
    if (ep >= 1 && ep <= 3)
    {
        uint8_t sreg = SREG;
        cli();

        USBNWrite(tx_ctrl[ep].txd, data);

        SREG = sreg;
    }
}

void Endpoint_ClearIN(uint8_t ep)
{
    if (ep >= 1 && ep <= 3)
    {
        uint8_t sreg = SREG;
        cli();

        USBNWrite(tx_ctrl[ep].txc, ((USBNRead(tx_ctrl[ep].txc)&TX_TOGL)^TX_TOGL)|TX_LAST|TX_EN);
        usbnTxEvent &= ~tx_ctrl[ep].tx_fifo;

        SREG = sreg;
    }
}

void Endpoint_ResetDataToggle(uint8_t ep)
{
    if (ep >= 1 && ep <= 3)
    {
        uint8_t sreg = SREG;
        cli();

        USBNWrite(tx_ctrl[ep].txc, USBNRead(tx_ctrl[ep].txc)&~TX_TOGL);

        SREG = sreg;
    }
}

uint8_t Endpoint_IsOUTReceived(uint8_t ep)
{
    uint8_t result = 0;

    if (ep >= 1 && ep <= 3)
    {
        uint8_t sreg = SREG;
        cli();

        result = usbnRxEvent & rx_ctrl[ep].rx_fifo ? 1 : 0;

        SREG = sreg;
    }

    return result;
}

uint8_t Endpoint_BytesInEndpoint(uint8_t ep)
{
    uint8_t result = 0;
    
    if (ep >= 1 && ep <= 3)
    {
        uint8_t sreg = SREG;
        cli();

        result = USBNRead(rx_ctrl[ep].rxs) & 0x0f ? 1 : 0;

        SREG = sreg;
    }

    return result;
}

uint8_t Endpoint_Read_Byte(uint8_t ep)
{
    uint8_t result = 0;

    if (ep >= 1 && ep <= 3)
    {
        uint8_t sreg = SREG;
        cli();

        result = USBNRead(rx_ctrl[ep].rxd);

        SREG = sreg;
    }

    return result;
}

void Endpoint_ClearOUT(uint8_t ep)
{
    if (ep >= 1 && ep <= 3)
    {
        uint8_t sreg = SREG;
        cli();

        usbnRxEvent &= ~rx_ctrl[ep].rx_fifo;

        // flush the FIFO
        USBNWrite(rx_ctrl[ep].rxc, FLUSH);
 
        // enable the FIFO
        USBNWrite(rx_ctrl[ep].rxc, IGN_SETUP|RX_EN);

        SREG = sreg;
    }
}
    
#endif


void _USBNReceiveEvent(void)
{
    uint8_t event;
    uint8_t status;
    uint8_t ep;

    event = USBNRead(RXEV);

//    DPRINT(PSTR("_USBNReceiveEvent() RXEV=%02x\n"), event);

    usbnRxEvent |= event & (RX_FIFO1|RX_FIFO2|RX_FIFO3);

    if(event & RX_FIFO0) 
    {
        _USBNReceiveFIFO0();
    }
  
    // dynamic function call
    for (ep = 1; ep < 4; ep++)
    {
        if(event & rx_ctrl[ep].rx_fifo)
        {
            status = USBNRead(rx_ctrl[ep].rxs);
//            DPRINT(PSTR("  RXS%u=%02x\n"), ep, status);
            if (status & RX_ERR)
            {
                DPRINT(PSTR("  RX_ERR\n"));
                usbnRxEvent &= ~rx_ctrl[ep].rx_fifo;
                USBNWrite(rx_ctrl[ep].rxc, FLUSH);
                USBNWrite(rx_ctrl[ep].rxc, RX_EN|IGN_SETUP);
            }
            else
            {
                if (rx_info[ep].func) 
                {
                    rx_info[ep].func();
                }
            }
        }
    }

}

void _USBNTransmitEvent(void)
{
    uint8_t event, ep;

    event = USBNRead(TXEV);

//    DPRINT(PSTR("_USBNTransmitEvent() TXEV=%02x\n"), event);

    usbnTxEvent |= (event & (TX_FIFO1|TX_FIFO2|TX_FIFO3));

    for (ep = 1; ep < 4; ep++)
    {
        if (event & tx_ctrl[ep].tx_fifo)
        {
            usbnTxStatus[ep] = USBNRead(tx_ctrl[ep].txs);
            DPRINT(PSTR("  TXS%u=%02x\n"), ep, usbnTxStatus[ep]);

            if (!(usbnTxStatus[ep] & ACK_STAT))
            {
//                USBNWrite(tx_ctrl[ep].txc, (USBNRead(tx_ctrl[ep].txc)&TX_TOGL)|TX_LAST|TX_EN|RFF);
                usbnTxEvent &= ~tx_ctrl[ep].tx_fifo;
            }
        }
    }

    for (ep = 0; ep < 4; ep++)
    {
        if (event & tx_ctrl[ep].tx_fifo && tx_info[ep].func)
        {
            tx_info[ep].func();
        }
    }
}

void _USBNAlternateEvent(void)
{
    uint8_t event;
    event = USBNRead(ALTEV)/* & USBNRead(ALTMSK)*/;
  
//    DPRINT(PSTR("_USBNAlternateEvent(), evwnt=%02x\n"), event);

    if(event & ALT_RESET)
    {
//        DPRINT(PSTR("  ALT_RESET\n"));
#if 1
        USBNWrite(WKUP, USBNRead(WKUP) & ~(PNDUSB|PNDUC)); 
        USBNWrite(EPC0, USBNRead(EPC0) & ~STALL);
        USBNWrite(NFSR, RST_ST);
        USBNWrite(FAR, AD_EN|0); 
        USBNWrite(EPC0,0x00);
        USBNWrite(NFSR,OPR_ST);
#else
        USBNWrite(WKUP, USBNRead(WKUP) & ~(PNDUSB|PNDUC)); 
        USBNWrite(NFSR, RST_ST);                    // NFS = NodeReset
        USBNWrite(FAR, AD_EN|0); 
        USBNWrite(EPC0,0x00);
        USBNWrite(TXC0,FLUSH);
        USBNWrite(RXC0,RX_EN);                      // allow reception
        USBNWrite(NFSR,OPR_ST);                     // NFS = NodeOperational
#endif
    }
    if(event & ALT_SD3)
    {
//        DPRINT(PSTR("  ALT_SUSPEND\n"));
#if 1
        USBNWrite(ALTMSK, ALT_SD3|ALT_RESET|ALT_RESUME);
        USBNWrite(NFSR, SUS_ST);

        USBNWrite(ALTMSK, ALT_RESUME|ALT_RESET);    // adjust interrupts
        USBNWrite(NFSR, SUS_ST);                    // enter suspend state
#endif
    }
    if(event & ALT_RESUME)
    {
//        DPRINT(PSTR("  ALT_RESUM\n"));
#if 1
        USBNWrite(ALTMSK, ALT_SD3|ALT_RESET);
        USBNWrite(NFSR,OPR_ST);
        USBNWrite(RXC0, USBNRead(RXC0)|RX_EN);
#else
        USBNWrite(ALTMSK,ALT_SD3|ALT_RESET|ALT_RESUME);
#endif
    }
    if(event & ALT_EOP)
    {
//    	DPRINT(PSTR("  ALT_EOP\n"));
    }
}


// ********************************************************************
// Receive and Transmit functions for EPs          
// ********************************************************************
void _USBNReceiveFIFO0(void)
{
    uint8_t rxstatus;
    DeviceRequest* req;
    uint8_t usbBuffer[8];

    rxstatus = USBNRead(RXS0);

    DPRINT(PSTR("_USBNReceiveFIFO0(), rxstatus=%02x\n"), rxstatus);

    if(rxstatus & SETUP_R)
    {
        memset(usbBuffer, 0, sizeof(usbBuffer));
        rx_info[0].BufferSize = 8;
        rx_info[0].BufferIndex = 0;
        rx_info[0].Buffer = usbBuffer;

        _USBNReceive(0);

        req = (DeviceRequest*)(usbBuffer);

        DPRINT(PSTR("  bmRequestType=%02x, bRequest=%02x, wValue=%04x, wIndex=%04x, wLength=%04x\n"),
               req->bmRequestType, req->bRequest, req->wValue, req->wIndex, req->wLength);
   
        USBNWrite(RXC0,FLUSH);		        // make sure the RX is off 
        USBNWrite(TXC0,FLUSH);		       	// make sure the TX is off 
        USBNWrite(EPC0,USBNRead(EPC0) & ~STALL);// turn of stall

        switch (req->bmRequestType & 0x60)      // decode request type     
        {
            case DO_STANDARD:			// standard request 
                switch (req->bRequest)	        // decode request code     
                {
#ifdef BOOTLDR
#else
                    case CLR_FEATURE:
                        DPRINT(PSTR("  CLR_FEATURE\n"));
                        _USBNClearFeature(req);
                        break;

                    case SET_FEATURE:
                        DPRINT(PSTR("  SET_FEATURE\n"));
                        _USBNSetFeature(req);
                        break;
#endif
                    case GET_CONFIGURATION:
                        DPRINT(PSTR("  GET_CONFIGURATION\n"));
                        USBNWrite(TXD0, USBNFunctionInfo.ConfigurationIndex);
                        break;

                    case GET_DESCRIPTOR:
                        DPRINT(PSTR("  GET_DESCRIPTOR\n"));
                        _USBNGetDescriptor(req);
                        break;

                    case GET_INTERFACE:
                        DPRINT(PSTR("  GET_INTERFACE\n"));
                        USBNWrite(TXD0, 0);
	                    USBNWrite(TXC0,TX_TOGL+TX_EN);  //enable the TX (DATA1)
                        break;

                    case GET_STATUS:
                        DPRINT(PSTR("  GET_STATUS\n"));
                        _USBNGetStatus(req);
                        break;

                    case SET_ADDRESS:
                        DPRINT(PSTR("  SET_ADDRESS\n"));
                        _USBNSetAddress(req);
                        break;

                    case SET_CONFIGURATION:
                        DPRINT(PSTR("  SET_CONFIGURATION\n"));
                        _USBNSetConfiguration(req); 
                        break;

                    case SET_INTERFACE:
                        DPRINT(PSTR("  SET_INTERFACE\n"));
                        if (req->wValue & 0xFF)
                        {
                            USBNWrite(EPC0,USBNRead(EPC0)|STALL);
                        }
                        else
                        {                    
	                        USBNWrite(TXC0,TX_TOGL+TX_EN);  //enable the TX (DATA1)
                        }
                        break;

                    default:
                        DPRINT(PSTR("  unknown standard request\n"));
                        USBNWrite(EPC0,USBNRead(EPC0)|STALL);
                        break;
                }      
                break;

            case DO_CLASS:     
                DPRINT(PSTR("  DO_CLASS\n"));
                USBNDecodeClassRequest(req);
//		        _USBNTransmit(0);
                break;

            case DO_VENDOR:
                DPRINT(PSTR("  DO_VENDOR\n"));
            	USBNDecodeVendorRequest(req);
		        _USBNTransmit(0);
                break;

            default:
                DPRINT(PSTR("  unknown (%02x)\n"), req->bmRequestType & 0x60);
                USBNWrite(EPC0,USBNRead(EPC0)|STALL);
                break;
        }

        //the following is done for all setup packets.  Note that if
        //no data was stuffed into the FIFO, the result of the fol-
        //lowing will be a zero-length response.                   
      
	// only for stage 2 transfers
	if(req->bmRequestType == 0x00)
	  USBNWrite(TXC0,TX_TOGL+TX_EN);  //enable the TX (DATA1)
    }
    else                              // if not a setuppacket
    {
        CHECKPOINT;
        if (tx_info[0].BufferSize >= tx_info[0].FifoSize)   // multi-pkt status stage? 
        {
            CHECKPOINT;
            if ((rxstatus & 0x5F) != 0x10)   // length error??          
            {
            }
            tx_info[0].BufferSize = 0;           // exit multi-packet mode  
            USBNWrite(TXC0,FLUSH);  // flush TX0 and disable   
        }
        USBNWrite(RXC0,RX_EN);      // re-enable the receiver  
    }  
}

void _USBNTransmitFIFO0(void)
{
  uint8_t txstat;

  txstat = USBNRead(TXS0);                        // get transmitter status

  DPRINT(PSTR("_USBNTransmitFIFO0(), txstatus=%02x\n"), txstat);

  if(txstat & TX_DONE)                            // if transmit completed
  {
    DPRINT(PSTR("  TX_DONE\n"));
    USBNWrite(TXC0, FLUSH);                       // flush TX0 and disable

    if(txstat & ACK_STAT)                         // ACK received
    {
      DPRINT(PSTR("  ACK_STAT\n"));

      if(tx_info[0].BufferSize && tx_info[0].BufferIndex <= tx_info[0].BufferSize)
      {
        _USBNTransmit(0);
      }
      else
      {
        USBNWrite(rx_ctrl[0].rxc, RX_EN);               // re-enable the receiver
      }
    }
    else                                              
    // this probably means we issued a stall handshake
    {
      DPRINT(PSTR("  re-enable receiver after a stall handshake\n"));
      USBNWrite(rx_ctrl[0].rxc, RX_EN);               // re-enable the receiver
    }
  }
  // otherwise something must have gone wrong with the previous
  // transmission, or we got here somehow we shouldn't have
  else
  { 
    DPRINT(PSTR("  error\n"));
  }
  // we do this stuff for all tx_0 events
}



// ********************************************************************
// Communicate with the FIFOs         
// ********************************************************************


void _USBNReceive(uint8_t ep)
{
    uint8_t size = 0;
    uint8_t curr;
    struct _rx_info* info = &rx_info[ep];
    uint8_t *ptr = info->Buffer + info->BufferIndex;


    DPRINT(PSTR("_USBNReceive(epnr=%u)\n"), ep);

    while ((curr = USBNRead(rx_ctrl[ep].rxs) & 0x0f) > 0 && 
           size < info->FifoSize &&
           info->BufferIndex < info->BufferSize)
    {
        if (size + curr > info->FifoSize)
            curr = info->FifoSize - size;
        if (info->BufferIndex + curr > info->BufferSize)
            curr = info->BufferSize - info->BufferIndex;

        USBNReadBlock(rx_ctrl[ep].rxd, ptr, curr);
        size += curr; 
        info->BufferIndex += curr;
        ptr += curr;
    }
    DPRINT(PSTR("  size=%u\n"), size);
}


void _USBNTransmit(uint8_t epnr)
{
  uint8_t size, rx_enable = 0;
  struct _tx_info* info = &tx_info[epnr];

  DPRINT(PSTR("_USBNTransmit(epnr=%u)\n"), epnr);

  if(info->BufferSize > 0)
  {
    DPRINT(PSTR("  Size=%u, Index=%u\n"), info->BufferSize, info->BufferIndex);
    if(info->BufferIndex < info->BufferSize)
    {
      
      USBNWrite(tx_ctrl[epnr].txc,FLUSH);       //send data to the FIFO

      if (info->BufferSize - info->BufferIndex > info->FifoSize)
      {
        size = info->FifoSize;
      }
      else
      {
        size = info->BufferSize - info->BufferIndex;
      }
      USBNWriteBlock(tx_ctrl[epnr].txd, info->Buffer + info->BufferIndex, size, info->isPgmSpace);
#if defined(DEBUG) && !defined(NDEBUG)
      for (uint8_t i = 0; i < size; i++)
      {
        uint8_t data = info->isPgmSpace ? pgm_read_byte(info->Buffer + info->BufferIndex + i) 
	                                : info->Buffer[info->BufferIndex + i];
	    if (i == 0)
	    {
	      DPRINT(PSTR("  %02x"), data);
        }
	    else
	    {
	      DbgPrint(PSTR(" %02x"), data);
        }
      }
      DbgPrint(PSTR("\n"));
#endif
      info->BufferIndex += size;

      // if end of multipaket
      if(info->BufferIndex >= info->BufferSize && !info->zeroLengthPkt)
      {
	    rx_enable = 1;
      }
    }
    else
    {
      // zero length packet
      DPRINT(PSTR("  zero length packet\n"));
      rx_enable = 1;
    }

    // toggle mechanism
    DPRINT(PSTR("  %sTX_TOGL\n"), info->DataPid ? "\0" : "!");
    USBNWrite(tx_ctrl[epnr].txc, info->DataPid ? TX_TOGL+TX_EN : TX_EN);
    info->DataPid ^= 1;

    if (rx_enable && epnr == 0)
    {
      DPRINT(PSTR("  re-enable receiver\n"));
      // re-enable the receiver, this shall be done after setting TX_EN in TXC0
      USBNWrite(rx_ctrl[0].rxc, RX_EN);
      info->BufferSize = 0;
    }
  }
}




// ********************************************************************
// standard request from EP0          
// ********************************************************************

#ifdef BOOTLDR
#else
void _USBNClearFeature(DeviceRequest* req)
{
    switch(req->bmRequestType & 0x03)
    {
        case 0: // Device
            break;

        case 1: // interface
            break;

        case 2: // endpoint 
            switch(req->wIndex & 0x0F)
            {
                case 0:
                    break;

                case 1:
                    USBNWrite(EPC1, USBNRead(EPC1) & ~STALL);
                    break;

                case 2:
                    USBNWrite(EPC2, USBNRead(EPC2) & ~STALL);
                    break;

                case 3:
                    USBNWrite(EPC3, USBNRead(EPC3) & ~STALL);
                    break;

                case 4:
                    USBNWrite(EPC4, USBNRead(EPC4) & ~STALL);
                    break;

                case 5:
                    USBNWrite(EPC5, USBNRead(EPC5) & ~STALL);
                    break;
              
                case 6:
                    USBNWrite(EPC6, USBNRead(EPC6) & ~STALL);
                    break;
        
                default:
                    break;
            }
            break;

        default:    // undefined
            break;
    }
}

void _USBNSetFeature(DeviceRequest* req)
{
    switch(req->bmRequestType & 0x03)
    {
        case 0: // Device
            break;

        case 1: // interface
            break;

        case 2: // endpoint 
            switch(req->wIndex & 0x0F)
            {
                case 0:
                    break;

                case 1:
                    USBNWrite(EPC1, USBNRead(EPC1) | STALL);
                    break;

                case 2:
                    USBNWrite(EPC2, USBNRead(EPC2) | STALL);
                    break;

                case 3:
                    USBNWrite(EPC3, USBNRead(EPC3) | STALL);
                    break;

                case 4:
                    USBNWrite(EPC4, USBNRead(EPC4) | STALL);
                    break;

                case 5:
                    USBNWrite(EPC5, USBNRead(EPC5) | STALL);
                    break;
              
                case 6:
                    USBNWrite(EPC6, USBNRead(EPC6) | STALL);
                    break;
        
                default:
                    break;
            }
            break;

        default:    // undefined
            break;
    }
}
#endif

void _USBNSetAddress(DeviceRequest *req)
{
  // set the address
  USBNWrite(EPC0,DEF);
  USBNWrite(FAR,AD_EN+req->wValue);
  USBNFunctionInfo.Address = req->wValue;
}


void _USBNGetDescriptor(DeviceRequest *req)
{
    DPRINT(PSTR("_USBNGetDescriptor()\n"));

    tx_info[0].DataPid = 1;
    tx_info[0].BufferIndex = 0;
    tx_info[0].isPgmSpace = 1;
    tx_info[0].BufferSize = USBNGetDescriptor(req->wValue, (void**)&tx_info[0].Buffer);

    if (tx_info[0].BufferSize > req->wLength)
    {
        tx_info[0].BufferSize = req->wLength;
    }
    tx_info[0].zeroLengthPkt = !(tx_info[0].BufferSize % tx_info[0].FifoSize) && tx_info[0].BufferSize < req->wLength;

    DPRINT(PSTR("  Addr=%04x, Size=%u, zeroLengthPkt=%u\n"), 
           tx_info[0].Buffer, tx_info[0].BufferSize, tx_info[0].zeroLengthPkt);
    
    _USBNTransmit(0);
}

void _USBNGetStatus(DeviceRequest *req)
{
/*
  char tmp[]={0x01,0x00};
  EP0tx.Size=4;
  EP0tx.Buf=tmp;
  _USBNTransmit(&EP0tx);
  */
}


void _USBNSetConfiguration(DeviceRequest *req)
{
#ifdef BOOTLDR
	USBNWrite(RXC1,FLUSH);					// EP1 mit callback function rein
	USBNWrite(EPC2,EP_EN+0x02); 
	USBNWrite(RXC1,RX_EN);

#else
    uint8_t in = 1;
    uint8_t out = 1;
    struct usb_endpoint_descriptor* EndpointDesc;
    uint16_t size;
    uint8_t Length;
    const uint8_t in_epc[] = {EPC1, EPC3, EPC5};
    const uint8_t out_epc[] = {EPC2, EPC4, EPC6}; 

    DPRINT(PSTR("_USBNSetConfiguration()\n"));

    size = USBNGetDescriptor((CONFIGURATION << 8) | ((req->wValue & 0xff) - 1), (void**)&EndpointDesc);
    Length = pgm_read_byte(&EndpointDesc->bLength);

    for(uint8_t i = 0; i < sizeof(in_epc); i++)
    {
        USBNWrite(in_epc[i], 0);
    }    
    for(uint8_t i = 0; i < sizeof(out_epc); i++)
    {
        USBNWrite(out_epc[i], 0);
    }    

    DPRINT(PSTR("  size=%u, EndpointDesc=%04x(%u)\n"), size, EndpointDesc, EndpointDesc ? Length : 0);

    while (Length && size >= Length)
    {
        uint8_t DescriptorType = pgm_read_byte(&EndpointDesc->bDescriptorType);
        DPRINT(PSTR("  type=%u\n"), DescriptorType);
        if (DescriptorType == ENDPOINT)
        {
            uint8_t EndpointAddress = pgm_read_byte(&EndpointDesc->bEndpointAddress);
            uint8_t epSize = pgm_read_word(&EndpointDesc->wMaxPacketSize);
            if (EndpointAddress & 0x80)
            {
                // IN endpoint
                if (in <= 3)
                {
                    DPRINT(PSTR("  TXC%u/EPC%u: %u\n"), in, 2 * in - 1, EndpointAddress & 0x7F);
                    tx_info[in].FifoSize = epSize;
                    USBNWrite(tx_ctrl[in].txc, FLUSH);
                    USBNWrite(in_epc[in - 1], EP_EN | (EndpointAddress & 0x7F));
                    usbnTxEvent |= tx_ctrl[in].tx_fifo;
                    in++;
                }
            }
            else
            {
                // OUT endpoint
                if (out <= 3)
                {
                    DPRINT(PSTR("  RXC%u/EPC%u: %u\n"), out, 2 * out, EndpointAddress);
                    rx_info[out].FifoSize = epSize;
                    USBNWrite(rx_ctrl[out].rxc, FLUSH);
                    USBNWrite(out_epc[out - 1], EP_EN | EndpointAddress);
                    USBNWrite(rx_ctrl[out].rxc, IGN_SETUP|RX_EN);
                    out++;
                }
            }
        }
        size -= Length;
        EndpointDesc = (struct usb_endpoint_descriptor*)((void*)EndpointDesc + Length);
        Length = pgm_read_byte(&EndpointDesc->bLength);
    }
#endif
}

           
