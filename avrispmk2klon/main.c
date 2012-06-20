/*
* usbprog - A Downloader/Uploader for AVR device programmers
* Copyright (C) 2006 Benedikt Sauter
* Copyright (C) 2008 Hartmut Birr
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
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <string.h>

#ifndef min
#define min(x, y)   ((x) < (y) ? (x) : (y))
#endif


//#define AT89MODE

//#define F_CPU 16000000
#include <util/delay.h>

#define DEBUG_ON 0

#include "wait.h"

#include "../usbprog_base/firmwarelib/avrupdate.h"

#if DEBUG_ON
#include "uart.h"
#endif

#include "usbn2mc.h"

/* command descriptions for mk2 */
#include "avr069.h"

#include <avr/pgmspace.h>

#include <avr/eeprom.h>

#include "uart.h"

#define NDEBUG
#include "debug.h"

#ifndef EEMEM
// alle Textstellen EEMEM im Quellcode durch __attribute__ ... ersetzen
#define EEMEM  __attribute__ ((section (".eeprom")))
#endif

uint8_t ee_sck_duration EEMEM = 1;
uint16_t sck_delay_value;

/**Structures to parse incoming packets more clearly
 */

struct cmd_enter_progmode_s {
  unsigned char id;
  unsigned char timeout;
  unsigned char stabdelay;
  unsigned char cmdexedelay;
  unsigned char synchloops;
  unsigned char bytedelay;
  unsigned char pollvalue;
  unsigned char pollindex;
  unsigned char cmd1;
  unsigned char cmd2;
  unsigned char cmd3;
  unsigned char cmd4;
};

struct cmd_spi_multi_s {
  unsigned char id;
  unsigned char numTx;
  unsigned char numRx;
  unsigned char rxStart;
  unsigned char txData[256]; //This can only be used if multiple-packet commands are supported
};

#define DDR_SPI     DDRB
#define SS PB4
#define MOSI  PB5
#define MISO  PB6
#define SCK   PB7
#define RESET_PIN   PB0
#define RESET_PORT  PORTB
#define LED_PIN     PA4
#define LED_PORT    PORTA

#define LED_on     (LED_PORT   |=  (1 << LED_PIN))   // red led
#define LED_off    (LED_PORT   &= ~(1 << LED_PIN))
#define RESET_high (RESET_PORT |=  (1 << RESET_PIN))
#define RESET_low  (RESET_PORT &= ~(1 << RESET_PIN)) // reset

/*** prototypes and global vars ***/
/* send a command back to pc */
void CommandAnswer(uint8_t length);
void QueueFirstAnswerByte(uint8_t);
void QueueAnswerByte(uint8_t);
void QueueLastAnswerByte(uint8_t);

volatile struct usbprog_t {
  char lastcmd;
  int longpackage;
  int cmdpackage;
  int datatogl;
  unsigned int sck_duration;
  uint16_t fill_pos;
  uint16_t send_pos;
  int avrstudio;
  int reset_pol;
  uint8_t ready_to_transmit: 1;
  uint8_t complete:     1;
} usbprog;

#define _BUF_LEN     300
#define _TMP_OFFSET  32
volatile char answer[_BUF_LEN];

struct pgmmode_t {
  uint16_t numbytes;
  uint8_t mode;
  uint8_t delay;
  uint8_t cmd1;
  uint8_t cmd2;
  uint8_t cmd3;
  uint8_t poll1;
  uint8_t poll2;
  uint32_t address;
  uint32_t pageaddress;
  uint8_t status;
  uint16_t poll_address;
  uint8_t poll_address_valid :1;
  uint8_t poll_address_odd   :1;
  uint8_t large_flash        :1;
  uint16_t ext_address;
} pgmmode;


unsigned short sck_lookup[]PROGMEM =
{
    0, 0, 0, 0, 2, 6, 14, 18, 20, 21, 23, 24, 26, 27, 29, 30, 32, 33, 36, 38,
    39, 41, 44, 45, 48, 50, 53, 56, 59, 62, 65, 68, 71, 75, 78, 83, 87, 92, 96,
    101, 105, 111, 116, 122, 128, 135, 141, 149, 156, 164, 171, 180, 189, 198,
    209, 218, 230, 240, 252, 264, 278, 291, 306, 321, 338, 354, 371, 390, 408,
    429, 450, 473, 495, 521, 546, 573, 602, 630, 662, 695, 729, 763, 803, 842,
    884, 927, 972, 1020, 1069, 1122, 1177, 1236, 1297, 1360, 1428, 1498, 1571,
    1648, 1729, 1814, 1904, 1998, 2096, 2198, 2307, 2419, 2539, 2664, 2795,
    2930, 3074, 3229, 3387, 3550, 3729, 3911, 4104, 4299, 4512, 4737, 4973,
    5206, 5462, 5728, 6022, 6307, 6620, 6942, 7297, 7660, 8030, 8401, 8847,
    9257, 9706, 10202, 10693, 11233, 11762, 12343, 12985, 13603, 14283, 14923,
    15623, 16391, 17239, 18016, 19045, 19998, 20962, 22000, 23092, 24211, 25410,
    26664, 27970, 29323, 30767, 32308, 33896, 35521, 37311, 39136
};

#define MAX_SCK_DURATION ((sizeof(sck_lookup)/sizeof(unsigned short))-1)



SIGNAL(SIG_INTERRUPT0)
{
  USBNInterrupt();
}

extern void (*avrupdate_jump_to_boot)( void ); 

void flashUartTxData(void);

/* id need for live update of firmware */
void USBNDecodeVendorRequest(DeviceRequest *req)
{
    DPRINT(PSTR("USBNDecodeVendorRequest()\n"));
    if(req->bRequest == STARTAVRUPDATE) {
        DPRINT(PSTR("avrupdate_jump_to_boot=%04x\n"), *avrupdate_jump_to_boot);
        cli();
#ifdef DEBUG
        flashUartTxData();
        _delay_ms(1);
#endif
        avrupdate_start();
    }
}

void USBNDecodeClassRequest(DeviceRequest *req)
{
    DPRINT(PSTR("USBNDecodeClassRequest()\n"));
    USBNWrite(EPC0,USBNRead(EPC0)|STALL);
}

void spi_init(void)
{
  usbprog.sck_duration = eeprom_read_byte(&ee_sck_duration);
  if(usbprog.sck_duration > MAX_SCK_DURATION)  // prevent error when EEPROM is erased
    usbprog.sck_duration = 0x03;  //  1MHz as default

  DPRINT(PSTR("spi_init(), usbprog.sck_duration = %u\n"), usbprog.sck_duration);

  PORTB   &=  ~((1 << SCK)|(1<<MISO)|(1<<MOSI)/*|(1<<RESET_PIN)*/);  // SCK have to be low in IDLE
  DDR_SPI &=~(1 << MISO);
  DDR_SPI = (1 << MOSI)|(1 << SCK)|(1 << RESET_PIN);
  // switches back to slave mode !
  DDR_SPI |= (1<<SS); // make SS an output for SPI master.

  SPCR = 0;
  SPSR = 0;

  //This delay value is also required in hardware SPI mode in spi_pulseclockonce
  sck_delay_value=pgm_read_word(&(sck_lookup[usbprog.sck_duration]));

    switch(usbprog.sck_duration)
    {
       case 0x00:  //08MHz
        SPCR = (1<<SPE)|(1<<MSTR);
        SPSR = (1<<SPI2X);
        break;

      case 0x01:  //04MHz
        SPCR = (1<<SPE)|(1<<MSTR);
        SPSR = 0x00;

        break;

      case 0x02:  //02MHz
        SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
        SPSR = (1<<SPI2X);
        break;

      case 0x03:	//01MHz
        SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
        SPSR = 0x00;
        break;

      case 0x04:  //500kHz
        SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1);
        SPSR = (1<<SPI2X);
        break;

      case 0x05:  //250kHz
        SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1);
        SPSR = 0x00;
        break;

      case 0x06:  //125kHz
        SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<SPR1);
        SPSR = 0x00;
        break;

	  default:
        PORTB   &=  ~(1 << SCK);
        DDR_SPI &=~(1 << MISO);
        DDR_SPI = (1 << MOSI)|(1 << SCK)|(1 << RESET_PIN);
		break;
    }
}


void spi_idle(void)
{
  //DDR_SPI = 0x00;
  DDR_SPI &= ~( (1<<MOSI) | (1<<SCK) | (1<<RESET_PIN) ); // don't make SS an input !
  //PORTB = 0xFF;	// only temp for jar
  PORTB = 0x00; // holger klabunde: tri state without pullups
}

void spi_active(void)
{
  PORTB   &=  ~((1<<MISO)|(1 << MOSI)|(1 << SCK)|(1 << RESET_PIN));  // switch off unused Pullup resistors
  DDR_SPI &=~(1 << MISO);
  DDR_SPI |= (1 << MOSI)|(1 << SCK)|(1 << RESET_PIN);
}

/** Full-duplex SPI communication. For sck_duration<=6, use hardware SPI. For
 * >=6, use software spi.
 * From AT90S2313 datasheet, timing is:
 * set data, wait for 1/2 period, clock high, wait another
 * 1/2 period, read data, clock low. In other words: AVR clocks in on rising
 * edge, AVRISP clocks in on falling edge
 *
 * However, for AT89xx (AT89S51, chapter 21), there is a setup period from data
 * to sck high and also a hold period from sck low to data change. Therefore,
 * sck and mosi run 90 degree out of phase:
 * -Setup data, wait
 * -SCK high, wait
 * -Read data, wait
 * -SCK low, wait
 *
 * This guarantees compatibility with both AVR and AT89 controllers, when
 * soft SPI mode is used.
 */

unsigned char spi_inout(unsigned char data)
{
  unsigned char bitvalue,din=0;

  if(usbprog.sck_duration <= 6)
  {
    SPDR = data;
    while ( !(SPSR & (1 << SPIF)) ) ;
    return SPDR;
  }
  else    // software SPI with delay
  {
    for (bitvalue=128;bitvalue;bitvalue>>=1)
    {
      //set new data
      if(data & bitvalue)
        PORTB   |=  (1 << MOSI);
      else
        PORTB   &=  ~(1 << MOSI);

      //wait 1/4 period
      _delay_loop_2(sck_delay_value>>1);

      //set clock high
      PORTB   |=  (1 << SCK);

      //wait another 1/4 period
      _delay_loop_2(sck_delay_value>>1);

      //read in bit
      if( PINB & (1<<MISO)) {
        din|=bitvalue;
      }

      //wait another 1/4 period
      _delay_loop_2(sck_delay_value>>1);

      //clear clock
      PORTB   &=  ~(1 << SCK);

      //wait another 1/4 period
      _delay_loop_2(sck_delay_value>>1);

    }
    return din;
  }
}

/**According to the syncronisation protocol recommended by atmel, the SCLK must
 * be pulsed once every retry to try to get in sync. This can't be done in
 * hardware spi mode, so switch to software temporarily
 */

void spi_pulseclockonce(void) {
  unsigned char spcr_save,spsr_save;
  spcr_save=SPCR;
  spsr_save=SPSR;
  SPCR = 0;
  SPSR = 0;
  //pulse
  _delay_loop_2(sck_delay_value);
  PORTB   |=  (1 << SCK);
  _delay_loop_2(sck_delay_value);
  PORTB   &=  ~(1 << SCK);

  SPCR=spcr_save;
  SPSR=spsr_save;
}

uint8_t spi_cmd(uint8_t cmd, uint16_t address, uint8_t data)
{
    spi_inout(cmd);
    spi_inout(address>>8);
    spi_inout(address);
    return spi_inout(data);
}

void spi_out(char data) {
  spi_inout(data);
}
char spi_in(void) {
  return spi_inout(0);;
}

void program_fsm(uint8_t* buffer, uint8_t eeprom)
{
  uint8_t databytes = 64;
  static uint8_t *ptr;

  if (usbprog.cmdpackage) {
    // skip the header bytes
    buffer += 10;
    databytes -= 10;
    
    if (eeprom) {
      answer[0] = CMD_PROGRAM_EEPROM_ISP;
      ptr = (uint8_t*)answer +_TMP_OFFSET;  // reserve some space at beginning of the buffer to handle
                                            // answer packets during programming if necessary
      if (pgmmode.numbytes > sizeof(answer) - _TMP_OFFSET) {
        answer[1] = STATUS_CMD_FAILED;      // we are not able to program more than this
                                            // in one cycle, because we haven't enough RAM
                                            // AVR-Studio sends max. 128 Bytes in one USB-Packet
                                            // so this should never happen...
      } else {
        answer[1] = STATUS_CMD_OK;
      }
    }
  }
  #if DEBUG_ON
  UARTWrite("pgm num ");
  SendHex(pgmmode.numbytes);
  UARTWrite("\r\n ");
  #endif


  if (pgmmode.numbytes>databytes) {
    pgmmode.numbytes-=databytes;
    usbprog.longpackage=1;        //Expect more data
  } else {
    databytes=pgmmode.numbytes;   //The number of valid bytes in this packet
    pgmmode.numbytes=0;           //No more data left
    usbprog.longpackage=0;        //Expect no more data
  }

  if (eeprom && answer[1] == STATUS_CMD_OK) {
    // copy the datas to our temp buffer
    memcpy(ptr, buffer, databytes);
    ptr += databytes;
  }
  if (!eeprom || pgmmode.numbytes == 0)
  {
    uint8_t poll;
    uint8_t i;
    uint8_t loop;
    uint8_t tmp;

    if (eeprom) {
      if (answer[1] != STATUS_CMD_OK) {
        // we got a buffer overflow, 
        // we must eat up all bytes before we can return an error
        pgmmode.status = STATUS_CMD_OK;
        pgmmode.poll_address_valid = 0;
        QueueFirstAnswerByte(CMD_PROGRAM_EEPROM_ISP);
        QueueLastAnswerByte(answer[1]);
        return;
      }
      // set the pointer to the saved datas  
      buffer = (uint8_t*)answer + _TMP_OFFSET;
      databytes = ptr - buffer; 
      poll = pgmmode.poll2;
    } else {
      poll = pgmmode.poll1;
    }

    for (i = 0; i < databytes; i++, buffer++) {
      if (pgmmode.large_flash && (pgmmode.address >> 16) != pgmmode.ext_address){
        // set the extended page address
        pgmmode.ext_address = pgmmode.address >> 16;
        spi_cmd(0x4d, pgmmode.ext_address, 0x00);
      }
      if (!eeprom) {
        if (i&1){
          pgmmode.cmd1 |= 8;
        }else{
          pgmmode.cmd1 &= ~8;
        }
      }
      spi_cmd(pgmmode.cmd1, pgmmode.address, *buffer);
      if (!(pgmmode.mode & 1)) {
        // byte/word mode
        loop = 200; // 200*250usec = 50msec
        if (pgmmode.mode & 8) {
          // RDY/BSY polling
          do {
            _delay_us(250);
            tmp = spi_cmd(0xf0, 0, 0);
          }
          while ((tmp & 1) && --loop);

          if (tmp & 1) {
            pgmmode.status = STATUS_RDY_BSY_TOUT;
          }
        }else if ((pgmmode.mode & 4) && *buffer != poll) {
          // value polling
          if (!eeprom) {
            if (i&1){
              pgmmode.cmd3 |= 8;
            }else{
              pgmmode.cmd3 &= ~8;
            }
          } 
          do{
            _delay_us(250);
            tmp = spi_cmd(pgmmode.cmd3, pgmmode.address, 0);
          }while(tmp != *buffer && --loop);

          if (tmp != *buffer) {
            pgmmode.status = STATUS_CMD_TOUT;
          }
        } else{
          wait_ms(pgmmode.delay);
        }
      } else { 
        if (*buffer != poll && !pgmmode.poll_address_valid) {
          pgmmode.poll_address = pgmmode.address;
          pgmmode.poll_address_odd = i&1; 
          pgmmode.poll_address_valid = 1;
        }
      }
      if (eeprom || (i&1)) {
        pgmmode.address++;
      }
    }

    if (pgmmode.numbytes==0) {
      if ((pgmmode.mode & 0x81) == 0x81) {
        // page mode
        loop = 200; // 200*250usec = 50msec
        spi_cmd(pgmmode.cmd2, pgmmode.pageaddress, 0);

        if (pgmmode.mode & 0x40){
          // RDY/BSY polling
          do {
            _delay_us(250);
            tmp = spi_cmd(0xf0, 0, 0);
          }
          while ((tmp & 1) && --loop);

          if (tmp & 1) {
            pgmmode.status = STATUS_RDY_BSY_TOUT;
          }
        }else if ((pgmmode.mode & 0x20) && pgmmode.poll_address_valid) {
          // value polling
          if (!eeprom) {
            if (pgmmode.poll_address_odd) {
              pgmmode.cmd3 |= 8;
            }else{
              pgmmode.cmd3 &= ~8;
            }  
          }
          do{
            _delay_us(250);
            tmp = spi_cmd(pgmmode.cmd3, pgmmode.poll_address, 0);
          }while(tmp == poll && --loop);

          if (tmp == poll) {
            pgmmode.status = STATUS_CMD_TOUT;
          }
        }else {
          // timed delay
          wait_ms(pgmmode.delay);
        }
      }
      QueueFirstAnswerByte(eeprom ? CMD_PROGRAM_EEPROM_ISP : CMD_PROGRAM_FLASH_ISP);
      QueueLastAnswerByte(pgmmode.status);
      pgmmode.status = STATUS_CMD_OK;
      pgmmode.poll_address_valid = 0;
    }
  }
}

void USBToglAndSend(void)
{
  uint8_t sreg = SREG;
  cli();
  if(usbprog.datatogl == 1) {
    USBNWrite(TXC1, TX_LAST+TX_EN+TX_TOGL);
    usbprog.datatogl = 0;
  } else {
    USBNWrite(TXC1, TX_LAST+TX_EN);
    usbprog.datatogl = 1;
  }
  SREG = sreg;
}

void SendCompleteAnswer(void)
{
  uint8_t size, sreg = SREG;
  cli();

  if(usbprog.send_pos >= usbprog.fill_pos) {
    SREG = sreg;
    return;
  }

  USBNWrite(TXC1, FLUSH);
  if (usbprog.fill_pos > usbprog.send_pos + 64)
  {
    size = 64;
  }
  else
  {
    size = usbprog.fill_pos - usbprog.send_pos;
  }
  USBNWriteBlock(TXD1, (unsigned char*)answer + usbprog.send_pos, size, 0);
  usbprog.send_pos += size;

  pgmmode.numbytes = 0;
  USBToglAndSend();
  
  SREG = sreg;
}

#if 0
void NackEvent(unsigned int number)
{
  if(number & 0x20) 
    USBNWrite(TXC1, FLUSH);

  if(number & 0x02) 
    USBNWrite(RXC1, FLUSH+RX_EN);
    USBNWrite(TXC1, FLUSH);
    if(usbprog.datatogl == 1) {
      USBNWrite(TXC1, TX_LAST+TX_EN+TX_TOGL);
      usbprog.datatogl=0;
    } else {
      USBNWrite(TXC1, TX_LAST+TX_EN);
      usbprog.datatogl=1;
    }
  }
}
#endif

void QueueAnswerByte(uint8_t data)
{
  uint8_t sreg;

  sreg = SREG;
  cli();

  answer[usbprog.fill_pos++] = data;
  if (usbprog.ready_to_transmit && usbprog.fill_pos >= usbprog.send_pos + 64) {
    CommandAnswer(64);
  }

  SREG = sreg;
}

void QueueFirstAnswerByte(uint8_t data)
{
  uint8_t sreg;

  sreg = SREG;
  cli();

  usbprog.ready_to_transmit = 1;
  usbprog.complete = 0;
  usbprog.fill_pos = 0;
  usbprog.send_pos = 0;

  SREG = sreg;

  QueueAnswerByte(data);

}

void QueueLastAnswerByte(uint8_t data)
{
  uint8_t sreg;
  uint16_t size;

  QueueAnswerByte(data);

  sreg = SREG;
  cli();

  usbprog.complete = 1;
  if (usbprog.ready_to_transmit) {
    size = usbprog.fill_pos - usbprog.send_pos;
    CommandAnswer(size > 64 ? 64 : size);
  }

  SREG = sreg;
}

void CommandAnswer(uint8_t length)
{
  uint8_t sreg = SREG;
  cli();
  USBNWrite(TXC1, FLUSH);
  USBNWriteBlock(TXD1, (uint8_t*)answer + usbprog.send_pos, length, 0);
  USBToglAndSend(); /* control togl bit */
  usbprog.send_pos += length;
  usbprog.ready_to_transmit = 0;
  SREG = sreg;
}

/** Enter ISP programming mode
 */
void cmd_enter_progmode(struct cmd_enter_progmode_s *cmd) {
  #if DEBUG_ON
  UARTWrite("enter\r\n");
  #endif
  #if DEBUG_ON
  int i;
  for (i=0;i<12;i++) {
    SendHex(*((char *)cmd+i));
    UARTPutChar(' ');
  }
  SendHex(cmd->cmd1);
  #endif
  pgmmode.address = 0;
  pgmmode.status = STATUS_CMD_OK;
  pgmmode.ext_address = 0xffff;
  pgmmode.large_flash = 0;
  pgmmode.poll_address_valid = 0;
  spi_active();
  LED_on;

  PORTB &= ~(1<<SCK);

  if(usbprog.reset_pol==1)
  {
    PORTB &= ~(1<<RESET_PIN);
    wait_ms(10);
    PORTB |= (1<<RESET_PIN);  // give reset a positive pulse
    wait_ms(10);
    PORTB &= ~(1<<RESET_PIN);
    wait_ms(10);
    
    #ifdef AT89MODE 
    PORTB |= (1<<RESET_PIN);  // give reset a positive pulse
    wait_ms(10);
    #endif
  }
  else
  {
    PORTB |= (1<<RESET_PIN);  // give reset a positive pulse
    wait_ms(10);
    PORTB &= ~(1<<RESET_PIN);
    wait_ms(10);
    PORTB |= (1<<RESET_PIN);  // give reset a positive pulse
    wait_ms(10);
  }

  QueueFirstAnswerByte(CMD_ENTER_PROGMODE_ISP);

  wait_ms(cmd->cmdexedelay);

  int syncloops = cmd->synchloops;
  unsigned char result3,result4;
  for (;syncloops > 0; syncloops--) {
    spi_out(cmd->cmd1);
    wait_ms(cmd->bytedelay);
    spi_out(cmd->cmd2);
    wait_ms(cmd->bytedelay);
    result3 = spi_inout(cmd->cmd3);
    wait_ms(cmd->bytedelay);
    result4 = spi_inout(cmd->cmd4);
    wait_ms(cmd->bytedelay);

    if ((cmd->pollindex==0) ||
        ((cmd->pollindex==3) && (result3==cmd->pollvalue)) ||
        ((cmd->pollindex==4) && (result4==cmd->pollvalue))) {
      QueueLastAnswerByte(STATUS_CMD_OK);
      return;
    }

    //Apparently, we're not in sync. Pulse SCK once to get in sync
    #if DEBUG_ON
    UARTPutChar('#');
    #endif
    spi_pulseclockonce();
  };

  QueueLastAnswerByte(STATUS_CMD_FAILED);
  return;
}

/** Send/receive multiple spi bytes
 */
void cmd_spi_multi(struct cmd_spi_multi_s *cmd) {
  char send,receive;
  /*numsent *must* be int because char may overflow if numRx+rxStart
    becomes too large to fit in char */

  int numsent=0,numrecv=0;
  if (cmd->numTx>60||cmd->numRx>60) {
    //Not implemented
    QueueFirstAnswerByte(CMD_SPI_MULTI);
    QueueLastAnswerByte(STATUS_CMD_UNKNOWN);
  } else {
    QueueFirstAnswerByte(CMD_SPI_MULTI);
    QueueAnswerByte(STATUS_CMD_OK);
    for(numsent=0;(numsent<cmd->numTx)||(numrecv<cmd->numRx);numsent++) {
      if (numsent<cmd->numTx) send=cmd->txData[numsent]; else send=0;
      receive=spi_inout(send);
      if (numsent>=cmd->rxStart) {
        //past this point, the data received from the AVR must be
        //sent to the USB
        QueueAnswerByte(receive);
        numrecv++;
      }
    }
    QueueLastAnswerByte(STATUS_CMD_OK);
  }
}


/* central command parser */
void USBFlash(char *buf)
{
  char result = 0;
  uint16_t numbytes;

  DPRINT(PSTR("USBFlash()\n"));
  
  USBNWrite(TXC1, 0x00);
  
  USBNWrite(TXC1, FLUSH);
  // first see if this packet is expected by Flash or EEPROM programming
  if(usbprog.longpackage) {
    #if DEBUG_ON
    UARTWrite("USBFlash longpackage\r\n ");
    #endif

    if(usbprog.lastcmd == CMD_PROGRAM_FLASH_ISP) {// last operation was flash programming
       program_fsm((uint8_t*)buf, 0);
    }

    if(usbprog.lastcmd == CMD_PROGRAM_EEPROM_ISP) {// last operation was eeprom programming
       program_fsm((uint8_t*)buf, 1);
    }
    return;
  }

  // if not, this is a command packet, we will decode here
  else {
    #if DEBUG_ON
    UARTWrite("cmd ");
    SendHex(buf[0]);
    UARTWrite("cmd\r\n");
    #endif

    static uint8_t last_cmd;
    last_cmd = usbprog.lastcmd;
    usbprog.lastcmd = buf[0]; // store current command for later use
    switch(buf[0]) {
    
    case CMD_SIGN_ON:
      DPRINT1(PSTR("  CMD_SIGN_ON\n"));
      QueueFirstAnswerByte(CMD_SIGN_ON);
      QueueAnswerByte(STATUS_CMD_OK);
      QueueAnswerByte(10);              // fixed length
      QueueAnswerByte('A');
      QueueAnswerByte('V');
      QueueAnswerByte('R');
      QueueAnswerByte('I');
      QueueAnswerByte('S');
      QueueAnswerByte('P');
      QueueAnswerByte('_');
      QueueAnswerByte('M');
      QueueAnswerByte('K');
      QueueLastAnswerByte('2');

      return;
    break;
    case CMD_SET_PARAMETER:
      DPRINT1(PSTR("  CMD_SET_PARAMETER\n"));
      switch(buf[1]){
        case PARAM_RESET_POLARITY:
          DPRINT(PSTR("    PARAM_RESET_POLARITY(%u)\n"), buf[2]);
          if(buf[2]==0x00)
            usbprog.reset_pol=0;
          else
            usbprog.reset_pol=1;

        break;

        case PARAM_SCK_DURATION:
          DPRINT1(PSTR("    PARAM_SCK_DURATION (%u)\n"), buf[2]);
	  #if DEBUG_ON
          UARTPutChar('S');
          SendHex(buf[2]);
	  #endif
          if (buf[2]>MAX_SCK_DURATION) buf[2]=MAX_SCK_DURATION;
          if (buf[2] != usbprog.sck_duration)
          {
            usbprog.sck_duration = buf[2];
            eeprom_write_byte(&ee_sck_duration, usbprog.sck_duration);
          }
          spi_init();
          break;
        default:
          DPRINT(PSTR("    unknown (%02x)\n"), buf[1]);
      }
      // do we like, all commands are successfully
      QueueFirstAnswerByte(CMD_SET_PARAMETER);
      QueueLastAnswerByte(STATUS_CMD_OK);

      return;
    break;
    case CMD_GET_PARAMETER:
      DPRINT1(PSTR("  CMD_GET_PARAMETER\n"));
      QueueFirstAnswerByte(CMD_GET_PARAMETER);
      QueueAnswerByte(STATUS_CMD_OK);

      switch(buf[1]){
        case PARAM_STATUS_TGT_CONN:
          DPRINT1(PSTR("    PARAM_STATUS_TGT_CONN=STATUS_ISP_READY\n"));
          QueueLastAnswerByte(STATUS_ISP_READY);
        break;

        case PARAM_SW_MAJOR:  // avrisp mkII special 1
          DPRINT1(PSTR("    PARAM_SW_MAJOR=9\n"));
          QueueLastAnswerByte(9);
        break;

        case PARAM_SW_MINOR:  // abrisp mkII special 6
          DPRINT1(PSTR("    PARAM_SW_MINOR=9\n"));
          QueueLastAnswerByte(9);
        break;

        case PARAM_HW_VER:
          DPRINT1(PSTR("    PARAM_HW_VER=0\n"));
          QueueLastAnswerByte(0);
        break;

        case PARAM_VTARGET:
          DPRINT1(PSTR("    PARAM_VTARGET=50\n"));
          QueueLastAnswerByte(50);
        break;

        case PARAM_SCK_DURATION:
          DPRINT1(PSTR("    PARAM_SCK_DURATION=%u\n"), usbprog.sck_duration);
          QueueLastAnswerByte(usbprog.sck_duration);
        break;

        default:
          DPRINT1(PSTR("    unknown (%02x)\n"), buf[1]);
          QueueLastAnswerByte(0x00); // FIXME all is not perfect!
      }
      return;

    break;
    case CMD_OSCCAL:
      DPRINT1(PSTR("  CMD_OSCCAL\n"));
      /* peforms a calibration secquence */
      QueueFirstAnswerByte(CMD_OSCCAL);
      QueueLastAnswerByte(STATUS_CMD_OK);
      return;
    break;

    case CMD_READ_OSCCAL_ISP:
      DPRINT1(PSTR("  CMD_READ_OSCCAL_ISP\n"));
      QueueFirstAnswerByte(CMD_READ_OSCCAL_ISP);
      QueueAnswerByte(STATUS_CMD_OK);
      spi_out(buf[2]);
      spi_out(buf[3]);
      spi_out(buf[4]);
      QueueAnswerByte(spi_in());
      QueueLastAnswerByte(STATUS_CMD_OK);
      return;

    break;

    case CMD_LOAD_ADDRESS:
      DPRINT1(PSTR("  CMD_LOAD_ADDRESS (%02x%02x%02x%02x)\n"), buf[4], buf[3], buf[2], buf[1]);
      // set given address
      pgmmode.address = buf[1];
      pgmmode.address = (pgmmode.address << 8) | buf[2];
      pgmmode.address = (pgmmode.address << 8) | buf[3];
      pgmmode.address = (pgmmode.address << 8) | buf[4];
      pgmmode.large_flash = buf[1] & 0x80 ? 1 : 0;
      QueueFirstAnswerByte(CMD_LOAD_ADDRESS);
      QueueLastAnswerByte(STATUS_CMD_OK);
      return;
    break;

    case CMD_FIRMWARE_UPGRADE:
      DPRINT1(PSTR("  CMD_FIRMWARE_UPGRADE\n"));
#ifdef DEBUG
      cli();
      flashUartTxData();  
      _delay_ms(1);
#endif
      avrupdate_start();
      return;
    break;

    case CMD_RESET_PROTECTION:
      DPRINT1(PSTR("  CMD_RESET_PROTECTION\n"));
      QueueFirstAnswerByte(CMD_RESET_PROTECTION);
      QueueLastAnswerByte(STATUS_CMD_OK);  // this command returns always ok!
      return;
    break;

    case CMD_ENTER_PROGMODE_ISP:
      DPRINT1(PSTR("  CMD_ENTER_PROGMODE_ISP\n"));
      cmd_enter_progmode((struct cmd_enter_progmode_s *)buf);
      pgmmode.poll_address_valid = 0;
      pgmmode.status = STATUS_CMD_OK;
      return;

      break;

    case CMD_LEAVE_PROGMODE_ISP:
      DPRINT1(PSTR("  CMD_LEAVE_PROGMODE_ISP\n"));
      #if DEBUG_ON
      UARTWrite("leave\r\n");
      #endif
      
      // clear extende addr 
      spi_cmd(0x4d, 0x0000, 0);

      LED_off;
      RESET_high;
      spi_idle();
      QueueFirstAnswerByte(CMD_LEAVE_PROGMODE_ISP);
      QueueLastAnswerByte(STATUS_CMD_OK);

      // wenn adapter vom avrdude aus angesteuert wird
      if(usbprog.avrstudio==0)
      usbprog.datatogl=0;  // to be sure that togl is on next session clear 
      //usbprog.datatogl=1;  // to be sure that togl is on next session clear
      //USBNWrite(RXC1, RX_EN);

      return;
    break;

    case CMD_CHIP_ERASE_ISP:
      DPRINT1(PSTR("  CMD_CHIP_ERASE_ISP\n"));
      #if DEBUG_ON
      UARTWrite("erase\r\n");
      #endif
      spi_out(buf[3]);
      spi_out(buf[4]);
      spi_out(0x00);
      spi_out(0x00);
      wait_ms(buf[1]);
      QueueFirstAnswerByte(CMD_CHIP_ERASE_ISP);
      QueueLastAnswerByte(STATUS_CMD_OK);
      return;
    break;

    case CMD_PROGRAM_FLASH_ISP:
      if (last_cmd != usbprog.lastcmd)
      {
        DPRINT1(PSTR("  CMD_PROGRAM_FLASH_ISP\n"));
      }
      #if DEBUG_ON 
      UARTWrite("pflash\r\n");
      for(i=0;i<10;i++) {
        SendHex(buf[i]);
        UARTPutChar(' ');
      }
      UARTWrite("len ");
        SendHex(buf[2]);
      UARTWrite(" \r\n");
      #endif

      pgmmode.numbytes = (buf[1] << 8) | (buf[2]);

      // buf[3] = mode
      pgmmode.mode = buf[3];
      // buf[4] = delay
      pgmmode.delay = buf[4];

      // buf[5] = spi command for load page and write program memory (one byte at a time)
      pgmmode.cmd1 = buf[5];
      // buf[6] = spi command for write program memory page (one page at a time)
      pgmmode.cmd2 = buf[6];
      // buf[7] = spi command for read program memory
      pgmmode.cmd3 = buf[7];
      // buf[8] = poll value #1 (used for flash programming)
      pgmmode.poll1 = buf[8];
      // buf[9] = poll value #2 (used for eeprom programming)
      pgmmode.poll2 = buf[9];

      // store the page address (which in fact are the first 5 bits) for page write command
      // because we will increment pgmode.address during transfer
      pgmmode.pageaddress = pgmmode.address;
      usbprog.cmdpackage = 1;
      program_fsm((uint8_t*)buf, 0);
      usbprog.cmdpackage = 0;
    break;

    case CMD_READ_FLASH_ISP:
      if (last_cmd != usbprog.lastcmd)
      {
        DPRINT1(PSTR("  CMD_READ_FLASH_ISP\n"));
      }
      #if DEBUG_ON
      UARTWrite("rflash\r\n");
      #endif
      numbytes = (buf[1] << 8) | (buf[2]);
      pgmmode.cmd3 = buf[3];  // read command
      pgmmode.numbytes = numbytes + 3;
      QueueFirstAnswerByte(CMD_READ_FLASH_ISP);
      QueueAnswerByte(STATUS_CMD_OK);
      // collect max first 62 bytes
      while (numbytes--) {
        if (pgmmode.large_flash && (pgmmode.address >> 16) != pgmmode.ext_address){
          pgmmode.ext_address = pgmmode.address >> 16;
          spi_cmd(0x4d, pgmmode.ext_address, 0);
        }
        QueueAnswerByte(spi_cmd(pgmmode.cmd3, pgmmode.address, 0));

        if(pgmmode.cmd3 == 0x20){
          pgmmode.cmd3 = 0x28;
        }
        else {
          pgmmode.cmd3 = 0x20;
          pgmmode.address++;
        }
      }

      // then toggle send next read bytes
      // and finish with status_cmd_ok
      QueueLastAnswerByte(STATUS_CMD_OK);
      return;
    break;

    case CMD_READ_LOCK_ISP:
      DPRINT1(PSTR("  CMD_READ_LOCK_ISP\n"));
      #if DEBUG_ON
      UARTWrite("rlock\r\n");
      #endif
      spi_out(buf[2]);
      spi_out(buf[3]);
      spi_out(buf[4]);
      result = spi_in();

      QueueFirstAnswerByte(CMD_READ_LOCK_ISP);
      QueueAnswerByte(STATUS_CMD_OK);
      QueueAnswerByte(result);
      QueueLastAnswerByte(STATUS_CMD_OK);
      return;
    break;

    case CMD_PROGRAM_LOCK_ISP:
      DPRINT1(PSTR("  CMD_PROGRAM_LOCK_ISP\n"));
      #if DEBUG_ON
      UARTWrite("plock\r\n");
      #endif
      spi_out(buf[1]);
      spi_out(buf[2]);
      spi_out(buf[3]);
      spi_out(buf[4]);

      QueueFirstAnswerByte(CMD_PROGRAM_LOCK_ISP);
      QueueAnswerByte(STATUS_CMD_OK);
      QueueLastAnswerByte(STATUS_CMD_OK);
      return;
    break;

    case CMD_PROGRAM_EEPROM_ISP:
      if (last_cmd != usbprog.lastcmd) 
      {
        DPRINT(PSTR("  CMD_PROGRAM_EEPROM_ISP\n"));
      }
      #if DEBUG_ON
      UARTWrite("peerpom\r\n");
      #endif
      // buf[1..2] = number of bytes to program
      pgmmode.numbytes = (buf[1] << 8) | (buf[2]);
      // buf[3] = mode
      pgmmode.mode = buf[3];
      // buf[4] = delay
      pgmmode.delay = buf[4];
      // buf[5] = spi command for load page and write program memory (one byte at a time)
      pgmmode.cmd1 = buf[5];
      // buf[6] = spi command for write program memory page (one page at a time)
      pgmmode.cmd2 = buf[6];
      // buf[7] = spi command for read program memory
      pgmmode.cmd3 = buf[7];
      // buf[8] = poll value #1 (used for flash programming)
      pgmmode.poll1 = buf[8];
      // buf[9] = poll value #2 (used for eeprom programming)
      pgmmode.poll2 = buf[9];

      // store the page address (which in fact are the first 5 bits) for page write command
      // because we will increment pgmode.address during transfer
      pgmmode.pageaddress = pgmmode.address;

      usbprog.cmdpackage = 1;
      program_fsm((uint8_t*)buf, 1);
      usbprog.cmdpackage = 0;
      return;
    break;

    case CMD_READ_EEPROM_ISP:
      if (last_cmd != usbprog.lastcmd)
      {
        DPRINT(PSTR("  CMD_READ_EEPROM_ISP\n"));
      }
      QueueFirstAnswerByte(CMD_READ_EEPROM_ISP);
      QueueAnswerByte(STATUS_CMD_OK);
      numbytes = (buf[1] << 8) | (buf[2]); // number of bytes
      pgmmode.numbytes = numbytes + 3;
      pgmmode.cmd3 = buf[3];  // read command
      // collect max first 62 bytes
      while(numbytes--) {
        QueueAnswerByte(spi_cmd(pgmmode.cmd3, pgmmode.address, 0));
        pgmmode.address++;
      }

      // then toggle send next read bytes
      // and finish with status_cmd_ok
      QueueLastAnswerByte(STATUS_CMD_OK);
	  return; 
    break;

    case CMD_PROGRAM_FUSE_ISP:
      DPRINT1(PSTR("  CMD_PROGRAM_FUSE_ISP\n"));
      #if DEBUG_ON
      UARTWrite("pfuse\r\n");
      #endif
      spi_out(buf[1]);
      spi_out(buf[2]);
      spi_out(buf[3]);
      spi_out(buf[4]);

      QueueFirstAnswerByte(CMD_PROGRAM_FUSE_ISP);
      QueueAnswerByte(STATUS_CMD_OK);
      QueueLastAnswerByte(STATUS_CMD_OK);
      return;

    break;

    case CMD_READ_FUSE_ISP:
      DPRINT1(PSTR("  CMD_READ_FUSE_ISP\n"));
      #if DEBUG_ON 
      UARTWrite("rfuse\r\n");
      #endif
      spi_out(buf[2]);
      spi_out(buf[3]);
      spi_out(buf[4]);
      result = spi_in();

      QueueFirstAnswerByte(CMD_READ_FUSE_ISP);
      QueueAnswerByte(STATUS_CMD_OK);
      QueueAnswerByte(result);
      QueueLastAnswerByte(STATUS_CMD_OK);
      return;
    break;

    case CMD_READ_SIGNATURE_ISP:
      DPRINT1(PSTR("  CMD_READ_SIGNATURE_ISP\n"));
      spi_out(buf[2]);
      spi_out(buf[3]);
      spi_out(buf[4]);
      result = spi_in();

      QueueFirstAnswerByte(CMD_READ_SIGNATURE_ISP);
      QueueAnswerByte(STATUS_CMD_OK);
      QueueAnswerByte(result);
      QueueLastAnswerByte(STATUS_CMD_OK);
      return;
    break;

    case CMD_SPI_MULTI:
      if (last_cmd != usbprog.lastcmd) 
      {
        DPRINT(PSTR("  CMD_SPI_MULTI\n"));
      }
      //usbprog.datatogl=0;
      #if DEBUG_ON
      UARTWrite("multi\r\n");
      #endif
      usbprog.avrstudio=0;  // only avrdude use this command // this is not true! also at89 @ avr studio
      cmd_spi_multi((struct cmd_spi_multi_s *)buf);
      return;
    break;
    }
  }
}

uint8_t buffer[128+64];
volatile uint8_t count = 0;
volatile uint8_t pos = 0;

uint8_t USBNGetRxData(uint8_t ep, uint8_t *buffer, uint8_t size);
uint8_t USBNGetRxStatus(uint8_t ep);
void USBNEnableRx(uint8_t ep);

void RxFunction(void)
{
    uint8_t buf[64];
    char line[100];
    uint8_t *ptr = buf;
    uint8_t size = USBNGetRxData(1, buf, 64);

    DPRINT(PSTR("RxFunction()\n"));

    for (uint8_t i = 0, len = 0; i < size; i += 16)
    {
        for (uint8_t j = 0; j < 16 && i + j < size; j++)
        {
            len += sprintf(line + len, " %02x", buf[i + j]);
        }
        DPRINT(PSTR("   %s\n"), line);
    }


    while(size--)
    {
        buffer[pos++] = *ptr++;
        if (pos >= sizeof(buffer))
        {
            pos = 0;
        }
        if (count < sizeof(buffer))
        {
            count++;
        }
    }
    

}

void Worker(void)
{
    uint8_t buf[64];
    uint8_t sreg;
    uint8_t size = 0;
    uint8_t *ptr = buf;

    sreg = SREG;
    cli();

    if (count)
    {
        size = count;
        if (size > 64)
        {
            size = 64;
        }
        while(size--)
        {
            if (count > pos)
            {
                *ptr++ = buffer[pos + sizeof(buffer) - count];
            }
            else
            {
                *ptr++= buffer[pos - count];
            }
            count--;
        }
   
        SREG = sreg;


        USBFlash((char*)buf);
    }
    else
    {
        SREG = sreg;
    }
}
void RequestTransmit(void)
{
    uint8_t sreg;
    uint16_t size;

    sreg = SREG;
    cli();

    usbprog.ready_to_transmit = 1;
    size = usbprog.fill_pos - usbprog.send_pos;
    if (size >= 64) {
      CommandAnswer(64);
    } else if (usbprog.complete && size) {
      CommandAnswer(size);
    }

    SREG = sreg;
}

uint8_t USBNGetRxData(uint8_t ep, uint8_t *buffer, uint8_t size);
uint8_t USBNGetRxStatus(uint8_t ep);
void USBNEnableRx(uint8_t ep);

uint16_t USBNGetCurrentConfig(void);
           
void USBNAddInEndpointCallback(uint8_t epnr, void (*fkt)(void));
void USBNAddOutEndpointCallback(uint8_t epnr, void (*fkt)(void));

// USB device parameters
struct usb_device_descriptor PROGMEM avrispmk2klonDevice =
{
    .bLength = sizeof(struct usb_device_descriptor),
    .bDescriptorType=DEVICE,
    .bcdUSB=0x0110,
    .bDeviceClass=0xff,
    .bDeviceSubClass=0x00,
    .bDeviceProtocol=0x00,
    .bMaxPacketSize0=0x08,
    .idVendor = 0x03eb,
    .idProduct = 0x2104,
    .bcdDevice = 0x0200,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

// configuration descriptor
struct
{
    struct usb_configuration_descriptor Config;
    struct usb_interface_descriptor Interface;
    struct usb_endpoint_descriptor DataInEndpoint;
    struct usb_endpoint_descriptor DataOutEndpoint;
}PROGMEM avrispmk2klonConf =
{
    .Config =
    {
        .bLength = sizeof(struct usb_configuration_descriptor),
        .bDescriptorType = CONFIGURATION,
        .wTotalLength = sizeof(avrispmk2klonConf),
        .bNumInterfaces = 1,
        .bConfigurationValue = 1,
        .iConfiguration = 0,
        .bmAttributes = 0xC0,   // bus powered alt 0x80
        .MaxPower = 0x32, // alt 0x1a
    },
    .Interface = 
    {
        .bLength = sizeof(struct usb_interface_descriptor),
        .bDescriptorType = INTERFACE,
        .bInterfaceNumber = 0,
        .bAlternateSetting = 0,
        .bNumEndpoints = 2,
        .bInterfaceClass = 0xff,
        .bInterfaceSubClass = 0,
        .bInterfaceProtocol = 0,
        .iInterface = 0,
    },
    .DataInEndpoint = 
    {
        .bLength = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType = ENDPOINT,
        .bEndpointAddress = 0x82,
        .bmAttributes = 0x02,
        .wMaxPacketSize = 64,
        .bIntervall = 10,
    },
    .DataOutEndpoint = 
    {
        .bLength = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType = ENDPOINT,
        .bEndpointAddress = 0x02,
        .bmAttributes = 0x02,
        .wMaxPacketSize = 64,
        .bIntervall = 10,
    },
};

struct usb_configuration_descriptor_tab PROGMEM avrispmk2klonConfigTab =
{
  .NumberOfConfigurations = 1,
  .Configurations = 
  {
    (struct usb_configuration_descriptor*)&avrispmk2klonConf,
  }
};        

// string descriptoren
struct usb_wstring_descriptor PROGMEM LanguageString = 
{
    .bLength = 4,
    .bDescriptorType = STRING,
    .wString = {0x0409},
};

struct usb_wstring_descriptor PROGMEM ManufacturerString = 
{
    .bLength = 2 * 5 + 2,
    .bDescriptorType = STRING,
    .wString = {'A', 'T', 'M', 'E', 'L'}
};

struct usb_wstring_descriptor PROGMEM ProductString = 
{
    .bLength = 2 * 11 + 2,
    .bDescriptorType = STRING,
    .wString = {'A', 'V', 'R', 'I', 'S', 'P', ' ', 'm', 'k', 'I', 'I'}
};

struct usb_wstring_descriptor PROGMEM SerialNumber =
{
    .bLength = 2 * 13 + 2,
    .bDescriptorType = STRING,
    //.wString = {'0', '0', '0', '0', 'A', '0', '0', '1', '2', '8', '2', '5', '6', 'X', '1', '2', '3'}
    .wString = {'0','0', '0', '2', '0', '0', '0', '9', '0', '6', '3', '8', 0}
};

struct usb_wstring_descriptor_tab PROGMEM avrispmk2klonStringTab =
{
    .NumberOfStrings = 4,
    .Strings = 
    {
        (struct usb_wstring_descriptor*)&LanguageString, 
        (struct usb_wstring_descriptor*)&ManufacturerString, 
        (struct usb_wstring_descriptor*)&ProductString, 
        (struct usb_wstring_descriptor*)&SerialNumber
    }
};

volatile uint32_t ticker;

void initTimer(void)
{
    uint8_t sreg = SREG;
    cli();
#if defined(__AVR_ATmega32__) || defined(__AVR_ATmega16__) 
    TCCR0 = 0;

    ticker = 0;

    TCNT0 = 0;
    OCR0 = 250-1;
    TCCR0 = (1<<WGM01)|(1<<CS01)|(1<<CS00); // ctc mode, CLK/64
    TIMSK &= ~(1<<TOIE0);
    TIMSK |= (1<<OCIE0);
#elif defined(__AVR_ATmega644__)
    TCCR0A = 0;
    TCCR0B = 0;

    ticker = 0;

    TCNT0 = 0;
    OCR0A = 199;
    OCR0B = 0;
    TCCR0A = (1<<WGM01); // ctc mode
    TCCR0B = (1<<CS01);

    TIMSK0 &= ~((1<<TOIE0)|(1<<OCIE0B));;
    TIMSK0 |= (1<<OCIE0A);
#else
#endif
    SREG = sreg;
}

uint32_t getTicker(void)
{
    uint32_t result;
    uint8_t sreg = SREG;
    cli();
    result = ticker;
    SREG = sreg;
    return result;
}

#if defined( __AVR_ATmega32__) ||defined( __AVR_ATmega16__)
ISR(TIMER0_COMP_vect)
#elif defined(__AVR_ATmega644__)
ISR(TIMER0_COMPA_vect)
#else
#endif 
{
    ticker++;
}     

int main(void)
{
#ifdef DEBUG
  initTimer();
  initUart();
  initDebug();
#endif


  spi_init();
  spi_idle();

  USBNInitMC();

  USBNInit((struct usb_device_descriptor*)&avrispmk2klonDevice,
           (struct usb_configuration_descriptor_tab*)&avrispmk2klonConfigTab, 
           (struct usb_wstring_descriptor_tab*)&avrispmk2klonStringTab);

  usbprog.longpackage = 0;
  usbprog.avrstudio = 1;   // 1 no
  usbprog.send_pos = 0;
  usbprog.reset_pol = 1;  // 1= avr 0 = at89
  usbprog.datatogl=0; 

  DDRA = (1 << PA4);
  LED_off;
  RESET_high;

  USBNAddInEndpointCallback(1, RequestTransmit);




  sei();

  DPRINT1(PSTR("=========== AVRISP mk2 Clone ===========\n"));

  // start usb chip
  USBNStart();

  while(1)
  {
#if 0
    Worker();  
    _delay_us(250);
#else
    if (USBNGetRxStatus(1))
    {
        uint8_t Buffer[64];
        uint8_t size;
        size = USBNGetRxData(1, Buffer, 64);
        memset(Buffer + size, 0, sizeof(Buffer) - size);

#if defined(DEBUG) && !defined(NDEBUG) 

        DPRINT(PSTR("=====================================================\n"));

        for (uint8_t i = 0; i < size; i += 16)
        {
            uint8_t buf_len = 0;
            char line[100];
            for (uint8_t j = 0; j < 16 && i + j < size; j++)
            {
                buf_len += sprintf_P(line + buf_len, PSTR("%02x "), Buffer[i + j]);
            }
            DPRINT(PSTR("%s\n"), line);

        }
        DPRINT(PSTR("=====================================================\n"));
#endif
        USBFlash((char*)Buffer);
    }
    else
    {
//        _delay_us(100);
    }
#endif    
  }
}


