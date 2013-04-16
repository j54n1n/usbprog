/*
    usbprog-jtag, by Cahya Wirawan <cahya@gmx.at> 
    Based on estick-jtag.
    Released under the MIT Licence.
*/

#ifndef _USBPROG_JTAG_H_
#define _USBPROG_JTAG_H_

	/* Includes: */
	#include <avr/io.h>
	#include <avr/wdt.h>
	#include <avr/power.h>
	#include <util/delay_basic.h>

	/* Macros: */

	/* Type Defines: */
	
  #define VID 0x1781
  #define PID 0x0c63
  #define FRAME_SIZE 64
	#define USBPROG_USB_BUFFER_SIZE 510
	#define USBPROG_USB_BUFFER_OFFSET 2
	#define USBPROG_IN_BUFFER_SIZE	(USBPROG_USB_BUFFER_SIZE)
	#define USBPROG_OUT_BUFFER_SIZE  (USBPROG_USB_BUFFER_SIZE)

	/* Global Variables: */

	/* Function Prototypes: */

  void ProcessData(void);
  void USBMessageSend(uint8_t *, uint16_t);

#endif //USBPROG_JTAG
