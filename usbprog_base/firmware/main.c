#include <stdlib.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/boot.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "../../usbn2mc/tiny/usbnapi.h"
#include "usbn2mc.h"


// Changes: Deleted all the UART stuff. it's just needed for debugging.

// Fixed bug discussed in threat: "USBProg doesn't save Firmware"
// by setting PD0 internal pull-up


// __heap_start is declared in the linker script

/* external interrupt from usbn9604 */
ISR (INT0_vect)
{
  USBNInterrupt ();
}

// Values of state
#define NONE		0x00

#define STARTAPP	0x01
#define WRITEPAGE	0x02
// #define GETVERSION   0x03
// #define SETVERSION   0x04
#define STOPPROGMODE	0x05

// USB device parameters
struct usb_device_descriptor PROGMEM avrupdateDevice = 
{
    .bLength = sizeof(struct usb_device_descriptor),
    .bDescriptorType = DEVICE,
    .bcdUSB = 0x0110,
    .bDeviceClass = 0,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 8,
    .idVendor = 0x1781,
    .idProduct = 0x0c62,
    .bcdDevice = 0x0000,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 0,
    .bNumConfigurations = 1
};

// configuration descriptor
struct
{
    struct usb_configuration_descriptor Config;
    struct usb_interface_descriptor Interface;
    struct usb_endpoint_descriptor DataOutEndpoint;
} PROGMEM avrupdateConf = 
{
    .Config =
    {
        .bLength = sizeof(struct usb_configuration_descriptor),
        .bDescriptorType = CONFIGURATION,
        .wTotalLength = sizeof(avrupdateConf),
        .bNumInterfaces = 1,
        .bConfigurationValue = 1,
        .iConfiguration = 0,
        .bmAttributes = 0xA0,   // bus powerded, remote wakup support
        .MaxPower = 0x1a,       // 25mA
    },
    .Interface = 
    {
        .bLength = sizeof(struct usb_interface_descriptor),
        .bDescriptorType = INTERFACE,
        .bInterfaceNumber = 0,
        .bAlternateSetting = 0,
        .bNumEndpoints = 1,
        .bInterfaceClass = 0,
        .bInterfaceSubClass = 0,
        .bInterfaceProtocol = 0,
        .iInterface = 0,
    },
    .DataOutEndpoint = 
    {
        .bLength = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType = ENDPOINT,
        .bEndpointAddress = 0x02,
        .bmAttributes = 0x02,   // bulk
        .wMaxPacketSize = 64,
        .bIntervall = 0,
    }
};

struct usb_configuration_descriptor_tab PROGMEM avrupdateConfTab =
{
    .NumberOfConfigurations = 1,
    .Configurations = 
    {
        (struct usb_configuration_descriptor*)&avrupdateConf
    }
};   

struct usb_wstring_descriptor avrupdateStrLanguage PROGMEM = 
{
    .bLength = 4,
    .bDescriptorType = STRING,
    .wString = {0x0409}
};

struct usb_wstring_descriptor avrupdateStrManufacturer PROGMEM = 
{
    .bLength = sizeof(L"USBprog EmbeddedProjects"), 
    .bDescriptorType = STRING,
    .wString = L"USBprog EmbeddedProjects",
};

struct usb_wstring_descriptor PROGMEM avrupdateStrProduct = 
{
    .bLength = sizeof(L"usbprogBase Mode"),
    .bDescriptorType = STRING,
    .wString = L"usbprogBase Mode"
};

struct usb_wstring_descriptor_tab PROGMEM StringTab =
{
    .NumberOfStrings = 3,
    .Strings =
    {
        (struct usb_wstring_descriptor*)&avrupdateStrLanguage,
        (struct usb_wstring_descriptor*)&avrupdateStrManufacturer,
        (struct usb_wstring_descriptor*)&avrupdateStrProduct
    }
};

uint16_t page_addr;
uint16_t page_addr_w;
uint8_t pageblock[128];
uint8_t collect128;
uint8_t state;
uint8_t address EEMEM = 1;

/* usbn2mc tiny needs this */
void
USBNDecodeVendorRequest (DeviceRequest * req)
{
}

void
USBNDecodeClassRequest (DeviceRequest * req/*, EPInfo * ep*/)
{
}

/* pointer to the beginning of application code */
void (*avrupdate_jump_to_app) (void) = 0x0000;

/*  wait function */
void
wait_ms (int ms)
{
  uint16_t i;
  for (i = 0; i < ms; i++)
    _delay_ms (1);
}

/* disable all hardware interrupts */
void disable_all_interrupts(void)
{
  cli();
  GICR &= ~((1<<INT1)|(1<<INT0)|(1<<INT2));
  TIMSK = 0;
  SPCR &= ~(1<<SPIE);
  UCSRB &= ~((1<<RXCIE)|(1<<TXCIE)|(1<<UDRIE));
  TWCR &= ~(1<<TWIE);
  ACSR &= ~(1<<ACIE);
  ADCSRA &= ~(1<<ADIE);
  SPMCR &= ~(1<<SPMIE);
  EECR &= ~(1<<EERIE);
}

/* pogramm a page into flash 
 *	@page = number of page
 *	global pageblock = data
 */
void
avrupdate_program_page (uint32_t page)
{
  uint16_t i;
  uint8_t sreg;

  //SendHex(page);
  page = page * 128;
  sreg = SREG;
  cli ();

  eeprom_busy_wait ();

  boot_page_erase (page);
  boot_spm_busy_wait ();	// Wait until the memory is erased.
  uint8_t wbufaddr = 0;

  for (i = 0; i < SPM_PAGESIZE; i += 2)
    {
      // Set up little-endian word.
      uint16_t w = pageblock[wbufaddr++];
      w += pageblock[wbufaddr++] << 8;
      boot_page_fill (page + i, w);
    }

  boot_page_write (page);	// Store buffer in flash page.
  boot_spm_busy_wait ();	// Wait until the memory is written.

  // Reenable RWW-section again. We need this if we want to jump back
  // to the application after bootloading.

  boot_rww_enable ();

  // Re-enable interrupts (if they were ever enabled).
  SREG = sreg;
}

/* called when Data was received via USB*/
void
avrupdate_cmd (void)
{
    uint8_t buf[64];
    uint8_t size;
    uint8_t sreg = SREG;
    cli ();			// disable Interrupts

    size = USBNGetRxData(1, buf, 64);

  // check state 
  if (state == WRITEPAGE)
    {
      // if page sizte= 128 collect two 64 packages to a 128
      //UARTWrite("128\r\n");
      //SendHex(page_addr);
      if (page_addr % 2)
	{
	  collect128 = 0;
	  //UARTWrite("odd\r\n");
	  // get sescond package
	  if (page_addr == 1)
	    page_addr_w = 0;
	  else
	    page_addr_w = (page_addr - 1) / 2;

        memcpy(pageblock + 64, buf, size);
        if (size < 64)
          memset(pageblock + 64 + size, 0xff, 64 - size);

	  // write page
      if (page_addr_w + SPM_PAGESIZE / 2 <= 0x7000)
	    avrupdate_program_page (page_addr_w);
	  state = NONE;
	}
      else
	{
	  collect128 = 1;
	  //UARTWrite("even\r\n");
	  // get first package 
      memcpy(pageblock, buf, size);
      if (size < 64)
        memset(pageblock + size, 0xff, 64 - size);

	  state = NONE;
	}
    }
  else
    {
      state = buf[0];
      if (state == WRITEPAGE){
	page_addr  = buf[1];
	page_addr |= buf[2] << 8;
      }
      if (state == STARTAPP)
	{
	  if (collect128)
	    {
	      // if just half a page (64bytes) were received, write that half page now
	      page_addr = page_addr / 2;
	      avrupdate_program_page (page_addr);
	    }

	  // switch to run app mode
	  USBNWrite (RXC1, FLUSH);
	  USBNWrite (RXC1, RX_EN);

	  USBNWrite (MCNTRL, SRST);	// clear all usb registers
	  disable_all_interrupts(); // disable all interrupts

	  GICR = _BV (IVCE);	// enable wechsel der Interrupt Vectoren
	  GICR = 0x00;		// Interrupts auf Application Section umschalten
	  //sei ();
	  SREG = sreg;            // enable Interrupts again

	  // Reenable flash RWW-section again, where new application was written to
	  boot_rww_enable ();

	  avrupdate_jump_to_app ();
	}
      if (state == STOPPROGMODE)
	{
	  // section is not used (bootloader could be restarted)
	}
    }
    SREG = sreg;	// enable Interrupts again
}

/* main program */
int
main (void)
{
  disable_all_interrupts();
  // Initialize
  DDRA = 0x00;			// First all pins input
  DDRA |= (1 << PA4);		// then configure PIN with red LED as output

  DDRD = 0x00;			// Configure all pins as input
  DDRB = 0x00;
  DDRC = 0x00;

  PORTD |= (1 << PD1);		// TXD/PD1: Set internal pull-up
  PORTD |= (1 << PD0);		// RXD/PD0: Set internal pull-up
  wait_ms (1);			// wait for PD1 to be pulled up

  if (bit_is_clear (PIND, PD1))	// Check for wrong Jumper Setting; Jumper between TXD and GND
    goto wrong_jumper_setting;


  // Start bootloader?
  // First condition
  if (bit_is_set (PINA, PA4))	// update-mode request by update-tool -> LED is turned on
    goto start_update_mode;

  // or second condition
  if (eeprom_read_byte (&address) == 0x01)
    {				// update-mode request by update-tool -> byte in eeprom = 0x01
      eeprom_busy_wait ();
      eeprom_write_byte (&address, 0x00);	// reset byte in eeprom

      goto start_update_mode;
    }

  PORTD &= ~(1 << PD1);		// TXD/PD1 -> low
  DDRD |= (1 << PD1);		// TXD/PD1 -> output
  wait_ms (1);			// wait for PD0 to be pulled down by jumper or not

  // or third condition
  if (bit_is_clear (PIND, PD0))	// check if Jumper is set between RX and TX of UART-Connector
    goto start_update_mode;

  DDRD &= ~(1 << PD1);		// TXD/PD1 als Ausgang festlegen
  PORTD |= (1 << PD1);		// TXD/PD1: Set internal pull-up

  // or forth condition
  if (pgm_read_byte ((void *) 0) == 0xFF)	// check for first test after bootloader is installed
    goto start_update_mode;	// no programm in flash memory


  // Jump to Application section
  avrupdate_jump_to_app ();


  // bootloader application starts here
start_update_mode:

  collect128 = 0;		// state of received data: one page of flashmemory has 128Bytes
  // 64bytes have to be send in two steps via USB

    // Init USB [usbn2mc.h]
    USBNInit((struct usb_device_descriptor*)&avrupdateDevice, 
             (struct usb_configuration_descriptor_tab*)&avrupdateConfTab,
             (struct usb_wstring_descriptor_tab*)&StringTab);

    USBNAddOutEndpointCallback(1, avrupdate_cmd);

  cli ();			//reset "Global Interrupt enable" [interrupt.h]
  GICR = _BV (IVCE);		//IVCE = 1, necessary to change Interrupt Vector, disables Interrupts
  GICR = _BV (IVSEL);		//IVSEL = 1, place Interrupt Vector at beginning of boot loader section
  sei ();			//set "Global Interrupt enable" [interrupt.h]


  USBNInitMC ();
  USBNStart ();			// start usb chip


  // endless loop
  while (1)
    {
#if 1
      PORTA |= (1 << PA4);	// red LED: on-off-on-off--------
      wait_ms (100);
      PORTA &= ~(1 << PA4);
      wait_ms (100);
      PORTA |= (1 << PA4);
      wait_ms (100);
      PORTA &= ~(1 << PA4);
      wait_ms (800);
#endif
    }

wrong_jumper_setting:
  // endless loop
  while (1)
    {
      PORTA |= (1 << PA4);	// red LED: on-----off-
      wait_ms (500);
      PORTA &= ~(1 << PA4);
      wait_ms (100);
    }


}
