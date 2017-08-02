/* Teensy RawHID example
 * http://www.pjrc.com/teensy/rawhid.html
 * Copyright (c) 2009 PJRC.COM, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above description, website URL and copyright notice and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// Version 1.0: Initial Release
// Version 1.1: fixed bug in analog

#define USB_PRIVATE_INCLUDE
#include "usb_rawhid.h"

/**************************************************************************
 *
 *  Configurable Options
 *
 **************************************************************************/

// You can change these to give your code its own name.
#define STR_MANUFACTURER	L"Logitech, Inc."
#define STR_PRODUCT		L"WingMan Force 3D"

// These 4 numbers identify your device.  Set these to
// something that is (hopefully) not used by any others!
#define VENDOR_ID		0x046d		//CHANGED to Logitech Joystick value
#define PRODUCT_ID		0xc283  //CHANGED to Logitech Joystick value
#define RAWHID_USAGE_PAGE	0xFFAB	// recommended: 0xFF00 to 0xFFFF
#define RAWHID_USAGE		0x0200	// recommended: 0x0100 to 0xFFFF

// These determine the bandwidth that will be allocated
// for your communication.  You do not need to use it
// all, but allocating more than necessary means reserved
// bandwidth is no longer available to other USB devices.
#define RAWHID_TX_SIZE		7	// transmit packet size  //CHANGED to suit Joystick
#define RAWHID_TX_INTERVAL	10	// max # of ms between transmit packets //CHANGED to match joystick
//Receive is currently unused by the joystick, but may be used for force feedback
#define RAWHID_RX_SIZE		64	// receive packet size
#define RAWHID_RX_INTERVAL	8	// max # of ms between receive packets


/**************************************************************************
 *
 *  Endpoint Buffer Configuration
 *
 **************************************************************************/

#define ENDPOINT0_SIZE		32
#define RAWHID_INTERFACE	0
#define RAWHID_TX_ENDPOINT	1
#define RAWHID_RX_ENDPOINT	2

#if defined(__AVR_AT90USB162__)
#define RAWHID_TX_BUFFER	EP_SINGLE_BUFFER
#define RAWHID_RX_BUFFER	EP_SINGLE_BUFFER
#else
#define RAWHID_TX_BUFFER	EP_DOUBLE_BUFFER
#define RAWHID_RX_BUFFER	EP_DOUBLE_BUFFER
#endif

static const uint8_t PROGMEM endpoint_config_table[] = {
	1, EP_TYPE_INTERRUPT_IN,  EP_SIZE(RAWHID_TX_SIZE) | RAWHID_TX_BUFFER,
	1, EP_TYPE_INTERRUPT_OUT,  EP_SIZE(RAWHID_RX_SIZE) | RAWHID_RX_BUFFER,
	0,
	0
};


/**************************************************************************
 *
 *  Descriptor Data
 *
 **************************************************************************/

// Descriptors are the data that your computer reads when it auto-detects
// this USB device (called "enumeration" in USB lingo).  The most commonly
// changed items are editable at the top of this file.  Changing things
// in here should only be done by those who've read chapter 9 of the USB
// spec and relevant portions of any USB class specifications!


static const uint8_t PROGMEM device_descriptor[] = {
	18,					// bLength
	1,					// bDescriptorType
	0x00, 0x02,				// bcdUSB  		//Joystick uses USB 1.0, we'll stick with 2.0
	0,					// bDeviceClass
	0,					// bDeviceSubClass
	0,					// bDeviceProtocol
	ENDPOINT0_SIZE,				// bMaxPacketSize0
	LSB(VENDOR_ID), MSB(VENDOR_ID),		// idVendor   //CHANGED to joystick value
	LSB(PRODUCT_ID), MSB(PRODUCT_ID),	// idProduct  //CHANGED to joystick value
	0x06, 0x01,				// bcdDevice									//CHANGED to joystick value
	1,					// iManufacturer    //Joystick uses 4, we'll keep 1
	2,					// iProduct         //Joystick uses 32, we'll use 2
	0,					// iSerialNumber
	1						// bNumConfigurations
};

//Modified version which exactly matches the Report descriptor
static const uint8_t PROGMEM rawhid_hid_report_desc[] = {
																																							0x05, 0x01, 0x09, 0x04,
	0xa1, 0x01, 0xa1, 0x02, 0x09, 0x01, 0xa1, 0x00,			0x75, 0x08, 0x95, 0x02, 0x15, 0x00, 0x26, 0xff,
	0x00, 0x35, 0x00, 0x46, 0xff, 0x00, 0x09, 0x30, 		0x09, 0x31, 0x81, 0x02, 0x06, 0x00, 0xff, 0x09,
	0x01, 0x75, 0x04, 0x95, 0x01, 0x25, 0x0f, 0x81, 		0x02, 0x05, 0x01, 0x25, 0x07, 0x46, 0x3b, 0x01,
	0x65, 0x14, 0x09, 0x39, 0x81, 0x42, 0x75, 0x08,     0x26, 0xff, 0x00, 0x46, 0xff, 0x00, 0x09, 0x35,
	0x81, 0x02, 0xc0, 0x65, 0x00, 0x75, 0x01, 0x95,			0x07, 0x25, 0x01, 0x45, 0x01, 0x05, 0x09, 0x19,
	0x01, 0x29, 0x07, 0x81, 0x02, 0x95, 0x01, 0x81, 		0x01, 0x05, 0x01, 0x75, 0x08, 0x26, 0xff, 0x00,
	0x46, 0xff, 0x00, 0x09, 0x36, 0x81, 0x02, 0x06, 		0x00, 0xff, 0x09, 0x01, 0x95, 0x01, 0x81, 0x02,
	0xc0, 0xa1, 0x02, 0x06, 0x00, 0xff, 0x09, 0x02, 		0x95, 0x08, 0x91, 0x02, 0xc0, 0xc0
};

#define CONFIG1_DESC_SIZE        (9+9+9+7+7)
#define RAWHID_HID_DESC_OFFSET   (9+9)
static const uint8_t PROGMEM config1_descriptor[CONFIG1_DESC_SIZE] = {
	// configuration descriptor, USB spec 9.6.3, page 264-266, Table 9-10
	9, 					// bLength;
	2,					// bDescriptorType;
	LSB(CONFIG1_DESC_SIZE),			// wTotalLength
	MSB(CONFIG1_DESC_SIZE),
	1,					// bNumInterfaces
	1,					// bConfigurationValue
	0,					// iConfiguration
	0xC0,				// bmAttributes  //Joytick is 0x80 (not self powered, no remote
															 //wakeup, but leave as is for Teensy hadware
	50,					// bMaxPower		//Joystick is 40, but leave as is for Teensy hardware
	// interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
	9,					// bLength
	4,					// bDescriptorType
	RAWHID_INTERFACE,			// bInterfaceNumber
	0,					// bAlternateSetting
	2,					// bNumEndpoints
	0x03,					// bInterfaceClass (0x03 = HID)
	0x00,					// bInterfaceSubClass 				//NOTE joystick doesn't use sub class
	0x00,					// bInterfaceProtocol  				//NOTE joystick doesn't use sub protocol
	0,					// iInterface
	//HID Descriptor modified to exactly match joystick
	// HID interface descriptor, HID 1.11 spec, section 6.2.1
	9,					// bLength
	0x21,					// bDescriptorType
	0x00, 0x01,				// bcdHID
	0,					// bCountryCode
	1,					// bNumDescriptors
	0x22,					// bDescriptorType
	sizeof(rawhid_hid_report_desc),	0,	// wDescriptorLength
	// endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
	7,					// bLength
	5,					// bDescriptorType
	RAWHID_TX_ENDPOINT | 0x80,		// bEndpointAddress
		0x03,					// bmAttributes (0x03=intr)
	RAWHID_TX_SIZE, 0,			// wMaxPacketSize
	RAWHID_TX_INTERVAL,			// bInterval
	// endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
	7,					// bLength
	5,					// bDescriptorType
	RAWHID_RX_ENDPOINT,			// bEndpointAddress
	0x03,					// bmAttributes (0x03=intr)
	RAWHID_RX_SIZE, 0,			// wMaxPacketSize
	RAWHID_RX_INTERVAL			// bInterval
};

// If you're desperate for a little extra code memory, these strings
// can be completely removed if iManufacturer, iProduct, iSerialNumber
// in the device desciptor are changed to zeros.
struct usb_string_descriptor_struct {
	uint8_t bLength;
	uint8_t bDescriptorType;
	int16_t wString[];
};
static const struct usb_string_descriptor_struct PROGMEM string0 = {
	4,
	3,
	{0x0409}
};
static const struct usb_string_descriptor_struct PROGMEM string1 = {
	sizeof(STR_MANUFACTURER),
	3,
	STR_MANUFACTURER
};
static const struct usb_string_descriptor_struct PROGMEM string2 = {
	sizeof(STR_PRODUCT),
	3,
	STR_PRODUCT
};

// This table defines which descriptor data is sent for each specific
// request from the host (in wValue and wIndex).
static const struct descriptor_list_struct {
	uint16_t	wValue;
	uint16_t	wIndex;
	const uint8_t	*addr;
	uint8_t		length;
} PROGMEM descriptor_list[] = {
	{0x0100, 0x0000, device_descriptor, sizeof(device_descriptor)},
	{0x0200, 0x0000, config1_descriptor, sizeof(config1_descriptor)},
	{0x2200, RAWHID_INTERFACE, rawhid_hid_report_desc, sizeof(rawhid_hid_report_desc)},
	{0x2100, RAWHID_INTERFACE, config1_descriptor+RAWHID_HID_DESC_OFFSET, 9},
	{0x0300, 0x0000, (const uint8_t *)&string0, 4},
	{0x0301, 0x0409, (const uint8_t *)&string1, sizeof(STR_MANUFACTURER)},
	{0x0302, 0x0409, (const uint8_t *)&string2, sizeof(STR_PRODUCT)}
};
#define NUM_DESC_LIST (sizeof(descriptor_list)/sizeof(struct descriptor_list_struct))

/**************************************************************************
 *
 *  Variables - these are the only non-stack RAM usage
 *
 **************************************************************************/

// zero when we are not configured, non-zero when enumerated
static volatile uint8_t usb_configuration=0;

// these are a more reliable timeout than polling the
// frame counter (UDFNUML)
static volatile uint8_t rx_timeout_count=0;
static volatile uint8_t tx_timeout_count=0;



/**************************************************************************
 *
 *  Public Functions - these are the API intended for the user
 *
 **************************************************************************/

// initialize USB
void usb_init(void)
{
	HW_CONFIG();
	USB_FREEZE();				// enable USB
	PLL_CONFIG();				// config PLL
  while (!(PLLCSR & (1<<PLOCK))) ;	// wait for PLL lock
  USB_CONFIG();				// start USB clock
	UDCON = 0;				// enable attach resistor
	usb_configuration = 0;
  UDIEN = (1<<EORSTE)|(1<<SOFE);
	sei();
}

// return 0 if the USB is not configured, or the configuration
// number selected by the HOST
uint8_t usb_configured(void)
{
	return usb_configuration;
}

// receive a packet, with timeout
int8_t usb_rawhid_recv(uint8_t *buffer, uint8_t timeout)
{
	uint8_t intr_state;
	uint8_t offset;

	// if we're not online (enumerated and configured), error
	if (!usb_configuration) return -1;
	intr_state = SREG;
	cli();
	rx_timeout_count = timeout;
	UENUM = RAWHID_RX_ENDPOINT;
	// wait for data to be available in the FIFO
	while (1) {
		if (UEINTX & (1<<RWAL)) break;
		SREG = intr_state;
		if (rx_timeout_count == 0) return 0;
		if (!usb_configuration) return -1;
		intr_state = SREG;
		cli();
		UENUM = RAWHID_RX_ENDPOINT;
	}
	//read data from the FIFO
	for (offset=0; offset<RAWHID_RX_SIZE; offset++){
		*buffer++ = UEDATX;
	}
	// release the buffer
	UEINTX = 0x6B;
	SREG = intr_state;
	return RAWHID_RX_SIZE;
}

// send a packet, with timeout
int8_t usb_rawhid_send(const uint8_t *buffer, uint8_t timeout)
{
	uint8_t intr_state;
	uint8_t offset;
	//Make we are in a Configured device state
	if (!usb_configuration) return -1;
	// Save global interrupt state, then disable interrupts
	intr_state = SREG;
	cli();
	//Inialize timeout (counted in the ISR() interrupt handler)
	tx_timeout_count = timeout;
	//===Wait for the FIFO to be ready to accept data===:
	while (1) {
		//Select the endpoint, check if the buffer is ready, and break if it is
		UENUM = RAWHID_TX_ENDPOINT;
		if (UEINTX & (1<<RWAL))
			break;
		//Restore interrupts
		SREG = intr_state;
		//Exit if we reach the timeout or if the device becomes unconfigured
		if (tx_timeout_count == 0) return 0;
		if (!usb_configuration) return -1;
		// Save and clear interrupts
		intr_state = SREG;
		cli();
	}
	// write bytes to the FIFO buffer
	for (offset=0; offset<RAWHID_TX_SIZE; offset++){
			UEDATX = *buffer++;
	}
	// Transmit the buffer
	UENUM = RAWHID_TX_ENDPOINT;
	UEINTX = 0x3A;
	// Restore interrupts
	SREG = intr_state;
	return RAWHID_TX_SIZE;
}

/**************************************************************************
 *
 *  Private Functions - not intended for general user consumption....
 *
 **************************************************************************/

#if (GCC_VERSION >= 40300) && (GCC_VERSION < 40302)
#error "Buggy GCC 4.3.0 compiler, please upgrade!"
#endif

// USB Device Interrupt - handle all device-level events
// the transmit buffer flushing is triggered by the start of frame
//
ISR(USB_GEN_vect)
{
	uint8_t intbits, t;

  intbits = UDINT;
  UDINT = 0;
  if (intbits & (1<<EORSTI)) {
		UENUM = 0;
		UECONX = 1;
		UECFG0X = EP_TYPE_CONTROL;
		UECFG1X = EP_SIZE(ENDPOINT0_SIZE) | EP_SINGLE_BUFFER;
		UEIENX = (1<<RXSTPE);
		usb_configuration = 0;
  }
	if ((intbits & (1<<SOFI)) && usb_configuration) {
		t = rx_timeout_count;
		if (t) rx_timeout_count = --t;
		t = tx_timeout_count;
		if (t) tx_timeout_count = --t;
	}
}

// Misc functions to wait for ready and send/receive packets
static inline void usb_wait_in_ready(void)
{
	while (!(UEINTX & (1<<TXINI))) ;
}
static inline void usb_send_in(void)
{
	UEINTX = ~(1<<TXINI);
}
static inline void usb_wait_receive_out(void)
{
	while (!(UEINTX & (1<<RXOUTI))) ;
}
static inline void usb_ack_out(void)
{
	UEINTX = ~(1<<RXOUTI);
}

// USB Endpoint Interrupt - endpoint 0 is handled here.  The
// other endpoints are manipulated by the user-callable
// functions, and the start-of-frame interrupt.
//
ISR(USB_COM_vect)
{
  uint8_t intbits;
	const uint8_t *list;
  const uint8_t *cfg;
	uint8_t i, n, len, en;
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
	uint16_t desc_val;
	const uint8_t *desc_addr;
	uint8_t	desc_length;

  UENUM = 0;
	intbits = UEINTX;
  if (intbits & (1<<RXSTPI)) {
    bmRequestType = UEDATX;
    bRequest = UEDATX;
    wValue = UEDATX;
    wValue |= (UEDATX << 8);
    wIndex = UEDATX;
    wIndex |= (UEDATX << 8);
    wLength = UEDATX;
    wLength |= (UEDATX << 8);
    UEINTX = ~((1<<RXSTPI) | (1<<RXOUTI) | (1<<TXINI));
  	if (bRequest == GET_DESCRIPTOR) {
			list = (const uint8_t *)descriptor_list;
			for (i=0; ; i++) {
				if (i >= NUM_DESC_LIST) {
					UECONX = (1<<STALLRQ)|(1<<EPEN);  //stall
					return;
				}
				desc_val = pgm_read_word(list);
				if (desc_val != wValue) {
					list += sizeof(struct descriptor_list_struct);
					continue;
				}
				list += 2;
				desc_val = pgm_read_word(list);
				if (desc_val != wIndex) {
					list += sizeof(struct descriptor_list_struct)-2;
					continue;
				}
				list += 2;
				desc_addr = (const uint8_t *)pgm_read_word(list);
				list += 2;
				desc_length = pgm_read_byte(list);
				break;
			}
			len = (wLength < 256) ? wLength : 255;
			if (len > desc_length) len = desc_length;
			do {
				// wait for host ready for IN packet
				do {
					i = UEINTX;
				} while (!(i & ((1<<TXINI)|(1<<RXOUTI))));
				if (i & (1<<RXOUTI)) return;	// abort
				// send IN packet
				n = len < ENDPOINT0_SIZE ? len : ENDPOINT0_SIZE;
				for (i = n; i; i--) {
					UEDATX = pgm_read_byte(desc_addr++);
				}
				len -= n;
				usb_send_in();
			} while (len || n == ENDPOINT0_SIZE);
			return;
    }
		if (bRequest == SET_ADDRESS) {
			usb_send_in();
			usb_wait_in_ready();
			UDADDR = wValue | (1<<ADDEN);
			return;
		}
		if (bRequest == SET_CONFIGURATION && bmRequestType == 0) {
			usb_configuration = wValue;
			usb_send_in();
			cfg = endpoint_config_table;
			for (i=1; i<5; i++) {
				UENUM = i;
				en = pgm_read_byte(cfg++);
				UECONX = en;
				if (en) {
					UECFG0X = pgm_read_byte(cfg++);
					UECFG1X = pgm_read_byte(cfg++);
				}
			}
        		UERST = 0x1E;
        		UERST = 0;
			return;
		}
		if (bRequest == GET_CONFIGURATION && bmRequestType == 0x80) {
			usb_wait_in_ready();
			UEDATX = usb_configuration;
			usb_send_in();
			return;
		}
		if (bRequest == GET_STATUS) {
			usb_wait_in_ready();
			i = 0;
			if (bmRequestType == 0x82) {
				UENUM = wIndex;
				if (UECONX & (1<<STALLRQ)) i = 1;
				UENUM = 0;
			}
			UEDATX = i;
			UEDATX = 0;
			usb_send_in();
			return;
		}
		if ((bRequest == CLEAR_FEATURE || bRequest == SET_FEATURE)
		  && bmRequestType == 0x02 && wValue == 0) {
			i = wIndex & 0x7F;
			if (i >= 1 && i <= MAX_ENDPOINT) {
				usb_send_in();
				UENUM = i;
				if (bRequest == SET_FEATURE) {
					UECONX = (1<<STALLRQ)|(1<<EPEN);
				} else {
					UECONX = (1<<STALLRQC)|(1<<RSTDT)|(1<<EPEN);
					UERST = (1 << i);
					UERST = 0;
				}
				return;
			}
		}
		if (wIndex == RAWHID_INTERFACE) {
			if (bmRequestType == 0xA1 && bRequest == HID_GET_REPORT) {
				len = RAWHID_TX_SIZE;
				do {
					// wait for host ready for IN packet
					do {
						i = UEINTX;
					} while (!(i & ((1<<TXINI)|(1<<RXOUTI))));
					if (i & (1<<RXOUTI)) return;	// abort
					// send IN packet
					n = len < ENDPOINT0_SIZE ? len : ENDPOINT0_SIZE;
					for (i = n; i; i--) {
						// just send zeros
						UEDATX = 0;
					}
					len -= n;
					usb_send_in();
				} while (len || n == ENDPOINT0_SIZE);
				return;
			}
			if (bmRequestType == 0x21 && bRequest == HID_SET_REPORT) {
				len = RAWHID_RX_SIZE;
				do {
					n = len < ENDPOINT0_SIZE ? len : ENDPOINT0_SIZE;
					usb_wait_receive_out();
					// ignore incoming bytes
					usb_ack_out();
					len -= n;
				} while (len);
				usb_wait_in_ready();
				usb_send_in();
				return;
			}
		}
	}
	UECONX = (1<<STALLRQ) | (1<<EPEN);	// stall
}
