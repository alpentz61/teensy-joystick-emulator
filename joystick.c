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

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "usb_rawhid.h"
#include "analog.h"

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

volatile uint8_t do_output=0;
uint8_t buffer[7];
uint8_t buffer_last[sizeof(buffer)];

int main(void)
{
	int8_t r;
	uint8_t i;
	uint16_t val, count=0;
	uint8_t offset;
	uint8_t buffVal;
	uint8_t buffChanged = 0;

	//Analog input map
	#define ROLL 0
	#define ELEVATOR 1
	#define RUDDER 3
	#define THROTTLE 5
	//Map of the fields to which each anlog value should be written
	//For convenience, these will be read from the corresponding Port F pins
	const uint8_t analogMap[] = {
		ROLL,
		ELEVATOR,
		RUDDER,
		THROTTLE
	};
	const int analogCount = sizeof(analogMap);
  uint8_t analogValues[analogCount];

	//Digital Inputs:
	uint8_t view_switch; //TODO implement view switch
	uint8_t buttons;

	// set for 16 MHz clock
	CPU_PRESCALE(0);

	// Initialize the USB, and then wait for the host to set configuration.
	// If the Teensy is powered without a PC connected to the USB port,
	// this will wait forever.
	usb_init();
	while (!usb_configured()) /* wait */ ;

	// Wait an extra second for the PC's operating system to load drivers
	// and do whatever it does to actually be ready for input
	_delay_ms(1000);

  // Configure timer 0 to generate a timer overflow interrupt every
  // 256*1024 clock cycles, or approx 61 Hz when using 16 MHz clock
  TCCR0A = 0x00;
  TCCR0B = 0x05;
  TIMSK0 = (1<<TOIE0);

	//Initialize buffers to 0
	for (i=0; i<sizeof(buffer); i++) {
		buffer[i]=0;
		buffer_last[i]=0;
	}
	//Dummy value which isn't excercized by normal functionality (maybe status byte?)
	buffer[6] = 3;

	while (1) {
		// if time to send output, transmit something interesting
		if (do_output) {
			do_output = 0;

			//Read button inputs from PORTD
			DDRD = 0x00;
			PORTD = 0xFF;
			buttons = ~PIND & 0x7f; //1)Active low, so invert, 2) mask out bit 7
			buffer[4] = buttons;

			//Read analog values									//Analog Map (location in the packet)
			for (i=0; i<analogCount; i++) {			  //  i : offset
			 	offset = analogMap[i]; 							//  0 -> 1
				val = analogRead(offset);						//  1 -> 1
				buffer[offset] = val >> 2;          //  2 -> 3
			}																			//  3 -> 5

			//Detect changes in buffer values,
			//then update the buffer's last values
			buffChanged = 0;
			for (i=0; i<sizeof(buffer); i++) {
				if (buffer[i] != buffer_last[i]){
					buffChanged = 1;
				}
				buffer_last[i] = buffer[i];
			}

			//if a change in the buffer value occured, send the HID packet
			if (buffChanged) {
				usb_rawhid_send(buffer, 50);
			}
		} //End do_output
	} //End while(1)
}

// This interrupt routine is run approx 61 times per second.
ISR(TIMER0_OVF_vect)
{
	// set the do_output varirable to trigger every interrupt
	do_output = 1;
}
