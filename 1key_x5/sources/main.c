/*
 * Project: 	1key_x5
 * Author: 	Charles Dorval (Deskwizard)
 * 
 * 	V-USB HID 1 key keyboard example with status LED
 *
 *	Demonstrate a 1 key USB HID keyboard with the use of V-USB 
 *	on an ATTINY45/85 using the internal oscillator tuned at 16.5mHz
 * 
 *
 *	A button press toggles between sending 'W' (Left shift + w) and idle states
 *	
 *
 *	USB states, LEDs and button GPIOs are configured in defines.h
 *
 *	Default GPIOs assignment are as follows:
 *		LED0 	PB3
 *		LED1	PB0
 *		KEY0	PB4
 *
 *		LED0 follows the USB states, on while sending keys and off when idle.
 *		LED1 is the Status LED and is configured as NUM_LOCK LED by default.
 *		KEY0 is a momentary push button in an active low configuration.
 *
 *
 *	Available keycodes, modifiers and status LEDs definitions are located in keycodes.h
 *	
 *	Please note that not all keycodes are defined, see link in keycodes.h
 *	for complete USB HID keycodes table.	 
 *
 *
 *	Based on:	 V-USB example code by Christian Starkjohann
 * 				 AVR ATtiny USB Tutorial by Joonas Pihlajamaa
 *				 V-USB HIDkeys reference implementation
 *				 V-USB EasyLogger reference implementation
 *				 Flip's 1 key keyboard example
 *				 USB descriptors from Frank Zhao's USB Business Card project
 *				 Various other V-USB examples
 *				 Danni's Debounce by Peter Dannegger
 *
 *
 *	In sources/ directory:
 *  To compile					make
 *  To compile and flash		make flash
 *  To clean and compile		make clean
 *
 *	To flash fuses:
 *  avrdude -c usbasp -p attiny45 -v -U lfuse:w:0xfe:m -U hfuse:w:0xd5:m -U efuse:w:0xff:m
 *
 *  Compiler:	gcc 5.4.0 20160609 (Ubuntu 5.4.0-6ubuntu1~16.04.4)
 * 
 *  Please note that the provided main file is configured to use the "usbasp" programmer
 *  by default. Edit the makefile accordingly if using another programmer.
 *
 *  Compiled HEX file is also provided for convenience.
 *
 *  Licensed under GNU GPL v3 (see License.txt)
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#include "usbdrv.h"
#include "defines.h"
#include "keycodes.h"


// USB descriptor
const PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)(Key Codes)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)(224)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)(231)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs) ; Modifier byte
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs) ; Reserved byte
    0x95, 0x05,                    //   REPORT_COUNT (5)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x05, 0x08,                    //   USAGE_PAGE (LEDs)
    0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,                    //   USAGE_MAXIMUM (Kana)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs) ; LED report
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x03,                    //   REPORT_SIZE (3)
    0x91, 0x03,                    //   OUTPUT (Cnst,Var,Abs) ; LED report padding
    0x95, 0x06,                    //   REPORT_COUNT (6)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)(Key Codes)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))(0)
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)(101)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};

// USB and keyboard variables
typedef struct {
	uint8_t modifier;
	uint8_t reserved;
	uint8_t keycode[6];
} keyboard_report_t;

static keyboard_report_t keyboard_report; 	// Sent to PC
volatile static uchar LED_state = 0xff; 	// Received from PC
static uchar idleRate;						// Repeat rate for keyboards

// Debouncing variables
unsigned char key_state;					// Debounced and inverted key state: bit = 1: key pressed
volatile unsigned char key_press;			// Key press detect
unsigned char ct0 = 0xFF, ct1 = 0xFF;		// Internal debouncing states

// States variables
uint8_t run_state = 0;						// Run state
uchar state = STATE_WAIT;					// USB state


// *********** USB HID ROUTINES ***********

usbMsgLen_t usbFunctionSetup(uchar data[8]) {
    usbRequest_t *rq = (void *)data;

    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) {
        switch(rq->bRequest) {
        case USBRQ_HID_GET_REPORT: 					// send "no keys pressed" if asked here
            										// wValue: ReportType (highbyte), ReportID (lowbyte)
            usbMsgPtr = (void *)&keyboard_report; 	// we only have this one
            keyboard_report.modifier = 0;
            keyboard_report.keycode[0] = 0;
            return sizeof(keyboard_report);
		case USBRQ_HID_SET_REPORT: 					// if wLength == 1, should be LED state
            return (rq->wLength.word == 1) ? USB_NO_MSG : 0;
        case USBRQ_HID_GET_IDLE: 					// send idle rate to PC as required by spec
            usbMsgPtr = &idleRate;
            return 1;
        case USBRQ_HID_SET_IDLE: 					// save idle rate as required by spec
            idleRate = rq->wValue.bytes[1];
            return 0;
        }
    }
    
    return 0; // by default don't return any data
}


usbMsgLen_t usbFunctionWrite(uint8_t * data, uchar len) {
	if (data[0] == LED_state)
        return 1;
    else
        LED_state = data[0];
	
    // LED state changed
	if(LED_state & NUM_LOCK)
		LED_PORT |= (1 << LED1); 	// Set status LED on
	else
		LED_PORT &= ~(1 << LED1); 	// Set status LED off
	
	return 1; // Data read, not expecting more
}


void buildReport(uchar modifier, uchar send_key) {

	keyboard_report.modifier = modifier;
	
	if(send_key != 0)
		keyboard_report.keycode[0] = send_key;
	else
		keyboard_report.keycode[0] = 0;
}


// *********** Debouncing ***********

ISR(TIMER0_OVF_vect ){					
	unsigned char i;

	i = key_state ^ ~KEY_PIN; 			// key changed ?
	ct0 = ~(ct0 & i); 					// reset or count ct0
	ct1 = ct0 ^ (ct1 & i); 				// reset or count ct1
	i &= ct0 & ct1;						// count until roll over ?
	key_state ^= i;						// then toggle debounced state
	key_press |= key_state & i; 		// 0 > 1: key press detect
}


unsigned char get_key_press( unsigned char key_mask ){
	cli();
	key_mask &= key_press;				// read key(s)
	key_press ^= key_mask;				// clear key(s)
	sei();
	return key_mask; 
}


void debounce(void){

	if( get_key_press( 1 << KEY0 ))	{		// Toggle LED0 and start sending keys on button press
	
			if (run_state) {
				run_state = 0;
				LED_PORT &= ~(1 << LED0);
				state = STATE_RELEASE_KEY; 	// Release the keys
			}
			else {
				run_state = 1;
				LED_PORT |= (1 << LED0);
				state = STATE_SEND_KEY;		// Start sending the keys
			}
	} // Key0

} // debounce


// *********** Main code ***********

int main() {

	uchar i;
	
	LED_DDR |= (1 << LED0); 	// Set as output
	LED_DDR |= (1 << LED1);		// Set as output
	KEY_PORT |= (1 << KEY0); 	// Set as input with internal pullup resistor activated

	// Initially clear the USB report
    for(i=0; i<sizeof(keyboard_report); i++) {		
        ((uchar *)&keyboard_report)[i] = 0;
    }

    wdt_enable(WDTO_1S); 		// Enable 1s watchdog timer

    usbInit();
	
    usbDeviceDisconnect(); 		// Enforce re-enumeration
    for(i = 0; i<250; i++) { 	// Wait 500 ms
        wdt_reset(); 			// Keep the watchdog happy
        _delay_ms(2);
    }

    usbDeviceConnect();			// Self-explanatory
	
	// Debouncing timer (timer0) configuration
	TCCR0B = (1 << CS01);		// Divide by 256 * 256
	TIMSK = (1 << TOIE0);		// Enable timer interrupt 
	key_state = ~KEY_PIN;		// No action on keypress during reset 
    
    sei(); 						// Enable interrupts after re-enumeration
	
	// Main loop
    while(1) {
        wdt_reset(); 			// Keep the watchdog happy
        usbPoll();

		debounce(); 			// React to debounced button

        // Characters are sent when messageState == STATE_SEND and after receiving
        // the initial LED state from PC (good way to wait until device is recognized)
        if(usbInterruptIsReady() && state != STATE_WAIT && LED_state != 0xff){
			switch(state) {
			case STATE_SEND_KEY:
				buildReport(MOD_SHIFT_LEFT, KEY_W);
				break;
			case STATE_RELEASE_KEY:
				buildReport(NULL, NULL);
				state = STATE_WAIT; 		// Go back to waiting
				break;
			default:
				state = STATE_WAIT; 		// Should not happen
			}
			
			// start sending
            usbSetInterrupt((void *)&keyboard_report, sizeof(keyboard_report));
        }
    }
	
    return 0;
}
