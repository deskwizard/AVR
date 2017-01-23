/*
 * Project: 	 1key_x5
 * Author: 		 Charles Dorval (Deskwizard)
 *
 * General definitions for 1key_x5 example
 *
 * License: GNU GPL v3 (see License.txt)
*/
// USB states
#define STATE_WAIT 0
#define STATE_SEND_KEY 1
#define STATE_RELEASE_KEY 2

// One "active low" button connected between GND and PB4
#define KEY_PIN PINB
#define KEY_PORT PORTB 
#define KEY_DDR DDRB 
#define KEY0 4 

// Two LEDs connected between PB3/PB0 and GND 
#define LED_DDR DDRB 
#define LED_PORT PORTB 
#define LED0 3
#define LED1 0

