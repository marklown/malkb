/* MalKb
 * Copyright (c) 2015 Mark Lown
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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
#include "usb_keyboard.h"
#include "malkb.h"

// --------- DEFINES ----------------------------------------------------------

// DELETE THIS
#define LED_CONFIG	(DDRD |= (1<<6))
#define LED_ON		(PORTD &= ~(1<<6))
#define LED_OFF		(PORTD |= (1<<6))
// -------

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

#define NUM_ROWS 5
#define NUM_COLS 15
#define NUM_LAYERS 2

#define INIT_WAIT 1000
#define DEBOUNCE_WAIT 2

// ---- STRUCTURES -----------------------------------------------------------
typedef struct pin_t {
	volatile uint8_t* port;		// PORTx - set output or configure input
	volatile uint8_t* direction; 	// DDRx  - set as input or output
	volatile uint8_t* pin;			// PINx  - read the pin
	uint8_t x;					// index of this letter port
} myPin;

// ---- DEFINE LAYERS --------------------------------------------------------
static const uint8_t layers[NUM_LAYERS][NUM_ROWS][NUM_COLS] = {
    /* Layer 0 */
    {{KEY_TILDE,      KEY_1,   KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8,     KEY_9,      KEY_0,         KEY_MINUS,       KEY_EQUAL,        KEY_BACKSPACE, 0},
     {KEY_TAB,        KEY_Q,   KEY_W, KEY_E, KEY_R, KEY_T, KEY_Y, KEY_U, KEY_I,     KEY_O,      KEY_P,         KEY_LEFT_BRACE,  KEY_RIGHT_BRACE,  KEY_BACKSLASH, 0},
     {KEY_FN0,        KEY_A,   KEY_S, KEY_D, KEY_F, KEY_G, KEY_H, KEY_J, KEY_K,     KEY_L,      KEY_SEMICOLON, KEY_QUOTE,       KEY_ENTER,        KEY_NONE, 0},
     {KEY_LEFT_SHIFT, KEY_Z,   KEY_X, KEY_C, KEY_V, KEY_B, KEY_N, KEY_M, KEY_COMMA, KEY_PERIOD, KEY_SLASH,     KEY_RIGHT_SHIFT, KEY_NONE,         KEY_PAUSE, 0},
     {KEY_LEFT_CTRL,  KEY_GUI, KEY_LEFT_ALT, KEY_SPACE,KEY_NONE,KEY_NONE,KEY_NONE,KEY_NONE,KEY_NONE,KEY_NONE,  KEY_RIGHT_ALT,   KEY_FN0, KEY_FN1, KEY_FN2, 0}},
    
    /* Layer 1 */
    {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
     {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
     {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
     {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
     {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}}
};

// --------- LOCAL VARIABLES --------------------------------------------------

//! Current state of the keys
bool keys[NUM_ROWS][NUM_COLS];


// --------- KEYBOARD FUNCTIONS ----------------------------------------------
void kb_loop(void);
void kb_init(void);
void kb_read_matrix(void);
void do_key_up(int row, int col);
void do_key_dn(int row, int col);

// ---- FUNCTIONS TO INTERACT WITH TEENSY IO ---------------------------------
void teensy_configure_as_output(struct pin_t* row_or_col, int x);
void teensy_configure_as_input(struct pin_t* row_or_col, int x, bool pu);
void teensy_set_high(struct pin_t* row_or_col, int x);
void teensy_set_low(struct pin_t* row_or_col, int x);
bool teensy_read_input(struct pin_t* row_or_col, int x);

// ---- JUMP TO BOOTLOADER ---------------------------------------------------
void teensy_reboot(void);

// ---- DEFINE THE MAPPING FROM ROW TO PIN ON TEENSY -------------------------
struct pin_t row_to_pin[5] = {
		{&PORTC, &DDRC, &PINC, 0},
		{&PORTC, &DDRC, &PINC, 1},
		{&PORTC, &DDRC, &PINC, 2},
		{&PORTC, &DDRC, &PINC, 3},
		{&PORTC, &DDRC, &PINC, 4},
};

// ---- DEFINE THE MAPPING FROM COLUMN TO PIN ON TEENSY ----------------------
struct pin_t col_to_pin[15] = {
		{&PORTB, &DDRB, &PINB, 0},
		{&PORTB, &DDRB, &PINB, 1},
		{&PORTB, &DDRB, &PINB, 2},
		{&PORTB, &DDRB, &PINB, 3},
		{&PORTB, &DDRB, &PINB, 4},
		{&PORTB, &DDRB, &PINB, 5},
		{&PORTB, &DDRB, &PINB, 6},
		{&PORTB, &DDRB, &PINB, 7},
		{&PORTD, &DDRD, &PIND, 0},
		{&PORTD, &DDRD, &PIND, 1},
		{&PORTD, &DDRD, &PIND, 2},
		{&PORTD, &DDRD, &PIND, 3},
		{&PORTD, &DDRD, &PIND, 4},
		{&PORTD, &DDRD, &PIND, 6},
		{&PORTF, &DDRF, &PINF, 7},
};

#define ROW_0_PORT PORTC
#define ROW_1_PORT PORTC
/*
	ROW_0_PORT |= (1<<1);
*/

// ---- MAIN ENTRY POINT -----------------------------------------------------
int main(void)
{
    // Set for 16 MHz clock
    CPU_PRESCALE(0);
	
	// Setup the keyboard
    kb_init();
    
    // Initialize the USB, and then wait for the host to set configuration.
    // If the Teensy is powered without a PC connected to the USB port,
    // this will wait forever.
    usb_init();
    while (!usb_configured());
    
    // Wait an extra second for the PC's operating system to load drivers
    // and do whatever it does to actually be ready for input
    _delay_ms(INIT_WAIT);
    
    // Main loop
    kb_loop();
    
    return 0;
}

void kb_loop(void)
{
    int row, col;
    bool isPressed = false;
    bool wasPressed = false;
    
    // Main keyboard loop
    while (1) {
        
        // Read the key matrix
        kb_read_matrix();
        
        // Loop over each row and column in key matrix
        for (row = 0; row < NUM_ROWS; row++) {
            for (col = 0; col < NUM_COLS; col++) {
                
                wasPressed = isPressed;
                isPressed = keys[row][col];
                
                // Different state this tick
                if (isPressed != wasPressed) {
                    if (isPressed) {
                        
                        // Key is newly pressed this tick
                        do_key_dn(row, col);
                        
                    } else {
                        
                        // Key is newly released this tick
                        do_key_up(row, col);
                    }
                }
            }
        }
        
        // Send key command over usb (only 6KRO right now)
        usb_keyboard_send();
        
        // Wait a small amount of time to debounce
        _delay_ms(DEBOUNCE_WAIT);
    }

}

void kb_init(void)
{
	int row, col;
	
	// Set rows as outputs
	for (row = 0; row < NUM_ROWS; row++) {
		teensy_configure_as_output(row_to_pin, row);
		teensy_set_low(row_to_pin, row);
	}
	
	// Set columns as inputs with pullup resistors
	for (col = 0; col < NUM_COLS; col++) {
		teensy_configure_as_input(col_to_pin, col, true);
	}
}

void kb_read_matrix(void)
{
    int row, col;
    
    // Loop through all rows
    for (row = 0; row < NUM_ROWS; row++) {
       
        // Set row output to high
        teensy_set_high(row_to_pin, row);
        
        // Loop through all columns
        for (col = 0; col < NUM_COLS; col++) {
            
            // If input is high the key at row, col is pressed
            if (teensy_read_input(col_to_pin, col)) {
                keys[row][col] = true;
            } else {
                keys[row][col] = false;
            }
        }
        
        // Set row back to low
        teensy_set_low(row_to_pin, row);
    }
}

void teensy_configure_as_output(struct pin_t* row_or_col, int x)
{
	(*row_or_col[x].direction) |= (1<<row_or_col[x].x);
}

void teensy_configure_as_input(struct pin_t* row_or_col, int x, bool pu)
{
	(*row_or_col[x].direction) &= ~(1<<row_or_col[x].x);
	pu ? (*row_or_col[x].port |= (1<<row_or_col[x].x)) :
		 (*row_or_col[x].port &= ~(1<<row_or_col[x].x));
}

void teensy_set_high(struct pin_t* row_or_col, int x)
{
	(*row_or_col[x].port) |= (1<<row_or_col[x].x);
}

void teensy_set_low(struct pin_t* row_or_col, int x)
{
	(*row_or_col[x].port) &= ~(1<<row_or_col[x].x);
}

bool teensy_read_input(struct pin_t* row_or_col, int x)
{
	if ((*row_or_col[x].pin) & (1<<row_or_col[x].x)) {
		return true;
	} else {
		return false;
	}
}

void do_key_up(int row, int col)
{
    
}

void do_key_dn(int row, int col)
{
    
}

void teensy_reboot(void)
{
	cli();
	// disable watchdog, if enabled
	// disable all peripherals
	UDCON = 1;
	USBCON = (1<<FRZCLK);  // disable USB
	UCSR1B = 0;
	_delay_ms(5);
	#if defined(__AVR_AT90USB162__)                // Teensy 1.0
		EIMSK = 0; PCICR = 0; SPCR = 0; ACSR = 0; EECR = 0;
		TIMSK0 = 0; TIMSK1 = 0; UCSR1B = 0;
		DDRB = 0; DDRC = 0; DDRD = 0;
		PORTB = 0; PORTC = 0; PORTD = 0;
		asm volatile("jmp 0x3E00");
	#elif defined(__AVR_ATmega32U4__)              // Teensy 2.0
		EIMSK = 0; PCICR = 0; SPCR = 0; ACSR = 0; EECR = 0; ADCSRA = 0;
		TIMSK0 = 0; TIMSK1 = 0; TIMSK3 = 0; TIMSK4 = 0; UCSR1B = 0; TWCR = 0;
		DDRB = 0; DDRC = 0; DDRD = 0; DDRE = 0; DDRF = 0; TWCR = 0;
		PORTB = 0; PORTC = 0; PORTD = 0; PORTE = 0; PORTF = 0;
		asm volatile("jmp 0x7E00");
	#elif defined(__AVR_AT90USB646__)              // Teensy++ 1.0
		EIMSK = 0; PCICR = 0; SPCR = 0; ACSR = 0; EECR = 0; ADCSRA = 0;
		TIMSK0 = 0; TIMSK1 = 0; TIMSK2 = 0; TIMSK3 = 0; UCSR1B = 0; TWCR = 0;
		DDRA = 0; DDRB = 0; DDRC = 0; DDRD = 0; DDRE = 0; DDRF = 0;
		PORTA = 0; PORTB = 0; PORTC = 0; PORTD = 0; PORTE = 0; PORTF = 0;
		asm volatile("jmp 0xFC00");
	#elif defined(__AVR_AT90USB1286__)             // Teensy++ 2.0
		EIMSK = 0; PCICR = 0; SPCR = 0; ACSR = 0; EECR = 0; ADCSRA = 0;
		TIMSK0 = 0; TIMSK1 = 0; TIMSK2 = 0; TIMSK3 = 0; UCSR1B = 0; TWCR = 0;
		DDRA = 0; DDRB = 0; DDRC = 0; DDRD = 0; DDRE = 0; DDRF = 0;
		PORTA = 0; PORTB = 0; PORTC = 0; PORTD = 0; PORTE = 0; PORTF = 0;
		asm volatile("jmp 0x1FC00");
	#endif 
}
