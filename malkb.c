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
#include <string.h>
#include "malkb.h"

// --------- DEFINES ----------------------------------------------------------

// Onboard LED, useful for debugging
#define LED_CONFIG	(DDRD |= (1<<6))
#define LED_OFF		(PORTD &= ~(1<<6))
#define LED_ON		(PORTD |= (1<<6))

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

#define NUM_ROWS 5
#define NUM_COLS 15
#define NUM_LAYERS 2

#define INIT_WAIT_MS 		1000
#define DEBOUNCE_WAIT_MS 	2
#define PIN_WAIT_US			30

// ---- STRUCTURES -----------------------------------------------------------

// ---- DEFINE LAYERS --------------------------------------------------------
static const uint8_t layers[NUM_LAYERS][NUM_ROWS][NUM_COLS] = {
/*     0       1     2     3     4     5     6     7     8     9     10    11     12     13     14 */
    
     /* Layer 0 */
     {{ESC,    N1,   N2,   N3,   N4,   N5,   N6,   N7,   N8,   N9,   N0,   MIN,   EQL,   BSPC,  PAUS},
      {TAB,    Q,    W,    E,    R,    T,    Y,    U,    I,    O,    P,    LBRC,  RBRC,  BSLSH, NONE},
      {FN0,    A,    S,    D,    F,    G,    H,    J,    K,    L,    SCOL, QUOT,  ENTR,  NONE,  NONE},
      {LSHIFT, Z,    X,    C,    V,    B,    N,    M,    COMM, PRD,  SLSH, RSHIFT,NONE,  UP,    NONE},
      {LCTRL,  LGUI, LALT, SPC,  NONE, NONE, NONE, NONE, NONE, NONE, RALT, FN0,   LEFT,  DOWN,  RIGHT}},

     /* Layer 1 - FN0 */
     {{TLDE,   F1,   F2,   F3,   F4,   F5,   F6,   F7,   F8,   F9,   F10,  F11,   F12,   DEL,   0},
      {TAB,    0,    0,    0,    0,    0,    0,    PGUP, 0,    0,    0,    0,     0,     0,     0},
      {FN0,    0,    0,    PGDN, 0,    0,    LEFT, DOWN, UP,   RIGHT,0,    0,     0,     0,     0},
      {LSHIFT, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,     0,     0},
      {LCTRL,  LGUI, LALT, SPC,  NONE, NONE, NONE, NONE, NONE, NONE, RALT, FN0,   FN1,   FN2,   NONE}}
};

// --------- LOCAL VARIABLES --------------------------------------------------

//! Current state of the keys
bool keys[NUM_ROWS][NUM_COLS];

//! Last state of keys 
bool keys_last[NUM_ROWS][NUM_COLS];

//! The current layer we are on
uint8_t current_layer = 0;

// --------- KEYBOARD FUNCTION PROTOTYPES -------------------------------------
void kb_loop(void);
void kb_init(void);

void kb_read_matrix(void);
void kb_set_row_col(uint8_t row, uint8_t col, bool val);

void check_columns_for_row(uint8_t row);
void unselect_rows(void);
void select_row(uint8_t row);

void kb_resolve_fns(void);
void kb_resolve_modifiers(void);
void kb_resolve_keys(void);
bool kb_resolve_pns(void);

void kb_keys_insert(uint8_t key);
void kb_keys_remove(uint8_t key);

bool is_function(uint8_t key);
bool is_modifier(uint8_t key);
bool is_normal_key(uint8_t key);

// --------- TEENSY FUNCTION PROTOTYPES ---------------------------------------
void teensy_reboot(void);

// ---- MAIN ENTRY POINT -----------------------------------------------------
int main(void)
{
    LED_CONFIG;
    uint8_t i;
    for (i = 0; i < 5; i++) {
        LED_ON;
        _delay_ms(100);
        LED_OFF;
        _delay_ms(100);
    }
    
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
    _delay_ms(INIT_WAIT_MS);
    
    // Main loop
    kb_loop();
    
    return 0;
}

void all_off(void)
{
    uint8_t row, col;
    for (row = 0; row < NUM_ROWS; row++) {
        for (col = 0; col < NUM_COLS; col++) {
            keys[row][col] = false;
        }
    }
}

void kb_loop(void)
{
    while (1) {
        
		// Save the last key state
		memcpy(keys_last, keys, sizeof(keys_last) * NUM_ROWS * NUM_COLS);
        
        // Read the key matrix
        kb_read_matrix();
        
		// Resolve key presses. First check for custom actions, if none then
        // check functions, modifiers, and finally regular keys.
        if (kb_resolve_pns() == false) {
            kb_resolve_fns();
            kb_resolve_modifiers();
            kb_resolve_keys();
        }
        
        // Send key command over usb (only 6KRO right now)
        usb_keyboard_send();
        
        // Wait a small amount of time to debounce
        _delay_ms(DEBOUNCE_WAIT_MS);
    }
}

void kb_init(void)
{
	// Configure rows - outputs initially Hi-Z
    // row: 0   1   2   3   4
    // pin: D0  D1  D2  D3  D4
    unselect_rows();
	
    // Configure columns - inputs with internal pull-up resistors
    // col: 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14
    // pin: F0  F1  F4  C7  C6  B6  D5  B1  B0  B5  B4  D7  D5  B3  F5
	DDRB &= ~((1<<7) | (1<<6) | (1<<5) | (1<<4) | (1<<3) | (1<<1) | (1<<0));
    PORTB |= ((1<<7) | (1<<6) | (1<<5) | (1<<4) | (1<<3) | (1<<1) | (1<<0));
    DDRC &= ~((1<<7) | (1<<6));
    PORTC |= ((1<<7) | (1<<6));
	DDRD &= ~((1<<7) | (1<<5));
    PORTD |= ((1<<7) | (1<<5));
    DDRF &= ~((1<<5) | (1<<4) | (1<<1) | (1<<0));
    PORTF |= ((1<<5) | (1<<4) | (1<<1) | (1<<0));
	
	// Initialize key lists
    memset(&keyboard_modifier_keys, 0, sizeof(keyboard_modifier_keys));
    memset(&keyboard_keys, 0, sizeof(keyboard_keys));
	
	memset(keys, 0, sizeof(keys[0][0] * NUM_ROWS * NUM_COLS));
	memset(keys_last, 0, sizeof(keys[0][0] * NUM_ROWS * NUM_COLS));
}

void kb_read_matrix(void)
{
    uint8_t row;
	for (row = 0; row < NUM_ROWS; row++) {
        select_row(row);
        check_columns_for_row(row);
        unselect_rows();
    }
}

void check_columns_for_row(uint8_t row)
{
    // A switch is on when it's pin is also pulled to ground
    PINF & (1<<0) ? kb_set_row_col(row, 0, false) : kb_set_row_col(row, 0, true);
    PINF & (1<<1) ? kb_set_row_col(row, 1, false) : kb_set_row_col(row, 1, true);
    PINF & (1<<4) ? kb_set_row_col(row, 2, false) : kb_set_row_col(row, 2, true);
    PINC & (1<<7) ? kb_set_row_col(row, 3, false) : kb_set_row_col(row, 3, true);
    PINC & (1<<6) ? kb_set_row_col(row, 4, false) : kb_set_row_col(row, 4, true);
    PINB & (1<<6) ? kb_set_row_col(row, 5, false) : kb_set_row_col(row, 5, true);
    PIND & (1<<5) ? kb_set_row_col(row, 6, false) : kb_set_row_col(row, 6, true);
    PINB & (1<<1) ? kb_set_row_col(row, 7, false) : kb_set_row_col(row, 7, true);
    PINB & (1<<0) ? kb_set_row_col(row, 8, false) : kb_set_row_col(row, 8, true);
    PINB & (1<<5) ? kb_set_row_col(row, 9, false) : kb_set_row_col(row, 9, true);
    PINB & (1<<4) ? kb_set_row_col(row, 10, false) : kb_set_row_col(row, 10, true);
    PIND & (1<<7) ? kb_set_row_col(row, 11, false) : kb_set_row_col(row, 11, true);
    PIND & (1<<5) ? kb_set_row_col(row, 12, false) : kb_set_row_col(row, 12, true);
    PINB & (1<<3) ? kb_set_row_col(row, 13, false) : kb_set_row_col(row, 13, true);
    PINF & (1<<5) ? kb_set_row_col(row, 14, false) : kb_set_row_col(row, 14, true);
}

void unselect_rows(void)
{
    // Hi-Z (floating)
    DDRD  &= ~0b00011111;
    PORTD &= ~0b00011111;
    _delay_us(PIN_WAIT_US);
}

void select_row(uint8_t row)
{
    // Pull selected pin to ground
    switch (row) {
        case 0:
            DDRD  |=  (1<<0);
            PORTD &= ~(1<<0);
            break;
        case 1:
            DDRD  |=  (1<<1);
            PORTD &= ~(1<<1);
            break;
        case 2:
            DDRD  |=  (1<<2);
            PORTD &= ~(1<<2);
            break;
        case 3:
            DDRD  |=  (1<<3);
            PORTD &= ~(1<<3);
            break;
        case 4:
            DDRD  |=  (1<<4);
            PORTD &= ~(1<<4);
            break;
    }
    _delay_us(PIN_WAIT_US);
}


void kb_set_row_col(uint8_t row, uint8_t col, bool val)
{
    keys[row][col] = val;
}

// --------- RESOLVE KEY PRESSES ---------------------------------------

void kb_resolve_fns(void)
{
    uint8_t row, col;
    
    for (row = 0; row < NUM_ROWS; row++) {
        for (col = 0; col < NUM_COLS; col++) {
            
            uint8_t key = layers[0][row][col];
            uint8_t state = keys[row][col];
            uint8_t last_state = keys_last[row][col];
            
            if (is_function(key) && (state != last_state)) {
                
                if (state == true) {
                    current_layer = 1;
                } else {
                    current_layer = 0;
                }
                
            }
        }
    }
}

void kb_resolve_modifiers(void)
{
    uint8_t row, col;
    
    keyboard_modifier_keys = 0;
    
    for (row = 0; row < NUM_ROWS; row++) {
        for (col = 0; col < NUM_COLS; col++) {
            
            uint8_t key = layers[0][row][col];
            uint8_t state = keys[row][col];
            uint8_t last_state = keys_last[row][col];
            
            if (is_modifier(key) && (state != last_state)) {
                
                if (state == true) {
                    keyboard_modifier_keys |= TO_MOD(layers[current_layer][row][col]);
                }
            }
        }
    }
}

void kb_resolve_keys(void)
{
    uint8_t row, col;
    
    for (row = 0; row < NUM_ROWS; row++) {
        for (col = 0; col < NUM_COLS; col++) {
            
            uint8_t key = layers[current_layer][row][col];
            uint8_t state = keys[row][col];
            uint8_t last_state = keys_last[row][col];
            
            if (is_normal_key(key) && (state != last_state)) {
                
                if (state == true) {
                    kb_keys_insert(key);
                } else {
                    kb_keys_remove(key);
                }
            }
        }
    }
}

bool kb_resolve_pns(void)
{
    // hacky hardcoded, but i'm lazy to fix this.
    if (keys[5][0] == true &&
        keys[5][2] == true &&
        keys[0][14] == true) {
        teensy_reboot();
        return true;
    } else {
        return false;
    }
}

inline bool is_modifier(uint8_t key)
{
	if (key == LCTRL || key == LSHIFT || key == LALT || key == LGUI ||
        key == RCTRL || key == RSHIFT || key == RALT || key == RGUI) {
        return true;
    } else {
        return false;
    }
}

inline bool is_function(uint8_t key)
{
	return (((key == FN0) || (key == FN1) || (key == FN2)) ? true : false);
}

inline bool is_normal_key(uint8_t key)
{
    if ((is_modifier(key) == false) && (is_function(key) == false)) return true;
    else return false;
}

uint8_t free_index = 0;
void kb_keys_insert(uint8_t key)
{
    uint8_t i;
    
    if (free_index >= sizeof(keyboard_keys)) {
        return;
    }
    
    for (i = 0; i < sizeof(keyboard_keys); i++) {
        if (keyboard_keys[i] == 0) {
            free_index = i;
        }
    }
    
    keyboard_keys[free_index] = key;
}

void kb_keys_remove(uint8_t key)
{
    uint8_t i;
    for (i = 0; i < sizeof(keyboard_keys); i++) {
        if (keyboard_keys[i] == key) {
            keyboard_keys[i] = 0;
            free_index = i;
        }
    }
}

// -----------------------------------------------------------------

void teensy_reboot(void)
{
	cli();
	// Disable watchdog and all peripherals, disable usb
	UDCON = 1;
	USBCON = (1<<FRZCLK);
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
