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
#include "usb_keyboard.h"
#include "malkb.h"

// --------- DEFINES ----------------------------------------------------------

// Onboard LED, useful for debugging
#define LED_CONFIG	(DDRD |= (1<<6))
#define LED_ON		(PORTD &= ~(1<<6))
#define LED_OFF		(PORTD |= (1<<6))

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

#define NUM_ROWS 5
#define NUM_COLS 15
#define NUM_LAYERS 2

#define NUM_SIM_KEYS		6

#define INIT_WAIT_MS 		1000
#define DEBOUNCE_WAIT_MS 	2
#define PIN_WAIT_US			2

#define SET_LOW(port, pin) {port &= ~(1<<pin); _delay_us(PIN_WAIT_US);}
#define SET_HIGH(port, pin) {port |= (1<<pin); _delay_us(PIN_WAIT_US);}
#define CHECK_COL_FOR_ROW(port, pin, row, col) {port & (1<<pin) ? \
	kb_set_row_col(row, col, true) : kb_set_row_col(row, col, false);}

// ---- STRUCTURES -----------------------------------------------------------

typedef struct keylist_t 
{
	uint8_t key[NUM_SIM_KEYS];
	uint8_t count;
	uint8_t free_index;
} myKeyList;

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

//! Last state of keys 
bool keys_last[NUM_ROWS][NUM_COLS];

//! Lists of current key codes
struct keylist_t current_fns;
struct keylist_t current_modifiers;
struct keylist_t current_keys;

//! The current layer we are on
uint8_t current_layer = 0;

// --------- KEYBOARD FUNCTIONS ----------------------------------------------
void kb_loop(void);
void kb_init(void);
void kb_read_matrix(void);
void kb_set_row_col(int row, int col, bool val);
void kb_list_init(struct keylist_t* list);
void kb_list_insert(struct keylist_t* list, uint8_t key);
void kb_list_remove(struct keylist_t* list, uint8_t key);
void kb_resolve_fns(void);
void kb_resolve_modifiers(void);
void kb_resolve_keys(void);

// --------- TEENSY FUNCTIONS ------------------------------------------------
void teensy_reboot(void);

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
    _delay_ms(INIT_WAIT_MS);
    
    // Main loop
    kb_loop();
    
    return 0;
}

void kb_loop(void)
{   
    while (1) {
        
		// Save the last key state
		memcpy(keys_last, keys, sizeof(keys_last) * NUM_ROWS * NUM_COLS);
		
        // Read the key matrix
        kb_read_matrix();
		
		// Resolve key presses
		kb_resolve_fns();
		kb_resolve_modifiers();
		kb_resolve_keys();
        
        // Send key command over usb (only 6KRO right now)
        usb_keyboard_send();
        
        // Wait a small amount of time to debounce
        _delay_ms(DEBOUNCE_WAIT_MS);
    }
}

void kb_init(void)
{
	// Configure rows as outputs and set them low to start
	DDRC  |=  ((1<<4) | (1<<3) | (1<<2) | (1<<1) | (1<<0));
	PORTC &= ~((1<<4) | (1<<3) | (1<<2) | (1<<1) | (1<<0));
	
	// Configure columns as inputs 
	DDRB &= ~((1<<7) | (1<<6) | (1<<5) | (1<<4) | (1<<3) | (1<<2) | (1<<1) | (1<<0));
	DDRD &= ~((1<<6) | (1<<5) | (1<<4) | (1<<3) | (1<<2) | (1<<1) | (1<<0));
	
	// Configure the rest of the unused pins as outputs and set them low
	DDRC  |=  ((1<<7) | (1<<6) | (1<<5));
	PORTC &= ~((1<<7) | (1<<6) | (1<<5));
	DDRD  |=  ((1<<7));
	PORTD &= ~((1<<7));
	DDRF  |=  ((1<<7) | (1<<6) | (1<<5) | (1<<4) | (1<<3) | (1<<2) | (1<<1) | (1<<0));
	PORTF &= ~((1<<7) | (1<<6) | (1<<5) | (1<<4) | (1<<3) | (1<<2) | (1<<1) | (1<<0));
	
	// Initialize key lists
	kb_list_init(&current_fns);
	kb_list_init(&current_keys);
	kb_list_init(&current_modifiers);
	
	memset(keys, 0, sizeof(keys[0][0] * NUM_ROWS * NUM_COLS));
	memset(keys, 0, sizeof(keys[0][0] * NUM_ROWS * NUM_COLS));
}

void kb_set_row_col(int row, int col, bool val) 
{
	keys[row][col] = val;
}

void kb_read_matrix(void)
{
	// This is not elegant, but it is fast and reliable. We can loop through
	// the rows because they are all on the same port, and then check each 
	// columns against that row.
	
	int row, col;
	for (row = 0; row < NUM_ROWS; row++) {
		col = 0;
		SET_HIGH(PORTC, row);
		CHECK_COL_FOR_ROW(DDRB, 0, row, col++);
		CHECK_COL_FOR_ROW(DDRB, 1, row, col++);
		CHECK_COL_FOR_ROW(DDRB, 2, row, col++);
		CHECK_COL_FOR_ROW(DDRB, 3, row, col++);
		CHECK_COL_FOR_ROW(DDRB, 4, row, col++);
		CHECK_COL_FOR_ROW(DDRB, 5, row, col++);
		CHECK_COL_FOR_ROW(DDRB, 6, row, col++);
		CHECK_COL_FOR_ROW(DDRB, 7, row, col++);
		CHECK_COL_FOR_ROW(DDRD, 0, row, col++);
		CHECK_COL_FOR_ROW(DDRD, 1, row, col++);
		CHECK_COL_FOR_ROW(DDRD, 2, row, col++);
		CHECK_COL_FOR_ROW(DDRD, 3, row, col++);
		CHECK_COL_FOR_ROW(DDRD, 4, row, col++);
		CHECK_COL_FOR_ROW(DDRD, 5, row, col++);
		CHECK_COL_FOR_ROW(DDRD, 6, row, col++);
		SET_LOW(PORTC, row);
	}
}

/*
// modifiers OR'd together
extern uint8_t keyboard_modifier_keys;

// up to 6 keycodes
extern uint8_t keyboard_keys[6];
*/

bool IS_MODIFIER(uint8_t key)
{
	if (key == KEY_CTRL || key == KEY_SHIFT || key == KEY_ALT ||
		key == KEY_GUI || key == KEY_LEFT_CTRL || key == KEY_LEFT_ALT ||
		key == KEY_LEFT_GUI || key == KEY_RIGHT_CTRL || key == KEY_RIGHT_SHIFT ||
		key == KEY_RIGHT_ALT || key == KEY_RIGHT_GUI) {
		return true;
	} else {
		return false;	
	}
}

bool IS_FUNCTION(uint8_t key)
{
	return (((key == KEY_FN0) || (key == KEY_FN1) || (key == KEY_FN2)) ? true : false);
}

void kb_resolve_fns(void)
{
	uint8_t row, col;
	
	// Loop over each row and column in key matrix
	for (row = 0; row < NUM_ROWS; row++) {
		for (col = 0; col < NUM_COLS; col++) {
			if (IS_FUNCTION(layers[0][row][col])) {
				if (keys[row][col] != keys_last[row][col]) {
					current_layer = 1;
				} else {
					current_layer = 0;
				}
				return; // done after first function found
			} 
		}
	}
}

void kb_resolve_modifiers(void)
{
	uint8_t row, col;
	
	// Loop over each row and column in key matrix
	for (row = 0; row < NUM_ROWS; row++) {
		for (col = 0; col < NUM_COLS; col++) {
			if (IS_MODIFIER(layers[current_layer][row][col])) {
				if (keys[row][col] != keys_last[row][col]) {
					keyboard_modifier_keys |= layers[current_layer][row][col];
				} else {
					keyboard_modifier_keys &= ~(layers[current_layer][row][col]);
				}
			} 
		}
	}
}

void kb_resolve_keys(void)
{
	uint8_t row, col, i;
	
	// Loop over each row and column in key matrix
	for (row = 0; row < NUM_ROWS; row++) {
		for (col = 0; col < NUM_COLS; col++) {
			if (!IS_MODIFIER(layers[current_layer][row][col]) &&
				!IS_FUNCTION(layers[current_layer][row][col])) {
				if (keys[row][col] != keys_last[row][col]) {
					kb_list_insert(&current_keys, layers[current_layer][row][col]);
				} else {
					kb_list_remove(&current_keys, layers[current_layer][row][col]);
				}
			} 
		}
	}
	
	for (i = 0; i < current_keys.count && i < sizeof(keyboard_keys); i++) {
		keyboard_keys[i] = current_keys.key[i];
	}
}

void kb_list_insert(struct keylist_t* list, uint8_t key)
{
	if (list == 0) return;
	
	if (list->free_index >= sizeof(list->key)) {
		return;
	}
	
	list->key[list->free_index++] = key;
	list->count++;
}

void kb_list_remove(struct keylist_t* list, uint8_t key)
{
	if (list == 0) return;
	
	uint8_t i;
	for (i = 0; i < sizeof(list->key); i++) {
		if (list->key[i] == key) {
			list->key[i] = 0;
			list->free_index = i;
			list->count--;
		}
	}
}

void kb_list_init(struct keylist_t* list)
{
	if (list == 0) return;
	
	memset(list->key, 0, sizeof(list->key));
	list->count = 0;
	list->free_index = 0;
}

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
