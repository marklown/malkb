/* malkb
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


#define LED_CONFIG			(DDRD |= (1<<6))
#define LED_OFF				(PORTD &= ~(1<<6))
#define LED_ON				(PORTD |= (1<<6))
#define CPU_PRESCALE(n)		(CLKPR = 0x80, CLKPR = (n))
#define NUM_ROWS 			5
#define NUM_COLS 			15
#define NUM_LAYERS 			2
#define INIT_WAIT_MS 		1000
#define LOOP_WAIT_MS 		1
#define IO_WAIT_US			30
#define DEBOUNCE			100

typedef struct Debounce {
	uint16_t On;
	uint16_t Off;
} _debounce;
// ctrl fn win alt    alt win menu ctrl (das layout)
typedef void (*MacroHandler)(void);

typedef struct KeyMacro {
	uint8_t Keys[6][6];		// row, col of up to 6 keys
	MacroHandler Handler;	// macro handler function pointer
} _macro;

static const uint8_t layers[NUM_LAYERS][NUM_ROWS][NUM_COLS] = {
/*     0       1     2     3     4     5     6     7     8     9     10    11     12     13     14 */
    
     /* Layer 0 */
     {{TLDE,   N1,   N2,   N3,   N4,   N5,   N6,   N7,   N8,   N9,   N0,   MIN,   EQL,   0,     BSPC},
      {TAB,    Q,    W,    E,    R,    T,    Y,    U,    I,    O,    P,    LBRC,  RBRC,  BSLSH, DEL},
      {FN0,    A,    S,    D,    F,    G,    H,    J,    K,    L,    SCOL, QUOT,  0,     ENTR,  ESC},
      {LSHIFT, Z,    X,    C,    V,    B,    N,    M,    COMM, PRD,  SLSH, 0,     RSHIFT,UP,    NONE},
      {LCTRL,  0,    LGUI, LALT,    0,    0,    0,    SPC,  0,    0,    RALT, FN0,   LEFT,  DOWN,  RIGHT}},

     /* Layer 1 - FN0 */
     {{ESC,    F1,   F2,   F3,   F4,   F5,   F6,   F7,   F8,   F9,   F10,  F11,   F12,   0,     DEL},
      {TAB,    NONE, NONE, NONE, NONE, NONE, NONE, PGUP, NONE, UP,   NONE, NONE,  NONE,  NONE,  VOLUP},
      {FN0,    NONE, NONE, PGDN, NONE, NONE, NONE, NONE, LEFT, DOWN, RIGHT,NONE,  0,     NONE,  VOLDN},
      {LSHIFT, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, 0,     NONE,  NONE,  MUTE},
      {LCTRL,  0,    LGUI, LALT,    0,    0,    0,    SPC,  0,    0,    RALT, FN0,   NONE,  NONE,  NONE}}
};

// ------ VARIABLES -------------------------------------------------------------------------------

bool currKeys[NUM_ROWS][NUM_COLS];						// Current matrix state
bool lastKeys[NUM_ROWS][NUM_COLS];						// Last matrix state
uint8_t currLayer = 0;									// Current active layer
struct Debounce debounce[NUM_ROWS][NUM_COLS];			// Current debounce state
struct KeyMacro macros[32];								// Array of custom macros
uint8_t macroIndex = 0;

// ------ PROTOTYPES ------------------------------------------------------------------------------

void KeyboardLoop(void);								// Main loop
void KeyboardInit(void);								// Init keyboard
void ReadMatrix(void);									// Reads the keyboard matrix via teensy
void SetRowAndCol(uint8_t row, uint8_t col, bool val);	// Sets the current matrix row, col state
void CheckColForRow(uint8_t row);						// Check a columns for a given row
void UnselectRows(void);								// Unselect all rows
void SelectRow(uint8_t row);							// Select one row
void ResolveFunctionKeys(void);							// Resolve function key presses
void ResolveModifierKeys(void);							// Resolve modifier key presses
void ResolveNormalKeys(void);							// Resolve normal key presses
void ResolveMacros(void);								// Handle custom key combos (macros)
void InsertKey(uint8_t key);							// Insert a pressed key into the list
void RemoveKey(uint8_t key);							// Remove a release key from the list
bool IsFunctionKey(uint8_t key);						// Check if this is a function key
bool IsModifierKey(uint8_t key);						// Check if this is a modifier key
bool IsNormalKey(uint8_t key);							// Check if this is a normal key
void Reboot(void);										// Jump to the bootloader
void AddMacro(uint8_t k1, uint8_t k2, uint8_t k3,		// Add a macro
			  uint8_t k4, uint8_t k5, uint8_t k6,
			  MacroHandler handler);
			  
// Macro handlers
void RebootMacro(void) {}


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
    KeyboardInit();
    
    // Initialize the USB, and then wait for the host to set configuration.
    // If the Teensy is powered without a PC connected to the USB port,
    // this will wait forever.
    usb_init();
    while (!usb_configured());
    _delay_ms(INIT_WAIT_MS);
    
    // Main loop
    KeyboardLoop();
    
    return 0;
}

void KeyboardLoop(void)
{
    while (1) {
        
		// Save the last key state
		memcpy(lastKeys, currKeys, sizeof(lastKeys) * NUM_ROWS * NUM_COLS);
        
        // Read the key matrix
		ReadMatrix();
		
        ResolveFunctionKeys();
        ResolveModifierKeys();
        ResolveNormalKeys();
		
		ResolveMacros();
        
        // Send key command over usb (only 6KRO right now)
        usb_keyboard_send();
        
        // Wait a small amount of time
        _delay_ms(LOOP_WAIT_MS);
    }
}

void KeyboardInit(void)
{
	// Configure rows - outputs initially Hi-Z
    // row: 0   1   2   3   4
    // pin: B4  D7  D4  D5  C7
    UnselectRows();
	
    // Configure columns - inputs with internal pull-up resistors
    // col: 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14
    // pin: C6  D3  B5  D2  D1  D0  B7  B3  B2  B1  B0  F5  F4  F0  F7
	DDRB &= ~((1<<7) | (1<<5) | (1<<3) | (1<<2) | (1<<1) | (1<<0));
    PORTB |= ((1<<7) | (1<<5) | (1<<3) | (1<<2) | (1<<1) | (1<<0));
    DDRC &= ~((1<<6));
    PORTC |= ((1<<6));
	DDRD &= ~((1<<3) | (1<<2) | (1<<1) | (1<<0));
    PORTD |= ((1<<3) | (1<<2) | (1<<1) | (1<<0));
    DDRF &= ~((1<<7) | (1<<5) | (1<<4) | (1<<0));
    PORTF |= ((1<<7) | (1<<5) | (1<<4) | (1<<0));
	
    memset(&keyboard_modifier_keys, 0, sizeof(keyboard_modifier_keys));
    memset(&keyboard_keys, 0, sizeof(keyboard_keys));

	memset(currKeys, 0, sizeof(currKeys[0][0]) * NUM_ROWS * NUM_COLS);
	memset(lastKeys, 0, sizeof(lastKeys[0][0]) * NUM_ROWS * NUM_COLS);
	memset(debounce, 0, sizeof(debounce[0][0]) * NUM_ROWS * NUM_COLS);
	
	// Add some macros
	AddMacro(LCTRL, LALT, PAUS, 0, 0, 0, RebootMacro);
}

void ReadMatrix(void)
{
    uint8_t row;
	for (row = 0; row < NUM_ROWS; row++) {
        SelectRow(row);
        CheckColForRow(row);
        UnselectRows();
    }
}

void CheckColForRow(uint8_t row)
{
    // col: 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14
    // pin: C6  D3  B5  D2  D1  D0  B7  B3  B2  B1  B0  F5  F4  F0  F7
    // A switch is on when it's pin is also pulled to ground
    PINC & (1<<6) ? SetRowAndCol(row, 0, false) : SetRowAndCol(row, 0, true);
    PIND & (1<<3) ? SetRowAndCol(row, 1, false) : SetRowAndCol(row, 1, true);
    PINB & (1<<5) ? SetRowAndCol(row, 2, false) : SetRowAndCol(row, 2, true);
    PIND & (1<<2) ? SetRowAndCol(row, 3, false) : SetRowAndCol(row, 3, true);
    PIND & (1<<1) ? SetRowAndCol(row, 4, false) : SetRowAndCol(row, 4, true);
    PIND & (1<<0) ? SetRowAndCol(row, 5, false) : SetRowAndCol(row, 5, true);
    PINB & (1<<7) ? SetRowAndCol(row, 6, false) : SetRowAndCol(row, 6, true);
    PINB & (1<<3) ? SetRowAndCol(row, 7, false) : SetRowAndCol(row, 7, true);
    PINB & (1<<2) ? SetRowAndCol(row, 8, false) : SetRowAndCol(row, 8, true);
    PINB & (1<<1) ? SetRowAndCol(row, 9, false) : SetRowAndCol(row, 9, true);
    PINB & (1<<0) ? SetRowAndCol(row, 10, false) : SetRowAndCol(row, 10, true);
    PINF & (1<<5) ? SetRowAndCol(row, 11, false) : SetRowAndCol(row, 11, true);
    PINF & (1<<4) ? SetRowAndCol(row, 12, false) : SetRowAndCol(row, 12, true);
    PINF & (1<<0) ? SetRowAndCol(row, 13, false) : SetRowAndCol(row, 13, true);
    PINF & (1<<7) ? SetRowAndCol(row, 14, false) : SetRowAndCol(row, 14, true);
}

void UnselectRows(void)
{
    // Hi-Z (floating)
	DDRB  &= ~0b00010000;
	PORTB &= ~0b00010000;
    DDRD  &= ~0b10110000;
    PORTD &= ~0b10110000;
	DDRC  &= ~0b10000000;
	PORTC &= ~0b10000000;
    _delay_us(IO_WAIT_US);
}

void SelectRow(uint8_t row)
{
    // Pull selected pin to ground
    switch (row) {
        case 0:
            DDRB  |=  (1<<4);
            PORTB &= ~(1<<4);
            break;
        case 1:
            DDRD  |=  (1<<7);
            PORTD &= ~(1<<7);
            break;
        case 2:
            DDRD  |=  (1<<4);
            PORTD &= ~(1<<4);
            break;
        case 3:
            DDRD  |=  (1<<5);
            PORTD &= ~(1<<5);
            break;
        case 4:
            DDRC  |=  (1<<7);
            PORTC &= ~(1<<7);
            break;
    }
    _delay_us(IO_WAIT_US);
}

void SetRowAndCol(uint8_t row, uint8_t col, bool val)
{
#if DEBOUNCE
	if (val == true) {
		debounce[row][col].Off = 0;
		if (debounce[row][col].On++ >= DEBOUNCE) {
			currKeys[row][col] = true;
		}
	} else {
		debounce[row][col].On = 0;
		if (debounce[row][col].Off++ >= DEBOUNCE) {
			currKeys[row][col] = false;
		}
	}
#else
    currKeys[row][col] = val;
#endif
}

// --------- MACROS ----------------------------------------------------

void AddMacro(uint8_t k1, uint8_t k2, uint8_t k3,
			  uint8_t k4, uint8_t k5, uint8_t k6,
			  MacroHandler handler)
{
	if (macroIndex > 32) {
		return;
	}
	struct KeyMacro m;
//	uint8_t k[6] = {k1,k2,k3,k4,k5,k6};
	m.Handler = handler;
//	memcpy(&m.Keys, &k, sizeof(k));
	memcpy(&macros[macroIndex++], &m, sizeof(m));
}

// --------- RESOLVE KEY PRESSES ---------------------------------------

void ResolveFunctionKeys(void)
{
    uint8_t row, col;
    
    for (row = 0; row < NUM_ROWS; row++) {
        for (col = 0; col < NUM_COLS; col++) {
            
            uint8_t key = layers[0][row][col];
            uint8_t state = currKeys[row][col];
            uint8_t last_state = lastKeys[row][col];
            
            if (IsFunctionKey(key) && (state != last_state)) {
                
                if (state == true) {
                    currLayer = 1;
                } else {
                    currLayer = 0;
                }
                
            }
        }
    }
}

void ResolveModifierKeys(void)
{
    uint8_t row, col;
        
    for (row = 0; row < NUM_ROWS; row++) {
        for (col = 0; col < NUM_COLS; col++) {
            
            uint8_t key = layers[0][row][col];
            uint8_t state = currKeys[row][col];
            uint8_t last_state = lastKeys[row][col];
            
            if (IsModifierKey(key) && (state != last_state)) {
                
                if (state == true) {
                    keyboard_modifier_keys |= TO_MOD(layers[currLayer][row][col]);
                } else {
                	keyboard_modifier_keys &= ~TO_MOD(layers[currLayer][row][col]);
                }
            }
        }
    }
}

void ResolveNormalKeys(void)
{
    uint8_t row, col;
    
    for (row = 0; row < NUM_ROWS; row++) {
        for (col = 0; col < NUM_COLS; col++) {
            
            uint8_t key = layers[currLayer][row][col];
            uint8_t state = currKeys[row][col];
            uint8_t last_state = lastKeys[row][col];
            
            if (IsNormalKey(key) && (state != last_state)) {
                
                if (state == true) {
                    InsertKey(key);
                } else {
                    RemoveKey(key);
                }
            }
        }
    }
}

void ResolveMacros(void)
{
	uint8_t i;
	uint8_t count = 0;
	for (i = 0; i < 6; i++) {
		uint8_t key = keyboard_keys[i];
		if (key == UP) {
			count++;
		}
		if (key == DEL) {
			count++;
		}
	}
	if (count > 1) {
		Reboot();
	}
}

inline bool IsModifierKey(uint8_t key)
{
	if (key == LCTRL || key == LSHIFT || key == LALT || key == LGUI ||
        key == RCTRL || key == RSHIFT || key == RALT || key == RGUI) {
        return true;
    } else {
        return false;
    }
}

inline bool IsFunctionKey(uint8_t key)
{
	return (((key == FN0) || (key == FN1) || (key == FN2)) ? true : false);
}

inline bool IsNormalKey(uint8_t key)
{
    if ((IsModifierKey(key) == false) && (IsFunctionKey(key) == false)) return true;
    else return false;
}

uint8_t free_index = 0;
void InsertKey(uint8_t key)
{
    uint8_t i;
    
    if (free_index >= sizeof(keyboard_keys)) {
        return;
    }
    
    for (i = 0; i < sizeof(keyboard_keys); i++) {
        if (keyboard_keys[i] == 0) {
            free_index = i;
			break;
        }
    }
    
    keyboard_keys[free_index] = key;
}

void RemoveKey(uint8_t key)
{
    uint8_t i;
    for (i = 0; i < sizeof(keyboard_keys); i++) {
        if (keyboard_keys[i] == key) {
            keyboard_keys[i] = 0;
            free_index = i;
        }
    }
}

void Reboot(void)
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
