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
#define NUM_COLS 14
#define NUM_LAYERS 2

#define INIT_WAIT 1000
#define DEBOUNCE_WAIT 2

static const uint8_t layers[NUM_LAYERS][NUM_ROWS][NUM_COLS] = {
    /* Layer 0 */
    {{KEY_TILDE,      KEY_1,   KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8,     KEY_9,      KEY_0,         KEY_MINUS,       KEY_EQUAL,        KEY_BACKSPACE},
     {KEY_TAB,        KEY_Q,   KEY_W, KEY_E, KEY_R, KEY_T, KEY_Y, KEY_U, KEY_I,     KEY_O,      KEY_P,         KEY_LEFT_BRACE,  KEY_RIGHT_BRACE,  KEY_BACKSLASH},
     {KEY_FN0,        KEY_A,   KEY_S, KEY_D, KEY_F, KEY_G, KEY_H, KEY_J, KEY_K,     KEY_L,      KEY_SEMICOLON, KEY_QUOTE,       KEY_ENTER,        KEY_NONE},
     {KEY_LEFT_SHIFT, KEY_Z,   KEY_X, KEY_C, KEY_V, KEY_B, KEY_N, KEY_M, KEY_COMMA, KEY_PERIOD, KEY_SLASH,     KEY_RIGHT_SHIFT, KEY_NONE,         KEY_PAUSE},
     {KEY_LEFT_CTRL,  KEY_GUI, KEY_LEFT_ALT, KEY_SPACE,KEY_NONE,KEY_NONE,KEY_NONE,KEY_NONE,KEY_NONE,KEY_NONE,  KEY_RIGHT_ALT,   KEY_FN0, KEY_FN1, KEY_FN2}},
    
    /* Layer 1 */
    {{0,0,0,0,0,0,0,0,0,0,0,0,0,0},
     {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
     {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
     {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
     {0,0,0,0,0,0,0,0,0,0,0,0,0,0}}
};

// --------- LOCAL VARIABLES --------------------------------------------------

//! Current state of the keys
bool keys[NUM_ROWS][NUM_COLS];


// --------------------------------------
// keys are stored in:
// uint8_t keyboard_modifier_keys;
// uint8_t keyboard_keys[6];
// using keycodes from usb_keyboard.h
// --------------------------------------


// --------- FUNCTION PROTOTYPES ----------------------------------------------

void kb_loop(void);
void kb_init(void);
void kb_read_matrix(void);
bool kb_read_col(int col);
void kb_set_row(int row, bool high);
void do_key_up(int row, int col);
void do_key_dn(int row, int col);

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
    // ------- Setup pins -------------------------------------------------------------------
    
    // ROW:  0,  1,  2,  3,  4
    // PIN: C0, C1, C2, C3, C4
    
    // COL:  0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10,  11,  12,  13
    // PIN: B0, B1, B2, B3, B4, B5, B6, B7, D0, D1,  D2,  D3,  D6,  F7
    
    // We are driving the rows, so set the row pins (5 of them) as outputs
    // and default them to low
    DDRC |= (1<<4 | 1<<3 | 1<<2 | 1<<1 | 1<<0);   // C0-C4 inputs
    PORTC &= ~(1<<4 | 1<<3 | 1<<2 | 1<<1 | 1<<0); // C0-C4 set low
    
    // We are reading the columns, so set the column pins (14 of them) as
    // inputs with pullup resistors
    DDRF &= ~(1<<7); // F7 input
    PORTF |= (1<<7); // F7 pullups
    DDRB &= ~(1<<7 | 1<<6 | 1<<5 | 1<<4 | 1<<3 | 1<<2 | 1<<1 | 1<<0); // B0-B7 inputs
    PORTB |= (1<<7 | 1<<6 | 1<<5 | 1<<4 | 1<<3 | 1<<2 | 1<<1 | 1<<0); // B0-B7 pullups
    DDRD &= ~(1<<6 | 1<<3 | 1<<2 | 1<<1 | 1<<0); // D0-D3, D6 inputs
    PORTD |= (1<<6 | 1<<3 | 1<<2 | 1<<1 | 1<<0); // D0-D3, D6 pullups
}

void kb_read_matrix(void)
{
    int row, col;
    
    // Loop through all rows
    for (row = 0; row < NUM_ROWS; row++) {
       
        // Set row output to high
        kb_set_row(row, true);
        
        // Loop through all columns
        for (col = 0; col < NUM_COLS; col++) {
            
            // If input is high the key at row, col is pressed
            if (kb_read_col(col)) {
                keys[row][col] = true;
            } else {
                keys[row][col] = false;
            }
        }
        
        // Set row back to low
        kb_set_row(row, false);
    }
}

bool kb_read_col(int col)
{
    /* A much better way to do this would be to use a map for pin to col
     or do something with macros. This pinout is duplicated here and 
     is a possible source for bugs. */
    
    bool high = false;
    switch (col) {
        case 0:
            if (PINB & (1<<0)) high = true;
            break;
        case 1:
            if (PINB & (1<<1)) high = true;
            break;
        case 2:
            if (PINB & (1<<2)) high = true;
            break;
        case 3:
            if (PINB & (1<<3)) high = true;
            break;
        case 4:
            if (PINB & (1<<4)) high = true;
            break;
        case 5:
            if (PINB & (1<<5)) high = true;
            break;
        case 6:
            if (PINB & (1<<6)) high = true;
            break;
        case 7:
            if (PINB & (1<<7)) high = true;
            break;
        case 8:
            if (PIND & (1<<0)) high = true;
            break;
        case 9:
            if (PIND & (1<<1)) high = true;
            break;
        case 10:
            if (PIND & (1<<2)) high = true;
            break;
        case 11:
            if (PIND & (1<<3)) high = true;
            break;
        case 12:
            if (PIND & (1<<6)) high = true;
            break;
        case 13:
            if (PINF & (1<<7)) high = true;
            break;
    }
    
    return high;
}

void kb_set_row(int row, bool high)
{
    /* A much better way to do this would be to use a map for pin to row
     or do something with macros. */
    
    switch (row) {
        case 0:
            high ? (PORTC != (1<<0)) : (PORTC &= ~(1<<0));
            break;
        case 1:
            high ? (PORTC != (1<<1)) : (PORTC &= ~(1<<1));
            break;
        case 2:
            high ? (PORTC != (1<<2)) : (PORTC &= ~(1<<2));
            break;
        case 3:
            high ? (PORTC != (1<<3)) : (PORTC &= ~(1<<3));
            break;
        case 4:
            high ? (PORTC != (1<<4)) : (PORTC &= ~(1<<4));
            break;
    }
}

void do_key_up(int row, int col)
{
    
}

void do_key_dn(int row, int col)
{
    
}


