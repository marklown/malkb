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

#define LED_CONFIG	(DDRD |= (1<<6))
#define LED_ON		(PORTD &= ~(1<<6))
#define LED_OFF		(PORTD |= (1<<6))
#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

#define NUM_ROWS 5
#define NUM_COLS 14
#define NUM_LAYERS 2

static const uint8_t layers[NUM_LAYERS][NUM_ROWS][NUM_COLS] = {
    /* Layer 0 */
    {{KEY_TILDE,      KEY_1,   KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8,     KEY_9,      KEY_0,         KEY_MINUS,       KEY_EQUAL,        KEY_BACKSPACE},
     {KEY_TAB,        KEY_Q,   KEY_W, KEY_E, KEY_R, KEY_T, KEY_Y, KEY_U, KEY_I,     KEY_O,      KEY_P,         KEY_LEFT_BRACE,  KEY_RIGHT_BRACE,  KEY_BACKSLASH},
     {KEY_FN1,        KEY_A,   KEY_S, KEY_D, KEY_F, KEY_G, KEY_H, KEY_J, KEY_K,     KEY_L,      KEY_SEMICOLON, KEY_QUOTE,       KEY_ENTER,        KEY_NONE},
     {KEY_LEFT_SHIFT, KEY_Z,   KEY_X, KEY_C, KEY_V, KEY_B, KEY_N, KEY_M, KEY_COMMA, KEY_PERIOD, KEY_SLASH,     KEY_RIGHT_SHIFT, KEY_NONE,         KEY_PAUSE},
     {KEY_LEFT_CTRL,  KEY_GUI, KEY_LEFT_ALT, KEY_SPACE,KEY_NONE,KEY_NONE,KEY_NONE,KEY_NONE,KEY_NONE,KEY_NONE,  KEY_RIGHT_ALT,   KEY_FN0, KEY_FN1, KEY_FN2}},
    
    /* Layer 1*/
    {{0,0,0,0,0,0,0,0,0,0,0,0,0,0},
     {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
     {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
     {0,0,0,0,0,0,0,0,0,0,0,0,0,0},
     {0,0,0,0,0,0,0,0,0,0,0,0,0,0}}
};

// --------- LOCAL VARIABLES --------------------------------------------------



// --------------------------------------
// keys are stored in:
// uint8_t keyboard_modifier_keys;
// uint8_t keyboard_keys[6];
// using keycodes from usb_keyboard.h
// --------------------------------------


// --------- FUNCTION PROTOTYPES ----------------------------------------------

void kb_loop(void);
void kb_init(void);
void do_key_up(int row, int col);
void do_key_dn(int row, int col);

// ----------------------------------------------------------------------------
// Main entry point for MalKb firmware
// ----------------------------------------------------------------------------
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
    _delay_ms(1000);
    
    // Main loop
    kb_loop();
    
    return 0;
}

// ----------------------------------------------------------------------------
// Main entry point for MalKb firmware
// ----------------------------------------------------------------------------
void kb_loop(void)
{
    int row, col;
    bool isPressed = false;
    bool wasPressed = false;
    
    // Main keyboard loop
    while (1) {
        
        // Read the key matrix
        // --- TODO
        
        // Loop over each row and column in key matrix
        for (row = 0; row < NUM_ROWS; row++) {
            for (col = 0; col < NUM_COLS; col++) {
                
                // Get isPressed and wasPressed
                // --- TODO
                
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
        
        // Save current matrix as last
        // --- TODO
        
        // Wait a small amount of time to debounce
        _delay_ms(2);
    }

}

// ----------------------------------------------------------------------------
// Main entry point for MalKb firmware
// ----------------------------------------------------------------------------
void kb_init(void)
{
    // Map pins to row and col
    // --- TODO
    
    
}


