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

#ifndef MalKb_malkb_h
#define MalKb_malkb_h

#include "usb_keyboard.h"

typedef uint8_t bool;
#define true 1
#define false 0

// Modifiers
#define LCTRL   200
#define LSHIFT  201
#define LALT    202
#define LGUI    203
#define RCTRL   204
#define RSHIFT  205
#define RALT    206
#define RGUI    207

// We need to convert from our convention to the usb keyboard protocol.
// We do this because the modifiers in the keyboard protocol share some
// values with the alpha-numeric keys. We need them to be unique.
inline uint8_t TO_MOD(uint8_t key)
{
    switch (key) {
        case LCTRL:
            return KEY_LEFT_CTRL;
        case LSHIFT:
            return KEY_LEFT_SHIFT;
        case LALT:
            return KEY_LEFT_ALT;
        case LGUI:
            return KEY_LEFT_GUI;
        case RCTRL:
            return KEY_RIGHT_CTRL;
        case RSHIFT:
            return KEY_RIGHT_SHIFT;
        case RALT:
            return KEY_RIGHT_ALT;
        case RGUI:
            return KEY_RIGHT_GUI;
        default:
            return 0;
    }
}

// Alpha-numeric
#define A   KEY_A
#define B   KEY_B
#define C   KEY_C
#define D   KEY_D
#define E   KEY_E
#define F   KEY_F
#define G   KEY_G
#define H   KEY_H
#define I   KEY_I
#define J   KEY_J
#define K   KEY_K
#define L   KEY_L
#define M   KEY_M
#define N   KEY_N
#define O   KEY_O
#define P   KEY_P
#define Q   KEY_Q
#define R   KEY_R
#define S   KEY_S
#define T   KEY_T
#define U   KEY_U
#define V   KEY_V
#define W   KEY_W
#define X   KEY_X
#define Y   KEY_Y
#define Z   KEY_Z
#define N1  KEY_1
#define N2  KEY_2
#define N3  KEY_3
#define N4  KEY_4
#define N5  KEY_5
#define N6  KEY_6
#define N7  KEY_7
#define N8  KEY_8
#define N9  KEY_9
#define N0  KEY_0
#define ENTR    KEY_ENTER
#define ESC     KEY_ESC
#define BSPC    KEY_BACKSPACE
#define TAB     KEY_TAB
#define SPC     KEY_SPACE
#define MIN     KEY_MINUS
#define EQL     KEY_EQUAL
#define LBRC    KEY_LEFT_BRACE
#define RBRC    KEY_RIGHT_BRACE
#define BSLSH   KEY_BACKSLASH
#define NUM     KEY_NUMBER
#define SCOL    KEY_SEMICOLON
#define QUOT    KEY_QUOTE
#define TLDE    KEY_TILDE
#define COMM    KEY_COMMA
#define PRD     KEY_PERIOD
#define SLSH    KEY_SLASH
#define CPSLK   KEY_CAPS_LOCK
#define F1      KEY_F1
#define F2      KEY_F2
#define F3      KEY_F3
#define F4      KEY_F4
#define F5      KEY_F5
#define F6      KEY_F6
#define F7      KEY_F7
#define F8      KEY_F8
#define F9      KEY_F9
#define F10     KEY_F10
#define F11     KEY_F11
#define F12     KEY_F12
#define PSCN    KEY_PRINTSCREEN
#define SLCK    KEY_SCROLL_LOCK
#define PAUS    KEY_PAUSE
#define NSRT    KEY_INSERT
#define HOME    KEY_HOME
#define PGUP    KEY_PAGE_UP
#define DEL     KEY_DELETE
#define END     KEY_END
#define PGDN    KEY_PAGE_DOWN
#define RIGHT   KEY_RIGHT
#define LEFT    KEY_LEFT
#define DOWN    KEY_DOWN
#define UP      KEY_UP
#define NLCK    KEY_NUM_LOCK
#define KPSLSH  KEYPAD_SLASH
#define KPASTX  KEYPAD_ASTERIX
#define KPMIN   KEYPAD_MINUS
#define KPPLS   KEYPAD_PLUS
#define KPENT   KEYPAD_ENTER
#define KP1     KEYPAD_1
#define KP2     KEYPAD_2
#define KP3     KEYPAD_3
#define KP4     KEYPAD_4
#define KP5     KEYPAD_5
#define KP6     KEYPAD_6
#define KP7     KEYPAD_7
#define KP8     KEYPAD_8
#define KP9     KEYPAD_9
#define KP0     KEYPAD_0
#define KPPRD   KEYPAD_PERIOD

// Function
#define FN0     100
#define FN1     101
#define FN2     102
#define NONE    0



#endif
