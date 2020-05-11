/**
 * Vanderbilt License
 * 
 * Copyright (c) 2020 Vanderbilt Commodore Open-Source Ventilator
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Motor
 *  DESCRIPTION: Defines a new Motor class for controlling the motor
 */


#include "SmartKnob.h"


//==================CREATE INSTANCE=====================//
//DESCRIPTION
//Creates instance of the class and initializes
SmartKnob::SmartKnob(const byte pin_number) {
	_pin = pin_number;
  initialize();
}


void SmartKnob::initialize() {
	pinMode(_pin, INPUT);
	_current_val = analogRead(_pin);
	_last_val = analogRead(_pin);
}

void SmartKnob::loop() {
	_current_val = analogRead(_pin);
}

bool SmartKnob::hasChanged() {
  _current_val = analogRead(_pin);
	if (abs(_current_val-_last_val)>2) {
    _last_val = _current_val;
		return true;
	}
	else {
		return false;
	}
}

int SmartKnob::get() {
	return _current_val;
}

byte SmartKnob::get_8bit() {
	return _current_val / 4;
}
