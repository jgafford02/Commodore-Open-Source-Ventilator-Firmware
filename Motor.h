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

#ifndef Motor_h
#define Motor_h

#if ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
	#include "pins_arduino.h"
	#include "WConstants.h"
#endif

class Motor {
public:
	Motor(byte P1, byte P2, byte EN);

  //Initialize motor
	void initialize();

  //Set motor speed instantaneously
	void set_speed(int sp, boolean dir);

  //Ramp up to motor speed
	void softstart(int sp, boolean dir);

  //Linearly transition between two speeds
	void soft_transition(int sp1, int sp2, boolean dir);

  //Soft brake by ramping down to 0 speed
	void softbrake(int sp, boolean dir);

  //Hard brake
  void brake(boolean dir);

private:
	byte _P1;
	byte _P2;
	byte _EN;
  const int _delay_microseconds = 250;
};

#endif
