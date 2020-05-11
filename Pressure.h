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
 *  DESCRIPTION: Defines a new Pressure class for reading pressure sensor
 */

#ifndef Pressure_h
#define Pressure_h

#if ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
	#include "pins_arduino.h"
	#include "WConstants.h"
#endif

class Pressure {
public:
	Pressure(byte _PS);

  //Read pressure, scale to CMH2O, and calculate max
	void read();

  //Reset max
	void reset();

  //Get current pressure reading
  const float& get();

  //Set PIP
  void setPIP();

  //Set Plateau Pressure
  void setPlateau();

  //Set PEEP Pressure
  void setPEEP();

  //Return max pressure
  const float& get_max();

  //Return PIP Pressure
  const float& get_PIP();

  //Return Plateau pressure
  const float& get_Plateau();

  //Return PEEP pressure
  const float& get_PEEP();

private:
	byte _PS;
  float _current_pressure;
  float _max_pressure;
  float _pip, _plat, _peep;
};

#endif
