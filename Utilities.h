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
 *  DESCRIPTION: Utility functions used in the main firmware code
 */

#ifndef Utilities_h
#define Utilities_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#include "WConstants.h"
#endif


namespace utils{

//Compute inspiration time
float get_insp_time(float BPM, float IE);

//Comput expiration time
float get_exp_time(float BPM, float IE);

float getBPM10bit(int V, float IE, int max_val, int min_val);

float getParam10bit(int V, int max_val, int min_val);

float map2(float x, float in_min, float in_max, float out_min, float out_max);

float getParam(int V, int max_val, int min_val);


}

#endif
