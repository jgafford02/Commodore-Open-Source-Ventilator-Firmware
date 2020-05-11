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
 
#include "Utilities.h"

namespace utils{

//=============CALCULATE INSPIRATION TIME=======================//
//DESCRIPTION
//This function calculates inspiration time based on current BPM and IE
//settings
float get_insp_time(float BPM, float IE) {
	int cycle_time = BPM * 60;
	return cycle_time * (1 / IE);
}

//=============CALCULATE EXPIRATION TIME=======================//
//DESCRIPTION
//This function calculates expiration time based on current BPM and IE
//settings
float get_exp_time(float BPM, float IE) {
	int cycle_time = BPM * 60;
	return cycle_time * (IE - 1);
}

//=============CALCULATE BPM AND IE=======================//
//DESCRIPTION
//This function maps calculates BPM and IE from a given input voltage (8-bit).
//Note that maximum BPM is a function of the maximum motor speed and IE (i.e. at
//it's fastest, the motor can manage 43 BPM at 1:1 I/E, so if we drop the I/E,
//the maximum achievable BPM drops). So this function adjusts the BPM based
//on the desired I/E
float getBPM10bit(int V, float IE, int max_val, int min_val) {
	float max_insp_cycle = 1 / (2.0 * (max_val) / 60.0);
	float insp_cycle = max_insp_cycle * (1024.0 / (1024.0-V));
	float exp_cycle = insp_cycle * IE;
	return (60.0 / (insp_cycle + exp_cycle));
}

//===================MAP TO 10-bit=======================//
//DESCRIPTION
//This function maps an 8-bit input to a range specified by max_val and min_val
float getParam10bit(int V, int max_val, int min_val) {
	return V * ((max_val - min_val) / 1024.0) + min_val;
}

//===================MAP TO RANGE=======================//
//DESCRIPTION
//This function maps variable x, which can have a minimum and maximum value
//of in_min and in_max, to variable y which is between out_min and out_max
float map2(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//===================MAP TO 8-bit=======================//
//DESCRIPTION
//This function maps an 8-bit input to a range specified by max_val and min_val
float getParam(int V, int max_val, int min_val) {
	return V * ((max_val - min_val) / 255.0) + min_val;
}


}
