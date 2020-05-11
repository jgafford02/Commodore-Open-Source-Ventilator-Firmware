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

/* Constants
 *  DESCRIPTION: Handles printing to LCD display for Venilator_Motor_Controller
 */

#ifndef Display_h
#define Display_h

#include "Arduino.h"
#include <Wire.h>
#include <hd44780.h>  
#include <hd44780ioClass/hd44780_I2Cexp.h>  // include i/o class header


namespace display {

static const int kWidth = 16;  // Width of the display
static const int kHeight = 2;  // Height of the display

/**
  Display
  Handles writing controller-specific elements to the display.
*/

class Display {

  public:
    // Constructor, save a pointer to the (global) display object
    Display(hd44780* lcd):
      lcd_(lcd)
    {
    }

    //Initialize display object
    void begin();

    //Clear display contents
    void clear();

    //Write disabled state to LCD
    void writeDisabled();

    //Write vent parameters in normal mode
    void writeNormalMode(const int& cycle, const float& pressure, const float& bpm, const float& ie);

    //Write programming parameters in pressure programming mode
    void writeProgModePressure(const float& pmax, const float& pmin);

    //Write programming parameters in rate programming mode
    void writeProgModeRate(const float& bpm, const float& ie);

    //Write programming parameters in BIPAP programming mode
    void writeProgModeBIPAP(const float& pmin, const float& timeout);

    //Write vent mode startup prompt
    void writeVentStart();

    //Write BiPAP mode startup prompt
    void writeBIPAPStart();

    //Write breath waiting prompt
    void writeBreathWaiting(const int& cycle, const float& pressure, const float& t_elapsed);

    //Write breath request prompt
    void writeBreathRequest(const float& pressure, const float& pmax);

    //Write inspiratory hold parameters
    void writeInspHold(const float& pressure, const float& pmax);

    //Write homing sequence
    void writeHome();

    //Write two character arrays to lines 1 and 2 of LCD
    void writeString(const char* _line1, const char* _line2);

    //Write a combination of character arrays and numerical parameter
    template <typename T>
    void writeStringParameter(const char* _line1, const char* _line2, const T& param);

    //Write welcome prompt
    void welcome();

  private:
    hd44780* lcd_;
    
    // Write printable starting at (row, col)
    template <typename T>
    void write(const int& row, const int& col, const T& printable);
};

// Instantiation of template methods
#define INSTANTIATE_WRITE(type) \
  template void Display::writeStringParameter(const char* _line1, const char* _line2, const type& param);
INSTANTIATE_WRITE(int)
INSTANTIATE_WRITE(float)
INSTANTIATE_WRITE(unsigned long)
INSTANTIATE_WRITE(byte)
#undef INSTANTIATE

}

#endif
