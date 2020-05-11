/**
   Vanderbilt License

   Copyright (c) 2020 Vanderbilt Commodore Open-Source Ventilator

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
*/

/* Constants
    DESCRIPTION: Handles printing to LCD display for Venilator_Motor_Controller
*/

//==================CREATE INSTANCE=====================//
//DESCRIPTION
//Creates instance of the class and initializes
#include "Display.h"

namespace display {

void Display::begin() {
  lcd_->begin(kWidth, kHeight);
  lcd_->noCursor();
  lcd_->leftToRight();
  welcome();
}

void Display::writeDisabled() {
  const char* header_1 = "DISABLED        ";
  const char* header_2 = "ENABLE TO START ";
  write(0, 0, header_1);
  write(1, 0, header_2);
}

void Display::writeHome() {
  const char* header_1 = "HOMING VENT     ";
  const char* header_2 = "PLEASE STANDBY..";
  write(0, 0, header_1);
  write(1, 0, header_2);
}

void Display::writeNormalMode(const int& cycle, const float& pressure, const float& bpm, const float& ie) {
  char _p_buff[5];
  dtostrf(pressure, 2, 0, _p_buff);
  char _bpm_buff[5];
  dtostrf(bpm, 2, 0, _bpm_buff);
  char _ie_buff[5];
  dtostrf(ie, 2, 1, _ie_buff);
  char _line1_buff[20];
  sprintf(_line1_buff, "NUM:%4d   P:%3s", cycle, _p_buff);
  write(0, 0, _line1_buff);
  char _line2_buff[20];
  sprintf(_line2_buff, "BPM:%2s IE:1:%4s", _bpm_buff, _ie_buff);
  write(1, 0, _line2_buff);

}

void Display::writeProgModePressure(const float& pmax, const float& pmin) {
  char _pmax_buff[5];
  dtostrf(pmax, 2, 0, _pmax_buff);
  char _pmin_buff[5];
  dtostrf(pmin, 2, 0, _pmin_buff);
  char _line1_buff[20];
  write(0, 0, "PRESS PP TO SET:");
  char _line2_buff[20];
  sprintf(_line2_buff, "Pmax:%2s  Pmin:%2s", _pmax_buff, _pmin_buff);
  write(1, 0, _line2_buff);
}


void Display::writeProgModeRate(const float& bpm, const float& ie) {
  char _bpm_buff[5];
  dtostrf(bpm, 2, 0, _bpm_buff);
  char _ie_buff[5];
  dtostrf(ie, 2, 1, _ie_buff);
  write(0, 0, "PRESS PP TO SET:");
  char _line2_buff[20];
  sprintf(_line2_buff, "BPM:%3s IE:1:%3s", _bpm_buff, _ie_buff);
  write(1, 0, _line2_buff);
}


void Display::writeProgModeBIPAP(const float& pmin, const float& timeout) {
  char _pmin_buff[5];
  dtostrf(pmin, 2, 0, _pmin_buff);
  char _timeout_buff[5];
  dtostrf(timeout, 3, 0, _timeout_buff);
  write(0, 0, "PRESS PP TO SET:");
  char _line2_buff[20];
  sprintf(_line2_buff, "Pt:%3s  Tmin:%3s", _pmin_buff, _timeout_buff);
  write(1, 0, _line2_buff);

}

void Display::writeVentStart() {
  const char* header_1 = "VENT MODE       ";
  const char* header_2 = "SELECTED...     ";
  write(0, 0, header_1);
  write(1, 0, header_2);
}

void Display::writeBIPAPStart() {
  const char* header_1 = "BIPAP MODE      ";
  const char* header_2 = "SELECTED...     ";
  write(0, 0, header_1);
  write(1, 0, header_2);
}


void Display::welcome() {
  const char* _line1 = "Firmware v8.0   ";
  const char* _line2 = "Initializing....";
  write(0, 0, _line1);
  write(1, 0, _line2);

}

void Display::clear() {
  lcd_->clear();
}


void Display::writeBreathRequest(const float& pressure, const float& pmax) {
  char _p_buff[5];
  dtostrf(pressure, 2, 0, _p_buff);
  char _pmax_buff[5];
  dtostrf(pmax, 2, 0, _pmax_buff);
  const char* title_string = "BREATH REQUEST  ";
  write(0, 0, "BREATH REQUEST  ");
  char _line2_buff[20];
  sprintf(_line2_buff,"P:%2s     Pmax:%2s", _p_buff, _pmax_buff);
  write(1, 0, _line2_buff);
}

void Display::writeBreathWaiting(const int& cycle, const float& pressure, const float& t_elapsed) {
  char _p_buff[5];
  dtostrf(pressure, 2, 0, _p_buff);
  char _t_buff[5];
  dtostrf(t_elapsed, 2, 1, _t_buff);
  char _line1_buff[20];
  sprintf(_line1_buff, "Cycles:%4d WAIT", cycle);
  write(0, 0, _line1_buff);
  char _line2_buff[20];
  sprintf(_line2_buff, "P:%2s   Time:%4s", _p_buff, _t_buff);
  write(1, 0, _line2_buff);
}

void Display::writeInspHold(const float& pressure, const float& pmax) {
  char _p_buff[5];
  dtostrf(pressure, 3, 1, _p_buff);
  char _pmax_buff[5];
  dtostrf(pmax, 3, 1, _pmax_buff);
  char _line1_buff[20];
  sprintf(_line1_buff, "PIP:%3s cmH2O   ", _pmax_buff);
  write(0, 0, _line1_buff);
  char _line2_buff[20];
  sprintf(_line2_buff, "PLAT:%3s cmH2O  ", _p_buff);
  write(1, 0, _line2_buff);
}

template <typename T>
void Display::writeStringParameter(const char* line1, const char* line2, const T& param) {
  float _tofloat = (float) param; //cast to float
  char buff1[17];
  strcpy(buff1, line1);
  write(0, 0, buff1);
  size_t buff1_length = strlen(buff1);
  if (buff1_length < kWidth) {
    for (int i = (buff1_length); i < kWidth; i++) {
      write(0, i, " ");
    }
  }
  char buff2[17];
  strcpy(buff2, line2);
  write(1, 0, buff2);
  size_t buff2_length = strlen(buff2);
  char buff[5];
  dtostrf(_tofloat, 4, 2, buff);
  write(1, buff2_length, buff);
  for (int i = (buff2_length + 5); i < kWidth; i++) {
    write(1, i, " ");
  }
}


void Display::writeString(const char* line1, const char* line2) {
  char buff1[17];
  strcpy(buff1, line1);
  write(0, 0, line1);
  size_t buff1_length = strlen(buff1);
  if (buff1_length < kWidth) {
    for (byte i = buff1_length; i < kWidth; i++) {
      write(0, i, F(" "));
    }
  }
  char buff2[17];
  strcpy(buff2, line2);
  write(1, 0, line2);
  size_t buff2_length = strlen(buff2);
  if (buff2_length < kWidth) {
    for (byte i = buff2_length; i < kWidth; i++) {
      write(1, i, F(" "));
    }
  }
}


template <typename T>
void Display::write(const int& row, const int& col, const T& printable) {
  lcd_->setCursor(col, row);
  lcd_->print(printable);
}
}
