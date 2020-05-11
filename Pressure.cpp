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


#include "Pressure.h"
#include "Utilities.h"

using namespace utils;


//==================CREATE INSTANCE=====================//
//DESCRIPTION
//Creates instance of the class and initializes
Pressure::Pressure(byte PS) {
  _PS = PS;
}

//==================MOTOR INITIALIZE=====================//
//DESCRIPTION
//Set PWM Frequence, define pin directions and enable
void Pressure::read() {

  //Read sensor and scale to CMH2O
  float val = analogRead(_PS);
  float voltage = map2(val, 0, 1024, 0, 5);
  float pressure_KPA = map2(voltage, 0.2, 4.7, 0, 10);
  float pressure_CMH2O = map2(pressure_KPA, 0, 10, 0, 101.972);
  
  _max_pressure = max(_max_pressure, pressure_CMH2O);
  _current_pressure = pressure_CMH2O;
}

//==============MOTOR DIRECTION CONTROL===================//
//DESCRIPTION
//This function controls the speed and direction of the motor
void Pressure::reset() {
  _max_pressure = 0.0;
}

const float& Pressure::get(){
  return _current_pressure;
}

void Pressure::setPIP() {
  _pip = get();
}

void Pressure::setPlateau(){
  _plat = get();
}

void Pressure::setPEEP(){
  _peep = get();
}

const float& Pressure::get_max(){
  return _max_pressure;
}

const float& Pressure::get_PIP(){
  return _pip;
}

const float& Pressure::get_Plateau(){
  return _plat;
}

const float& Pressure::get_PEEP(){
  return _peep;
}
