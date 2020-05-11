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

/* Motor
    DESCRIPTION: Defines a new Motor class for controlling the motor
*/


#include "Motor.h"


//==================CREATE INSTANCE=====================//
//DESCRIPTION
//Creates instance of the class and initializes
Motor::Motor(byte P1, byte P2, byte EN) {
  _P1 = P1;   //PWM1
  _P2 = P2;   //PWM2
  _EN = EN;   //Phase
  initialize();
}

//==================MOTOR INITIALIZE=====================//
//DESCRIPTION
//Set PWM Frequence, define pin directions and enable
void Motor::initialize() {

  //Increase PWM Frequency on 9 and 10 to 4 kHz to reduce ringing (UNO Only)
  TCCR1B = TCCR1B & B11111000 | B00000010;

  // Define pin directions and enable
  pinMode(_EN, OUTPUT);
  pinMode(_P1, OUTPUT);
  pinMode(_P2, OUTPUT);
  digitalWrite(_EN, HIGH);  //enable motors
}

//==============MOTOR DIRECTION CONTROL===================//
//DESCRIPTION
//This function controls the speed and direction of the motor
void Motor::set_speed(int sp, boolean dir) {
  if (dir == true) {
    analogWrite(_P2, 0);
    analogWrite(_P1, sp);
  }
  else {
    analogWrite(_P1, 0);
    analogWrite(_P2, sp);
  }
}

//==============MOTOR SOFT START===================//
//DESCRIPTION
//Gently starts the motor by slowly ramping up to speed
void Motor::softstart(int sp, boolean dir) {
  for (int i = 1; i <= sp; i++) {
    set_speed(i, dir);
    delayMicroseconds(_delay_microseconds);
  }
}

//=============MOTOR SOFT TRANSITION================//
//DESCRIPTION
//Gently transitions the motor speed from sp1 to sp2
void Motor::soft_transition(int sp1, int sp2, boolean dir) {
  if (sp1 > sp2) {
    for (int i = sp1; i >= sp2; i--) {
      set_speed(i, dir);
      delayMicroseconds(_delay_microseconds);
    }
  }
  else {
    for (int i = sp1; i <= sp2; i++) {
      set_speed(i, dir);
      delayMicroseconds(_delay_microseconds);
    }
  }
}

//==============MOTOR SOFT BRAKE===================//
//DESCRIPTION
//Gently brakes the motor by slowly ramping down speed
void Motor::softbrake(int sp, boolean dir) {
  for (int i = sp; i >= 0; i--) {
    set_speed(i, dir);
    delayMicroseconds(_delay_microseconds);
  }
}

//==============MOTOR HARD BRAKE===================//
//DESCRIPTION
//Swiftly brake motor by sending short pulse in opposite direction
void Motor::brake(boolean dir) {
  set_speed(255, ~dir);
  delayMicroseconds(_delay_microseconds);
  set_speed(0, dir);
}
