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
 *  DESCRIPTION: Defines all of the constants used in Venilator_Motor_Controller
 */

#ifndef Constants_h
#define Constants_h

 //Serial Baud Rate
const long BAUD_RATE = 115200;

//Pin Definitions
const byte PRESSURE = A0;   //Pressure Sensor
const byte POT1 = A2;       //BPM/Pmax Pot
const byte POT2 = A3;       //IE/Pmin Pot
const byte ALARM_LED = 2;   //alarm LED
const byte ENABLE = 3;      //enable switch
const byte PROG = 4;        //Prog button
const byte BUZZ = 5;        //piezo buzzer
const byte FAULT1 = 6;      //motor bridge fault 1
const byte FAULT2 = 7;      //motor bridge fault 2
const byte MOM = 8;         //PP/Clear button
const byte M1_P1 = 10;      //motor PWM 1
const byte M1_P2 = 9;       //motor PWM 2
const byte M1_EN = 11;      //motor enable pin
const byte LIM1 = 13;       //limit switch 1 (at full inspiration)
const byte LIM2 = 12;       //limit switch 2  (at full exhalation)

//Controller Modes

enum Modes {
  VENT_MODE,            // 0 (Ventilation Mode)
  BIPAP_MODE            // 1 (Breath Assist Mode)
};

// Controller States
enum States {

  //States used in Ventilator controller
  HOME_STATE,           // 0 (Homing Maneuver)
  INSP_STATE,           // 1 (Inspiratory Maneuver)
  EXP_STATE,            // 2 (Expiratory Maneuver)
  BARO_STATE,           // 3 (Barotrauma Avoidance Maneuver)
  PP_STATE,             // 4 (Inspiratory Hold Maneuver)
  HOLD_STATE,           // 5 (Expiratory Hold Maneuver)
  PROG_STATE,           // 6 (Programming State)
  DISABLED_STATE,       // 7 (Disabled State)

  //States used in BIPAP controller
  BIPAP_PROG_STATE,     // 8 (BiPAP Startup State)
  BIPAP_DISABLED_STATE, // 9 (BiPAP Disabled State)
  BIPAP_WAITING_STATE,  // 10 (BiPAP Breath Waiting State)
  BIPAP_ASSIST_STATE,   // 11 (BiPAP Breath Delivery State)
  BIPAP_BARO_STATE,      // 12 (BiPAP Barotrauma Avoidance Maneuver)
  BIPAP_HOME_STATE      // 13 (BiPAP Homing Maneuver)

};

const float DEBOUNCE_TIME = 20;   //Switch debounce time (in ms)

const float BARO_PRESSURE = 40;       //Maximum pressure for barotrauma warning
const float DISCONNECT_PRESSURE = 3;  //Minimum PIP pressure for disconnect alarm

const float MAX_SPEED = 255;      //Maximum motor speed (8-bit PWM)
const float MIN_SPEED = 60;       //Minimum motor speed (8-bit PWM)
const bool FORWARD = 0;               //Motor forward direction
const bool REV = 1;                   //Motor reverse direction

//BPM/IE Limits
const float BPM_LOW = 5;     //Minimum programmable BPM
const float BPM_HIGH = 56;   //Maximum programmable BPM
const float IE_LOW = 1;      //Minimum programmable I/E
const float IE_HIGH = 4;     //Maximum programmable I/E
const float ALPHA = 1.0;       //Correction Factor
const float BPM_MEM_SCALE = 5.0;    //Scale factor for storing BPM settings into 8bit memory
const float IE_MEM_SCALE = 60.0;    //Scale factor for storing IE settings into 8bit memory
const float P_MAX_MEM_SCALE = 6.0;  //Scale factor for storing Pmax settings into 8bit memory
const float P_MIN_MEM_SCALE = 6.0;  //Scale factor for storing Pmin settings into 8bit memory

//Timeout settings
unsigned long MOTOR_TIMEOUT_TIME = 5000; //Motor Timeout
unsigned long IDLE_TIMEOUT_TIME = 5000;   //Idle Timeout
unsigned long MAX_PAUSE = 5000;           //Max pause during discontinuous state

//EEPROM Addresses
const byte ADR1 = 1;             //address to store BPM settings
const byte ADR2 = 2;             //address to store IE settings
const byte ADR3 = 3;             //address to store max pressure threshold
const byte ADR4 = 4;             //address to score min pressure threshold

#endif
