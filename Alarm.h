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

/* NOTE: Portions of this code were inspired by the MIT e-vent alarm implementation
 *  which can be found here: https://e-vent.mit.edu/resources/downloads/
 */

/* Alarm
    DESCRIPTION: Defines a new Alarm class for managing alarm states and controlling 
    the alarm buzzer and LED based on 60601-standard alarm profiles
*/

#ifndef Alarm_h
#define Alarm_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#include "WConstants.h"
#endif

#include "Display.h"

using display::Display;

namespace alarms {

// Alarm levels in order of increasing priority
enum AlarmLevel {
  NO_PRIORITY,      //0
  MEDIUM_PRIORITY,  //1
  HIGH_PRIORITY     //2
};

/**
   Alarm
   Monitors the state and priority of a specific alarm.
*/

class Alarm {
  public:
    Alarm() {};

    Alarm(const char* text, const AlarmLevel& level, const bool& is_clearable, byte times_to_clear);

    // turn alarm on or off
    void setCondition(const bool& turn_on);

    // decrement 'times to clear' counter
    void clear_behavior();

    // reset alarm state
    void reset();

    // Check if this alarm is on
    inline const bool& isOn() const { return _on; }

    // If alarm can be cleared by a certain number of fault-free cycles (rather than user intervention)
    inline const bool& is_alarm_clearable() { return _is_clearable; }

    // Return alarm clear status
    inline byte& clear_status() {return _times_to_clear; }

    // Get the text of this alarm
    inline const char* text() const {
      return _text;
    }

    // Get the alarm level of this alarm
    inline AlarmLevel alarmLevel() const {
      return _alarm_level;
    }

  private:
    const char* _text;
    AlarmLevel _alarm_level;
    byte _times_to_clear;
    byte _initial_times_to_clear;
    bool _on = false;
    bool _is_clearable;
};

/**
   Alarm Manager
   Keeps track of all alarms and prioritizes them
   The display, LED and buzzer are updated according to the highest-
   priority alarm.
*/

class Alarm_Manager {

    enum Alarm_States {
      MOTOR_TIMEOUT_ALARM,  // 0 (Motor Timeout/Fault Alarm)
      BARO_ALARM,           // 1 (Barotrauma Warning Alarm)
      OVER_PRESSURE_ALARM,  // 2 (Over-Pressure Alarm on PIP)
      UNDER_PRESSURE_ALARM, // 3 (Under-Pressure Alarm on PIP)
      DISCONNECT_ALARM,     // 4 (Patient Disconnect Alarm)
      IDLE_TIMEOUT_ALARM,   // 5 (Motor Idle (Vent Mode) or Patient Idle (BiPAP Mode)
      NO_ALARM,
      NUM_ALARM
    };

  public:
    Alarm_Manager(const byte buzz_pin, const byte led_pin, Display* displ):
    _displ(displ),
    _buzz(buzz_pin),
    _led(led_pin) {
      _alarm_type[MOTOR_TIMEOUT_ALARM] = Alarm(" MOTOR TIMEOUT! ", HIGH_PRIORITY, false, 2);
      _alarm_type[BARO_ALARM] = Alarm(" BARO WARNING!  ", HIGH_PRIORITY, true, 2);
      _alarm_type[OVER_PRESSURE_ALARM] = Alarm(" OVER PRESSURE! ", HIGH_PRIORITY, true, 3);
      _alarm_type[UNDER_PRESSURE_ALARM] = Alarm("UNDER PRESSURE! ", HIGH_PRIORITY, true, 3);
      _alarm_type[IDLE_TIMEOUT_ALARM] = Alarm(" IDLE TIMEOUT!  ", MEDIUM_PRIORITY, false, 1);
      _alarm_type[DISCONNECT_ALARM] = Alarm("   DISCONNECT!  ", MEDIUM_PRIORITY, true, 3);
      _alarm_type[NO_ALARM] = Alarm("   No Alarms!   ", NO_PRIORITY, true, 1);
    }

    //Initialize Alarm_Manager object
    void initialize();

    //Update all alarms
    bool update();

    //Select alarm profile (Medium vs. High Priority)
    void alarm_profile(const AlarmLevel& priority);

    //Turn alarms off
    void alarm_off();

    //Set over-pressure alarm
    inline void set_over_pressure(const bool& onoff){
      _alarm_type[OVER_PRESSURE_ALARM].setCondition(onoff);
    }

    //Set under-pressure alarm
    inline void set_under_pressure(const bool& onoff){
      _alarm_type[UNDER_PRESSURE_ALARM].setCondition(onoff);
    }

    //Set barotraumatic warning alarm
    inline void set_baro_pressure(const bool& onoff){
      _alarm_type[BARO_ALARM].setCondition(onoff);
    }

    //Set motor timeout alarm
    inline void set_motor_timeout(const bool& onoff){
      _alarm_type[MOTOR_TIMEOUT_ALARM].setCondition(onoff);
    }

    //Set idle timeout alarm
    inline void set_idle_timeout(const bool& onoff){
      _alarm_type[IDLE_TIMEOUT_ALARM].setCondition(onoff);
    }

    //Set patient disconnect alarm
    inline void set_disconnect(const bool& onoff){
      _alarm_type[DISCONNECT_ALARM].setCondition(onoff);
    }

    //Get status of each alarm
    inline const bool& get_over_pressure()  {return _alarm_type[OVER_PRESSURE_ALARM].isOn();}
    inline const bool& get_under_pressure()  {return _alarm_type[UNDER_PRESSURE_ALARM].isOn();}
    inline const bool& get_baro_pressure()  {return _alarm_type[BARO_ALARM].isOn();}
    inline const bool& get_motor_timeout()  {return _alarm_type[MOTOR_TIMEOUT_ALARM].isOn();}
    inline const bool& get_idle_timeout()  {return _alarm_type[IDLE_TIMEOUT_ALARM].isOn();}
    inline const bool& get_disconnect()  {return _alarm_type[DISCONNECT_ALARM].isOn();}

    //Clear all alarms
    void clear_alarms();

    //Reset alarm states
    void reset_alarms();

    //Check if any alarms are present
    bool check_if_any_alarms();

    //Implement high-priority alarm scheme
    void high_priority_alarm();

    //Implement medium-priority alarm scheme
    void medium_priority_alarm();

    //Turn alarm LED and buzzer on or off
    void sound_Alarm(bool _onoff);

    //Toggle alarm state from off
    void toggle_alarms();

    //Check to see if alarms have been enabled
    bool is_alarm_enabled();

  private:
    Display* _displ;
    Alarm _alarm_type[NUM_ALARM];
    byte _buzz;
    byte _led;
    unsigned long _previousMillis = 0;
    unsigned long _currentMillis = 0;
    bool _alarms_off;
    byte _current_interval = 0;
    bool _entering_alarm = true;
    bool _alarms_present = false;

    //Alarm Parameters
    const int FREQ = 2200;     // Frequency of buzzer

    //High-priority burst profile (from 60601)
    const byte TD = 150;         // Buzzer pulse duration (high priority)
    const byte TS = 100;         // Spacing between pulses (high priority)
    unsigned long HP_ALARM_PROFILE[20] PROGMEM = { TD, TS, TD, TS, TD, 2 * TS + TD, TD, TS, TD, 1000, TD, TS, TD, TS, TD, 2 * TS + TD, TD, TS, TD, 2500 };
    const bool HP_ALARM_ONOFF[20] PROGMEM = { 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0 };
    const byte HP_NUM_OF_INTERVALS = sizeof(HP_ALARM_PROFILE) / sizeof(unsigned long);

    //Medium-priority burst profile (from 60601)
    const byte TD_M = 250;         // Buzzer pulse duration (medium priority)
    const byte TS_M = 200;         // Spacing between pulses (medium priority)
    unsigned long MP_ALARM_PROFILE[6] PROGMEM = { TD_M, TS_M, TD_M, TS_M, TD_M, 4000};
    const bool MP_ALARM_ONOFF[6] PROGMEM = { 1, 0, 1, 0, 1, 0 };
    const byte MP_NUM_OF_INTERVALS = sizeof(MP_ALARM_PROFILE) / sizeof(unsigned long);

};

}

#endif
