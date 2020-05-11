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
    DESCRIPTION: Defines a new Alarm class for controlling the alarm buzzer and LED
  based on 60601-standard alarm profiles
*/


#include "Alarm.h"

namespace alarms {

/**
   Alarm
   Monitors the state and priority of a specific alarm.
*/

Alarm::Alarm(const char* text, const AlarmLevel& level, const bool& is_clearable, byte times_to_clear):
  _text(text),
  _alarm_level(level),
  _initial_times_to_clear(times_to_clear),
  _times_to_clear(times_to_clear),
  _is_clearable(is_clearable)
  {}

void Alarm::setCondition(const bool& turn_on) {
  if (turn_on) {
    _on = true;
    reset();
  } else {
    _on = false;
    if(_is_clearable){
      clear_behavior();
    }
  }
}

//Decrement alarm clearance counter
void Alarm::clear_behavior() {
  --_times_to_clear;
  if (_times_to_clear < 0) {
    _times_to_clear = 0;
  }
}

//Reset times to clear
void Alarm::reset() {
  _times_to_clear = _initial_times_to_clear;
}

/**
   Alarm Manager
   Keeps track of all alarms and prioritizes them
   The display, LED and buzzer are updated according to the highest-
   priority alarm.
*/

void Alarm_Manager::initialize() {
  pinMode(_buzz, OUTPUT);
  pinMode(_led, OUTPUT);
  digitalWrite(_buzz, LOW);
  digitalWrite(_led, LOW);
  _current_interval = 0;
}

//Select alarm profile based on priority
void Alarm_Manager::alarm_profile(const AlarmLevel& priority) {
  if (_entering_alarm == true) {
    _previousMillis = millis();
    _entering_alarm = false;
  }
  switch (priority) {
    case HIGH_PRIORITY:
      high_priority_alarm();
      break;
    case MEDIUM_PRIORITY:
      medium_priority_alarm();
      break;
    default:
      alarm_off();
  }
}

//Turn buzzer/LED off
void Alarm_Manager::alarm_off() {
  sound_Alarm(false);
  _current_interval = 0;
  _entering_alarm = true;
}

//Turn buzzer/LED on
void Alarm_Manager::sound_Alarm(bool _onoff) {
  if (_onoff) {
    noTone(_buzz);
    tone(_buzz, FREQ);
    digitalWrite(_led, HIGH);
  }
  else {
    noTone(_buzz);
    digitalWrite(_led, LOW);
  }
}


//Clear all alarms
void Alarm_Manager::clear_alarms() {
  for (byte i = 0; i < NUM_ALARM; i++) {
    _alarm_type[i].setCondition(false);
    _alarm_type[i].reset();
  }
  alarm_off();
}

void Alarm_Manager::reset_alarms() {
  for (byte i = 0; i < NUM_ALARM; i++) {
    _alarm_type[i].setCondition(false);
    _alarm_type[i].reset();
  }
  alarm_off();
}

//Check if any alarms are present
bool Alarm_Manager::check_if_any_alarms() {
  return _alarms_present;
}

//Enable or disable alarms
void Alarm_Manager::toggle_alarms() {
  _alarms_off = !_alarms_off;
  if (_alarms_off) {
    _displ->writeString("ALARMS: DISABLED", "PP+PROG ENABLES");
    delay(1000);
  } else {
    _displ->writeString("ALARMS: ENABLED", "PP+PROG DISABLES");
    delay(1000);
  }
}

//Check to see if alarms have been enabled or disabled
bool Alarm_Manager::is_alarm_enabled() {
  return !_alarms_off;
}

bool Alarm_Manager::update() {
  _alarms_present = false;
  char* alarm_text = "None";
  AlarmLevel priority = NO_PRIORITY;

  if (!_alarms_off) {
    //Step through all alarms in order of priority
    for (byte i = 0; i < NUM_ALARM; i++) {
      if (_alarm_type[i].isOn() && _alarm_type[i].clear_status() > 0) {
        priority = _alarm_type[i].alarmLevel();
        alarm_text = _alarm_type[i].text();
        break;
      }
    }

    //Display highest-priority alarm
    if (priority > NO_PRIORITY) {
      alarm_profile(priority);
      _displ->writeString(alarm_text, "   PP TO CLEAR  ");
      _alarms_present =  true;
    }
  }

  return _alarms_present;
}

//The following implements the high-priority alarm scheme per 60601
void Alarm_Manager::high_priority_alarm() {
  _currentMillis = millis();
  if (_currentMillis - _previousMillis >= HP_ALARM_PROFILE[_current_interval]) {
    _current_interval += 1;
    if (_current_interval >= HP_NUM_OF_INTERVALS) {
      _current_interval = 0;
    }
    sound_Alarm(HP_ALARM_ONOFF[_current_interval]);
    _previousMillis = _currentMillis;
  }
}

//The following implements the medium-priority alarm scheme per 60601
void Alarm_Manager::medium_priority_alarm() {
  _currentMillis = millis();
  if (_currentMillis - _previousMillis >= MP_ALARM_PROFILE[_current_interval]) {
    _current_interval += 1;;
    if (_current_interval >= MP_NUM_OF_INTERVALS) {
      _current_interval = 0;
    }
    sound_Alarm(MP_ALARM_ONOFF[_current_interval]);
    _previousMillis = _currentMillis;
  }
}

}
