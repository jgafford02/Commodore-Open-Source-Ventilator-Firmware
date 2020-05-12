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

/*
  Commodore Open-Source Ventilator (COV) Firmware Code

  This code implements the state machines used to control the Commodore Open-Source
  Ventilator. More details here: https://vandyvents.com/

  External connections required:

    A0: Analog Pressure Sensor inputs
    A2: BPM/Pmax potentiometer input
    A3: IE/Pmin potentiometer input

    D2: Alarm LED output
    D3: Enable switch input
    D4: Program button input
    D5: Piezo buzzer output
    D6: Motor Fault 1 input
    D7: Motor Fault 2 input
    D8: PP/Clear button input
    D9: PWM 1 output
    D10: PWM 2 output
    D11: Motor enable output
    D12: Expiration limit switch input
    D13: Inspiration limit switch input
    SCL/SDA: LCD with I2C backpack

  Created 03/28/2020
  Updated 05/07/2020
  By Joshua Gafford

*/

//OTS Library includes
#include <EEPROM.h>
#include <Wire.h>
#include <ezButton.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

//Custom Library includes
#include "Constants.h"
#include "Utilities.h"
#include "Motor.h"
#include "Display.h"
#include "Pressure.h"
#include "SmartKnob.h"
#include "Alarm.h"

using namespace utils;

//Attach ezButton objects to all digital inputs
ezButton insp_limit(LIM1);
ezButton exp_limit(LIM2);
ezButton pp_button(MOM);
ezButton prog_button(PROG);
ezButton enable_switch(ENABLE);

//Attach SmartKnob objects to input potentiometers
SmartKnob pot1(POT1);
SmartKnob pot2(POT2);

//attach pressure object to pressure sensor
Pressure pressure(PRESSURE);

//attach Motor object to motor
Motor motor(M1_P1, M1_P2, M1_EN);

//Structure to hold all relevant respiration timing variables
typedef struct timing_state {
  unsigned long insp_time;        //Inspiration time
  unsigned long exp_time;         //Expiration Time
  unsigned long exp_time_des;     //Desired expiration time (for discontinuous mode)
  unsigned long insp_time_start;  //Inspiration cycle start
  unsigned long exp_time_start;   //Expiration cycle start
  float cycle_time;    //Total cycle time
  float BPM_meas;      //Measured BPM
  float IE_meas;       //Measure I/E
};

typedef struct timing_state T_state;
T_state t_state;

int insp_speed = 150;     //Motor speed on inspiration cycle (8-bit)
int exp_speed = 100;      //Motor speed on expiration cycle (8-bit)
int cycle_count = 0;      //Running count of number of respiratory cycles

//Ventilator Parameters
float BPM_des = 30.0;             //Default desired BPM
float IE_des = 2.0;               //Default desired I/E
float max_pressure = 30.0;        //Default desired max pressure threshold
float min_pressure = 10.0;        //Default desired min pressure threshold

unsigned long hold_time_start = 0;  //start timer for hold state (discontinuous mode)
bool hold_flag = false;             //triggers an expiration hold
bool startup_flag = true;           //toggles to false after startup sequence is executed

//BiPAP parameters
float idle_timeout = 10000;       //Stores idle timeout limit for BiPAP mode
unsigned long patient_timeout = 0;  
float trigger_pressure = -1.0;    //Stores breath trigger pressure for BiPAP mode
bool insp_limit_tripped = false; //Stores 'true' if ventilator successfully delivered a breath

//Set up display
hd44780_I2Cexp lcd;
display::Display displ(&lcd);

//Attach alarm object
alarms::Alarm_Manager alarm(BUZZ, ALARM_LED, &displ);
bool alarms_present = false;            //holds 'true' if there are any alarms present
boolean alarm_silent = false;         //Silence medium-priority alarms

//States
States state;
bool entering_state;
float tStateTimer;

//Modes (Ventilation vs. BiPAP)
Modes mode;
bool entering_mode;
bool mode_select = false;


void setup() {

  // Initialize displays, motor and alarm
  displ.begin();
  buttons_initialize();
  motor.initialize();
  alarm.initialize();

  // Load timer structs with initial conditions
  initialize_timer_settings(t_state);

  // Get mode (vent or BiPAP) on bootup based on status of Enable switch
  mode_select = enable_switch.getState();

  //Initialize serial communications
  Serial.begin(BAUD_RATE);
  delay(1000);
}

/*==================================================================================
  MAIN LOOP START
  ====================================================================================*/

void loop() {

  //Read sensors, buttons, knobs, and check for faults/alarms
  pressure.read();
  buttons_loop();
  knobs_loop();
  check_faults();
  handle_alarms();
  alarms_present = alarm.update();

  //Determine if in continuous or discontinuous mode
  hold_flag = update_speed();

  // run startup sequence on first execution and then put controller into standby state
  if (startup_flag == true) {
    load_from_memory();
    //Boot up into control mode
    if (mode_select) {
      setMode(VENT_MODE);
    }
    else {
      setMode(BIPAP_MODE);
    }

    //Bring out of startup mode
    startup_flag = false;
  }
  switch (mode) {
    case VENT_MODE:
      if (entering_mode == true) {

        //Display welcome message and load into VENT mode
        displ.writeVentStart();
        setState(DISABLED_STATE);
        entering_mode = false;
        delay(1000);
      }
      controller_run();   //Main ventilation control state machine
      break;

    case BIPAP_MODE:
      if (entering_mode == true) {

        //Display welcome message and load into BIPAP mode
        displ.writeBIPAPStart();
        setState(BIPAP_DISABLED_STATE);
        entering_mode = false;
        delay(1000);
      }
      breath_assist();    //Main breath assist state machine
      break;
  }
  print_serial();         //Print control status to serial
}

/*==================================================================================
  MAIN LOOP END
  ====================================================================================*/

/*==================================================================================
  MAIN VENTILATION CONTROLLER STATE MACHINE START
  ====================================================================================*/

void controller_run() {

  //Update display
  update_display_vent();

  switch (state) {

    /*Programming State
       This state allows the user to modify BPM/IE/Pmax/Pmin. Note: this state
       is inaccessible if there are alarms present - they must be cleared first.
    */
    case PROG_STATE:
      if (entering_state) {
        entering_state = false;
      }
      updateParams(prog_button.getState());
      if (pp_button.isPressed()) {
        write_to_memory();
        displ.clear();
        setState(DISABLED_STATE);
      }
      if (enable_switch.isPressed()) {
        setState(INSP_STATE);
      }
      break;

    /*Disabled State
       This state puts the ventilator into a standby state. Pressing the PROG
       button loads the programming screen. Pressing PROG and PP/Clear simultaneously
       enables or disables alarm reporting. Switching the Enable switch puts the
       ventilator into normal ventilation mode.
    */
    case DISABLED_STATE:
      if (entering_state) {
        cycle_count = 0;
        entering_state = false;
        alarm_silent = false;
        alarm.reset_alarms();
      }
      //User can disable all alarms by pressing 'PP/Clear' and 'PROG' at the same time
      if (!pp_button.getState() && prog_button.isPressed()) {
        alarm.toggle_alarms();
      }
      if (enable_switch.isPressed()) {
        setState(INSP_STATE);
      }
      if (prog_button.isPressed() && !alarms_present) {
        setState(PROG_STATE);
      }
      break;

    /*Inspiratory State
       This state executes an inspiratory maneuver under TV control. If the
       PIP exceeds the barotraumatic limit, the vent is immediately put into
       BARO_STATE. If the inspiration limit switch is tripped, the vent is
       (1) held at full inspiration if PP/Clear is pressed, or (2)immediately
       put into EXP_STATE if not. If the ENABLE switch is flipped, the
       vent is put into HOME_STATE.
    */
    case INSP_STATE:
      if (entering_state) {
        entering_state = false;
        pressure.reset();
        pressure.setPEEP();
        t_state.exp_time = millis() - t_state.exp_time_start;
        t_state.insp_time_start = millis();
        if (cycle_count == 0) {
          motor.softstart(insp_speed, FORWARD);
        }
        else
        {
          measure_cycle();    //Get measured BPM/IE
        }
      }
      if (pressure.get_max() > BARO_PRESSURE) {
        setState(BARO_STATE);
      }
      if (insp_limit.isPressed()) {
        pressure.setPlateau();
        delay(10);
        pressure.setPIP();
        motor.soft_transition(insp_speed, exp_speed, FORWARD);
        setState(EXP_STATE);
        if (pp_button.getState() == 0) {
          setState(PP_STATE);
        }
      }
      if (enable_switch.isReleased()) {
        setState(HOME_STATE);
      }
      //allow user to update speed dynamically
      if (!prog_button.getState() && !alarms_present) {
        updateParams(prog_button.getState());
      }
      break;

    /*Barotrauma Avoidance State
       This state immediately executes an expiratory motion if the inspiratory
       pressure exceeds the barotraumatic limit
    */
    case BARO_STATE:
      if (entering_state) {
        pressure.setPIP();
        entering_state = false;
        motor.softbrake(insp_speed, FORWARD);
        motor.softstart(exp_speed, REV);
        t_state.insp_time = millis() - t_state.insp_time_start;
        t_state.exp_time_start = millis();
      }
      if (exp_limit.isPressed()) {
        if (hold_flag) {
          setState(HOLD_STATE);
        }
        else {
          motor.softbrake(exp_speed, REV);
          motor.softstart(insp_speed, FORWARD);
          setState(INSP_STATE);
        }
      }
      break;

    /*Inspiratory Hold State
      This state implements an inspiratory hold at full inspiration for
      as long as the user holds down the PP/Clear button. The pressure
      status is printed on the LCD.
    */
    case PP_STATE:
      if (entering_state) {
        entering_state = false;
        motor.softbrake(insp_speed, FORWARD);
      }
      pressure.setPlateau();
      if (pp_button.isReleased()) {
        motor.softstart(exp_speed, FORWARD);
        setState(EXP_STATE);
      }
      break;

    /*Expiratory State
      This state sends the ventilator to full expiratory state. If the expiration
      limit switch is tripped, the vent is (1) sent to HOLD_STATE if hold_flag
      (discontinuous mode), or (2) sent to INSP_STATE if !hold_flag (continuous
      mode). If the ENABLE switch is flipped, the vent is put into HOME_STATE.
    */
    case EXP_STATE:
      if (entering_state) {
        entering_state = false;
        pressure.setPIP();
        t_state.insp_time = millis() - t_state.insp_time_start;
        t_state.exp_time_start = millis();
      }
      if (exp_limit.isPressed()) {
        cycle_count++;
        if (hold_flag) {
          setState(HOLD_STATE);
        }
        else {
          motor.soft_transition(exp_speed, insp_speed, FORWARD);
          setState(INSP_STATE);
        }
      }
      if (enable_switch.isReleased()) {
        setState(HOME_STATE);
      }
      //allow user to update speed dynamically
      if (!prog_button.getState() && !alarms_present) {
        updateParams(prog_button.getState());
      }
      break;

    /*Homing State
       This state causes the ventilator to complete its current respiratory
       cycle, and then enter the Disabled state the next time the expiration
       limit switch is triggered.
    */
    case HOME_STATE:
      if (entering_state == true) {
        cycle_count = 0;
        entering_state = false;
      }
      if (insp_limit.isPressed()) {
        motor.soft_transition(insp_speed, exp_speed, FORWARD);
      }
      else if (exp_limit.isPressed()) {
        motor.softbrake(exp_speed, FORWARD);
        setState(DISABLED_STATE);
      }
      break;

    /*Expiratory Hold State
      This state executes an expiratory hold if the commanded motor
      speed is below the recommended deadband speed (discontinuous mode).
      The motor stops here until the expiration time limit is exceeded,
      and the vent is put into INSP_STATE. If the ENABLE switch is flipped,
      the vent is put into DISABLED_STATE.
    */
    case HOLD_STATE:
      if (entering_state == true) {
        entering_state = false;
        t_state.exp_time = millis() - t_state.exp_time_start;
        motor.softbrake(exp_speed, FORWARD);
        t_state.exp_time_des = t_state.insp_time * IE_des;
      }
      if ((t_state.exp_time_des - t_state.exp_time) < 0.0) {
        setState(INSP_STATE);
      }
      else {
        //Calculate time to hold
        unsigned long hold_time = millis() - tStateTimer;
        unsigned long hold_time_requested = t_state.exp_time_des - t_state.exp_time;
        if (hold_time > hold_time_requested) {
          motor.softstart(insp_speed, FORWARD);
          setState(INSP_STATE);
        }
        if (enable_switch.isReleased()) {
          setState(DISABLED_STATE);
        }
      }
      break;
  }
}

/*==================================================================================
  MAIN VENTILATION CONTROLLER STATE MACHINE END
  ====================================================================================*/

/*==================================================================================
  MAIN BIPAP CONTROLLER STATE MACHINE START
  ====================================================================================*/

void breath_assist() {

  update_display_bipap();

  switch (state) {

    /*BiPAP Programming State
      The ventilator is in a standby state. User can use the two potentiometers
      to adjust the breath trigger pressure and idle timeout limit. Pressing
      'PP/Clear' puts the ventilator into Disabled state.
    */
    case BIPAP_PROG_STATE:
      if (entering_state == true) {
        entering_state = false;
      }
      updateParamsBIPAP();
      if (pp_button.isPressed()) {
        setState(BIPAP_DISABLED_STATE);
      }
      break;

    /*BiPAP Disabled State
      The ventilator is in a standby state. User can press PROG button
      to enter programming mode, or switch on Enable switch to activate
      breath-assist mode.
    */
    case BIPAP_DISABLED_STATE:
      if (entering_state == true) {
        entering_state = false;
        cycle_count = 0;
        alarm.reset_alarms();
      }
      //User can disable all alarms by pressing 'PP/Clear' and 'PROG' at the same time
      if (!pp_button.getState() && prog_button.isPressed()) {
        alarm.toggle_alarms();
      }
      if (enable_switch.isReleased()) {
        setState(BIPAP_WAITING_STATE);
      }
      if (prog_button.isPressed()) {
        setState(BIPAP_PROG_STATE);
      }
      break;

    /*BiPAP Waiting State
      This state continuously monitors the pressure. Once the pressure
      drops below the trigger pressure setting, a breath is delivered.
      If not breath is detected when an idle timeout occurs, deliver a 
      breath and sound an alarm. Switching the Enable switch disables 
      the ventilator.
    */
    case BIPAP_WAITING_STATE:
      if (entering_state) {
        entering_state = false;
        patient_timeout = millis();
      }

      //If a breath is requested, or if an idle timeout occurs, deliver one
      if ((pressure.get() < trigger_pressure)||(millis()-patient_timeout)>(idle_timeout+100)) {
        cycle_count++;
        setState(BIPAP_ASSIST_STATE);
      }

      //Disable the ventilator
      if (enable_switch.isPressed()) {
        setState(BIPAP_DISABLED_STATE);
      }
      break;

    /*BiPAP Breath State
      This state delivers a breath as fast as possible under TV control.
      If the pressure exceeds the barotraumatic limit, the direction is
      reversed immediately. Once the expiration limit switch is triggered,
      the ventilator goes back to BiPAP Waiting state.
    */
    case BIPAP_ASSIST_STATE:

      if (entering_state) {
        entering_state = false;
        insp_limit_tripped = false;
        pressure.reset();
        motor.softstart(MAX_SPEED, FORWARD);
      }

      //We've reached full inspiration
      if (!insp_limit.isPressed()) {
        insp_limit_tripped = true;
      }

      //We're at risk of inducing barotrauma
      if (pressure.get() > BARO_PRESSURE) {
        motor.softbrake(MAX_SPEED, FORWARD);
        setState(BIPAP_BARO_STATE);
      }

      //We've finished the cycle
      if (exp_limit.isPressed() && insp_limit_tripped) {
        motor.softbrake(MAX_SPEED, FORWARD);
        setState(BIPAP_WAITING_STATE);
      }

      if (enable_switch.isPressed()){
        setState(BIPAP_HOME_STATE);
      }
      break;


    /*BiPAP Barotrauma Avoidance State
       This state immediately executes an expiratory motion if the inspiratory
       pressure exceeds the barotraumatic limit.
    */
    case BIPAP_BARO_STATE:
      if (entering_state == true) {
        entering_state = false;
        motor.softstart(MAX_SPEED, REV);
      }
      if (exp_limit.isPressed()) {
        motor.softbrake(MAX_SPEED, REV);
        setState(BIPAP_WAITING_STATE);
      }
      break;


    /*BiPAP Homing State
       This state sends the ventilator to full expiratory position and
       then disables.
    */
    case BIPAP_HOME_STATE:
      if (entering_state == true) {
        cycle_count = 0;
        entering_state = false;
      }
      if (!exp_limit.getState()) {
        motor.softbrake(MAX_SPEED, FORWARD);
        setState(BIPAP_DISABLED_STATE);
      }
      break;
  }
}

/*==================================================================================
  MAIN BIPAP CONTROLLER STATE MACHINE END
  ====================================================================================*/

/*==================================================================================
  HELPER FUNCTIONS
  ====================================================================================*/

//Update LCD display depending on state (Vent mode)
void update_display_vent() {

  //Only update display if there are no alarms to show
  if (!alarms_present) {
    switch (state) {
      case DISABLED_STATE:
        displ.writeDisabled();
        break;
      case INSP_STATE:
        if (prog_button.getState()) {
          displ.writeNormalMode(cycle_count, pressure.get_PEEP(), t_state.BPM_meas, t_state.IE_meas);
        }
        break;
      case PP_STATE:
        displ.writeInspHold(pressure.get_Plateau(), pressure.get_PIP());
        break;
      case EXP_STATE:
        if (prog_button.getState()) {
          displ.writeNormalMode(cycle_count, pressure.get_PIP(), t_state.BPM_meas, t_state.IE_meas);
        }
        break;
      case HOME_STATE:
        displ.writeHome();
        break;
      case HOLD_STATE:
        displ.writeString("Expiratory Hold.", "DISABLE->Cancel");
        break;
    }
  }
}

//Update LCD display depending on state (BiPAP mode)
void update_display_bipap() {

  //Only update display if there are no alarms to show
  if (!alarms_present) {
    switch (state) {
      case BIPAP_DISABLED_STATE:
        displ.writeDisabled();
        break;
      case BIPAP_WAITING_STATE:
        displ.writeBreathWaiting(cycle_count, pressure.get(), (millis() - patient_timeout) / 1000.0);
        break;
      case BIPAP_ASSIST_STATE:
        displ.writeBreathRequest(pressure.get(), pressure.get_max());
        break;
      default:
        break;
    }
  }
}

//Load previous settings from EEPROM (on startup)
void load_from_memory() {

  //load previous parameter values (Scaled to 8-bit) from memory
  byte BPM_mem = EEPROM.read(ADR1);    //True BPM multiplied by 5;
  byte IE_mem = EEPROM.read(ADR2);     //True IE multiplied by 50;
  byte max_p_mem = EEPROM.read(ADR3);  //True pmax multiplied by 6;
  byte min_p_mem = EEPROM.read(ADR4);  //True pmin multiplied by 6;

  //if there's something there, load it and scale to true value
  if (BPM_mem > 0 && IE_mem > 0) {
    IE_des = IE_mem / IE_MEM_SCALE;
    BPM_des = BPM_mem / BPM_MEM_SCALE;
  }
  //if not, load defaults
  else {
    IE_des = 2.0;
    BPM_des = 30.0;
  }
  if (max_p_mem > 0 && min_p_mem > 0) {
    max_pressure = max_p_mem / P_MAX_MEM_SCALE;
    min_pressure = min_p_mem / P_MIN_MEM_SCALE;
  }
  else {
    max_pressure = 30.0;
    min_pressure = -1.0;
  }
}

//Write selected rate/pressure parameters to EEPROM
void write_to_memory() {
  EEPROM.write(ADR1, int(BPM_des * BPM_MEM_SCALE));           //store BPM as 8-bit int
  EEPROM.write(ADR2, int(IE_des * IE_MEM_SCALE));             //store IE as 8-bit int
  EEPROM.write(ADR3, int(max_pressure * P_MAX_MEM_SCALE));    //store max pressure as 8-bit int
  EEPROM.write(ADR4, int(min_pressure * P_MIN_MEM_SCALE));    //store min pressure as 8-bit int
  delay(100); //give time to write
}

//Prints ventilator status to serial for data logging
void print_serial() {
  float prog_time = millis() / 1000.0;

  //Print the following to use the serial plotter functionality
  switch (mode) {
    case VENT_MODE:
      Serial.print(prog_time);
      Serial.print(F(" "));
      Serial.print(cycle_count);
      Serial.print(F(" "));
      Serial.print(pressure.get());
      Serial.print(F(" "));
      Serial.print(t_state.BPM_meas);
      Serial.print(F(" "));
      Serial.println(t_state.IE_meas);
      break;
      
    case BIPAP_MODE:
      Serial.print(prog_time);
      Serial.print(F(" "));
      Serial.print(cycle_count);
      Serial.print(F(" "));
      Serial.println(pressure.get());
      break;
  }

}

//Check for various fault conditions
bool check_faults() {

  //Only check if alarms have been enabled
  if (alarm.is_alarm_enabled()) {

    //Check for motor or idle timeout faults
    bool motor_timeout_fault = motor_timeout_check();
    bool idle_timeout_fault = idle_timeout_check();

    //Carry over idle timeout if it triggered a breath during breath assist
    if ((state == BIPAP_ASSIST_STATE || state == BIPAP_WAITING_STATE ) && alarm.get_idle_timeout()){
      idle_timeout_fault = true;
    }
    alarm.set_motor_timeout(motor_timeout_fault);
    alarm.set_idle_timeout(idle_timeout_fault);

    //Check for barotrauma pressure limit
    if (entering_state && state == BARO_STATE) {
      alarm.set_baro_pressure(pressure.get() >= BARO_PRESSURE);
    }

    //Check for other pressure faults
    if (entering_state && state == EXP_STATE) {
      const bool over_pressure = pressure.get_PIP() > max_pressure;
      const bool under_pressure = ((pressure.get_Plateau() < min_pressure) && (pressure.get_PIP() > (pressure.get_PEEP() + 2.0)));
      const bool disconnect_pressure = pressure.get_PIP() < DISCONNECT_PRESSURE;
      alarm.set_over_pressure(over_pressure);
      alarm.set_under_pressure(under_pressure);
      alarm.set_disconnect(disconnect_pressure);
    }
  }
}

//Handling/clearing alarms
void handle_alarms() {
  //clear alarms if PP button is pressed
  if (pp_button.isPressed() && alarm.check_if_any_alarms()) {
    alarm.clear_alarms();

    //Zero max/min alarm pressures and motor/idle timeout
    tStateTimer = millis();
    alarm_silent = true;
  }
}

//Check to see if motor times out (motor fault) (returns true if so)
bool motor_timeout_check() {
  unsigned long elapsed_time = millis();
  bool timeout = false;
  //Motor timeouts only occur when motion is expected
  if (state == INSP_STATE || state == EXP_STATE || state == BIPAP_ASSIST_STATE) {
    if ((elapsed_time - tStateTimer) > MOTOR_TIMEOUT_TIME) {
      timeout = true;
    } else {
      timeout = false;
    }
  }
  return timeout;
}

//Check to see if an idle timeout fault occurs (returns true if so)
bool idle_timeout_check() {
  unsigned long elapsed_time = millis();
  bool timeout = false;
  //Idle timeout only occurs during Disabled states
  if (state == DISABLED_STATE || state == BIPAP_DISABLED_STATE) {
    if ((elapsed_time - tStateTimer) > IDLE_TIMEOUT_TIME) {
      timeout = true;
    } else {
      timeout = false;
    }
  }
  //Breath waiting timeout
  else if (state == BIPAP_WAITING_STATE) {
    if (elapsed_time - tStateTimer > idle_timeout) {
      timeout = true;
    } else {
      timeout = false;
    }
  }
  return timeout;
}

//Set new control mode
void setMode(Modes newMode) {
  entering_mode = true;
  mode = newMode;
}

//Set new control state
void setState(States newState) {
  entering_state = true;
  state = newState;
  tStateTimer = millis();
}

//Update BIPAP parameters during programming state
void updateParamsBIPAP() {
  trigger_pressure = map2(pot1.get(), 0.0, 1024.0, 5.0, -5.0);        //Read max pressure pot and map to range
  idle_timeout = map2(pot2.get(), 0.0, 1024.0, 100000.0, 1000.0);     //Read timeout pot and map to range
  displ.writeProgModeBIPAP(trigger_pressure, idle_timeout / 1000.0);  //update display
}

//Update parameters during programming state
void updateParams(boolean param_set) {
  bool change_check = false;
  if (param_set) {

    //Update parameters if potentiometer position has changed
    if (pot1.hasChanged()) {
      max_pressure = map2(pot1.get(), 0.0, 1024.0, 40.0, 20.0); //Read max pressure pot
      change_check = true;
    }
    if (pot2.hasChanged()) {
      min_pressure = map2(pot2.get(), 0.0, 1024.0, 20.0, -5.0); //Read min pressure pot
      change_check = true;
    }

    //Don't let user select too small of a maximum threshold
    if (max_pressure < (min_pressure + 1.0)) {
      max_pressure = min_pressure + 1.0;
    }
    displ.writeProgModePressure(max_pressure, min_pressure);  //update display
  }
  else {

    //Update parameters if potentiometer position has changed
    if (pot1.hasChanged() || pot2.hasChanged()) {
      update_rate(pot1.get(), pot2.get());
      change_check = true;
    }
    displ.writeProgModeRate(BPM_des, IE_des);  //update display
    if (change_check == true) {
      tStateTimer = millis();
    }
  }
}

//Calculate BPM/IE from potentiometer values
void update_rate(int pot1, int pot2) {
  IE_des = getParam10bit(pot2, IE_HIGH, IE_LOW);
  BPM_des = getBPM10bit(pot1, IE_des, BPM_HIGH, BPM_LOW);
  update_speed();
}

//Calculate inspiratory/expiratory speed (8-bit) based on BPM/IE settings
boolean update_speed() {
  boolean hold_flag = false;
  float t_max = 60.0 / (2.0 * BPM_HIGH);                      //maximum cycle semi-period
  float BPM_MAX_AT_IE_DES = 60.0 / (t_max + IE_des * t_max);  //maximum BPM at current IE setting

  //calculate motor speeds and scale to 0-255
  insp_speed = ((BPM_des) / BPM_MAX_AT_IE_DES) * (MAX_SPEED - MIN_SPEED) + MIN_SPEED;
  exp_speed = insp_speed / ((1.0 + 0.03 * (IE_des - 1.0)) * IE_des);

  //If requested expiration speed is below motor's deadband, set hold_flag
  //to true to put ventilator into discontinuous mode
  if (exp_speed < MIN_SPEED) {
    hold_flag = true;
    exp_speed = MIN_SPEED;
  } else {
    hold_flag = false;
  }
  return hold_flag;
}

//Measures cycle time, BPM and IE
void measure_cycle() {
  t_state.cycle_time = (t_state.insp_time + t_state.exp_time);
  t_state.BPM_meas = 60.0 / (t_state.cycle_time / 1000.0);
  t_state.IE_meas = float(t_state.exp_time) / float(t_state.insp_time);
}

//SmartKnob loop function
void knobs_loop() {
  pot1.loop();
  pot2.loop();
}

//Initialize ezButton objects
void buttons_initialize() {

  //Set debounce time and counting mode
  insp_limit.setDebounceTime(DEBOUNCE_TIME);
  exp_limit.setDebounceTime(DEBOUNCE_TIME);
  pp_button.setDebounceTime(DEBOUNCE_TIME);
  prog_button.setDebounceTime(DEBOUNCE_TIME);
  enable_switch.setDebounceTime(DEBOUNCE_TIME);

  insp_limit.setCountMode(COUNT_FALLING);
  exp_limit.setCountMode(COUNT_FALLING);
  pp_button.setCountMode(COUNT_FALLING);
  prog_button.setCountMode(COUNT_FALLING);
  enable_switch.setCountMode(COUNT_FALLING);
}

void buttons_loop() {
  insp_limit.loop();
  exp_limit.loop();
  pp_button.loop();
  prog_button.loop();
  enable_switch.loop();
}

//Initialize timing variable structure
void initialize_timer_settings(T_state & t_state) {
  //Timing state initialization
  t_state.insp_time = 1.0;        //Inspiration time
  t_state.exp_time = 1.0;         //Expiration Time
  t_state.exp_time_des = 1.0;     //Desired expiration time (for discontinuous mode)
  t_state.insp_time_start = 0;  //Inspiration cycle start
  t_state.exp_time_start = 0;   //Expiration cycle start
  t_state.cycle_time = 0;    //Total cycle time
  t_state.BPM_meas = 0.0;      //Measured BPM
  t_state.IE_meas = 0.0;       //Measure I/E

}
