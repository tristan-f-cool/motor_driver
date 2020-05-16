/**********************************************
 * PRECISION MICRODRIVES
 * Gearmotor Driver
 * Tristan Cool
 * April 2020
 *********************************************/

 /********************************************
  * RELEASE / CHANGE RECORD
  * v1.0 - program creation (TC) 11/04/2020
  * v1.0 - uni-directional drive with NPN transistor, emergency stop and PWM/Speed control (TC) 15/04/2020
  * v1.1 - uni-directional drive with NPN transistor, emergency stop, PWM/Speed control and endstops (TC) 18/04/2020
  * v1.2 - bidirection H-Bridge transitor drive with PWM/Speed control, emergency stop, and endstops (TC) 19/04/2020
  * v1.3 - bidirection H-Bridge transitor drive with PWM/Speed control, emergency stop, and endstops, DEVELOPER mode and Menu (TC) 20/04/2020
  * v1.4 - bidirection H-Bridge transistor drive with PWM/Speed control, emergency stop, endstop, encoder count  (TC) 25/04/2020
  */
  
//******************************************* LIBRARIES ******************************************************************//
#include "defs.h"
#include "motor.h"
#include "limit_switch.h"
#include "encoder.h"

//******************************************* VARIABLES *****************************************************************//

// SLIDER & POTENTIOMETER
int slider_val;
int SLIDER_MODE = SLIDER_POSITION;              //(SLIDER_DISABLED = 0 ; SLIDER_POSITION = 1)
int SLIDER_PREV;

int potentiometer_val;
int POTENTIOMETER_MODE = POTENTIOMETER_PWM;     //(POTENTIOMETER_DISABLED = 0 ; POTENTIOMETER_PWM = 1)
int POTENTIOMETER_PREV;

// BUTTONS
long button_time = 0;
long button_debounce = 100;   

byte button_stop_go_state;
byte button_stop_go_prev_state = HIGH;

byte button_reset_state;
byte button_reset_prev_state = HIGH;

// OPERATION MODE
int CONTROL = MANUAL;                           //(MANUAL = 0 ; AUTO = 1 ; PROFILE = 2)
int MOTION = TRAPEZOID;                         //(TRIANGLE = 0 ; TRAPEZOID = 1 ; S = 2)
bool emergency_stop = false;
bool restart = false;
bool reset = false;

// LOAD & POSITION
int cycle_count = 0;
int CYCLES = MAX_CYCLES;        //-Default (3)
bool cycle_complete = false;

bool myOUTPUT = LOAD;
int command_position = 0;
int START = START_POSITION;     //-Default (75)
bool position_reached = false;
bool forward = true;

// MOTOR
Motor motor = Motor();

// ENCODER
Encoder encoder = Encoder();

// LIMIT SWITCHES
Limit_Switch home_limit = Limit_Switch();
Limit_Switch end_limit  = Limit_Switch();

// PROFILE 
//(y = PWM ; x = Position) (y=ax+b)

// Triangle profile (phase I, II)
// Accel (I):  y = (MAX_DUTY/(MAX_POS/2)x = 2*MAX_DUTY/MAX_POS
// Deccel(II): y = (-MAX_DUTY/(MAX_POS/2)x + 2*MAX_DUTY = -2*MAX_DUTY/MAX_POS + 2*MAX_DUTY
int tri_a1 = 2*MAX_DUTY/MAX_POSITION;
int tri_a2 = -tri_a1;
int tri_b2 = 2*MAX_DUTY;

// Trapezoid profile (phase I,II,III)
// Accel (I):  y = (MAX_DUTY/(MAX_POS/3)x = 3*MAX_DUTY/MAX_POS
// Deccel(II): y = (-MAX_DUTY/(MAX_POS/3)x + 3*MAX_DUTY = -3*MAX_DUTY/MAX_POS + 3*MAX_DUTY
int trap_a1 = 3*MAX_DUTY/MAX_POSITION;
int trap_a3 = -trap_a1;
int trap_b3 = 3*MAX_DUTY;

//############################################ SETUP #################################################################//
void setup() 
{
  // - Setup serial
  Serial.begin( 9600 );
  
  // - Title 
  Serial.println(F("##############################################"));
  Serial.println(F("~~~~~~~~~ PRECISION MICRODRIVES ~~~~~~~~~~~~~~"));
  Serial.println(F("*****        Gearmotor Driver           ******"));
  Serial.println(F("##############################################"));
  Serial.println();
  delay(1000);

  // - Slider & Potentiometer - Position / PWM
  pinMode(potentiometer1_pin, INPUT);
  pinMode(potentiometer2_pin, INPUT);
  
  // - Buttons - 1:STOP/GO ; 2:RESET
  pinMode(button1_pin, INPUT_PULLUP);
  pinMode(button2_pin, INPUT_PULLUP);     

  // - Operation Mode
  mode(CONTROL);

  // - Motor
  
  // - Encoder
  attachInterrupt(digitalPinToInterrupt(encoderA_pin), read_counts, CHANGE);
  
  // - Load
  
  // - Profile
    
  // - Menu set paramters
  if(MENU)
    menu_setting_config();

  // - 5 second safety delay
  warning_count();
  
  // - Home load and drive to start position
  drive_to_start();
  
}//END: setup

//################################################ LOOP ##############################################################//
void loop() 
{
  if(DEBUG)
    debug();

  else{
  // - Input Polling (Interrupts) -
  stop_go_button();
  reset_button();
  if(SLIDER_MODE != SLIDER_DISABLED)
    slider();
  if(POTENTIOMETER_MODE != POTENTIOMETER_DISABLED)
    potentiometer();
  
  // - Motor restart after emergency stop
  if(restart)
  {
    warning_count();
    motor.restart();
    restart = false;
  }
  // - Motor normal operation
  if(!emergency_stop)
  {
    switch(CONTROL)
    {
      //MANUAL (0)
      case MANUAL:
        mode_MANUAL();
        break;
      //PROFILE (2)
      case PROFILE:
        mode_PROFILE();
        break;
      //DEVELOPER (9)
      case DEVELOPER:
        mode_DEVELOPER();
        break;
      //AUTO (1) {DEFAULT}
      case AUTO:
      default:
        mode_AUTO();
        break;
    }
  }
  // - Motor emergency stop
  else
  {
    motor.brake();
  }
  }
}//END: loop

//************************************************* FUNCTIONS **********************************************************//

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ DEBUG & TEST ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

// #Debug test function
void debug()
{
  motor.drive_forward();
  delay(3000);
  motor.brake();
  delay(3000);
  motor.drive_reverse();
  delay(3000);
  motor.brake();
  delay(3000);
}//END: debug

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ INPUT POLLING ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

// #Button: Emergency Stop / Restart
void stop_go_button()
{
  if(emergency_stop)
    Serial.println(F("Push START/STOP to resume operation"));
    
  button_stop_go_state = digitalRead(button1_pin);

  if(!button_stop_go_state && button_stop_go_prev_state && (millis() - button_time > button_debounce))
  {
    if(emergency_stop)
    {
      Serial.println(F("- - - - - - - - - - - - - - - - - - - - - - - -"));
      Serial.println("                 ##! RESTART !##");
      Serial.println(F("- - - - - - - - - - - - - - - - - - - - - - - -"));
      Serial.println();
      restart = true;
      emergency_stop = false;
      //Resume previous mode settings and values
      SLIDER_MODE = SLIDER_PREV;
      POTENTIOMETER_MODE = POTENTIOMETER_PREV;
    }
    else
    {
      Serial.println(F("- - - - - - - - - - - - - - - - - - - - - - - -"));
      Serial.println("             ##! EMERGENCY STOP !##");
      Serial.println(F("- - - - - - - - - - - - - - - - - - - - - - - -"));
      Serial.println();
      emergency_stop = true;
      motor.prev_pwm_duty         = motor.get_duty();
      //Save current mode settings and values
      SLIDER_PREV = SLIDER_MODE;
      SLIDER_MODE = SLIDER_DISABLED;
      POTENTIOMETER_PREV = POTENTIOMETER_MODE;
      POTENTIOMETER_MODE = POTENTIOMETER_DISABLED;
    }
  }
  button_stop_go_prev_state = button_stop_go_state;
}//END: stop_go_button

// #Button: Reset position
void reset_button()
{
  if(reset)
    Serial.println(F("Push RESET to resume operation"));
    
  button_reset_state = digitalRead(button2_pin);
  
  if(!button_reset_state && button_reset_prev_state && (millis() - button_time > button_debounce))
  {
    if(reset)
    {
      reset = false;
      emergency_stop = false;
      //Resume previous mode settings and values
      SLIDER_MODE = SLIDER_PREV;
      POTENTIOMETER_MODE = POTENTIOMETER_PREV;
    }
    else
    {
      Serial.println(F("- - - - - - - - - - - - - - - - - - - - - - - -"));
      Serial.println("                 ##! RESET !##");
      Serial.println(F("- - - - - - - - - - - - - - - - - - - - - - - -"));
      Serial.println();
      motor.brake();
      delay(2000);
      drive_to_start();
      reset = true;
      emergency_stop = true;
      //Resume previous mode settings and values
      SLIDER_MODE = SLIDER_PREV;
      POTENTIOMETER_MODE = POTENTIOMETER_PREV;
    }
  }
  button_reset_prev_state = button_reset_state;
}//END: reset_button

// #Slider: Potentiometer control of position
void slider()
{
  slider_val = analogRead(potentiometer1_pin);
  command_position = map(slider_val,0,1023,MIN_POSITION,MAX_POSITION);
}//END: slider

// #Potentiometer: Potentiometer control of PWM
void potentiometer()
{
  potentiometer_val = analogRead(potentiometer2_pin);
  motor.set_duty(map(potentiometer_val,0,1023,motor.min_duty,motor.max_duty));
}//END: potentiometer

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ISR INTERRUPT ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// #Read Counts (Encoder)
void read_counts()
{
  encoder.stateA  = digitalRead(encoderA_pin);
  encoder.stateB  = digitalRead(encoderB_pin);

  if(encoder.stateA == HIGH)
  {
    if(encoder.stateB == LOW)
    {
      encoder.count++;
      encoder.rotation = FORWARD;
      Serial.println("aaaaaaaaaa");
    }
    else
    {
      encoder.count--;
      encoder.rotation = REVERSE;
      Serial.println("bbbbbbbb");
    }
  }
  else
  {
    if(encoder.stateB == LOW)
    {
      encoder.count--;
      encoder.rotation = REVERSE;
      Serial.println("cccccccccc");
    }
  }
Serial.println(encoder.count);
  
}//END: read_counts

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ MODE  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

// #Mode: configure slider and potentiometer based on operation mode
void mode(int mode)
{
  CONTROL = mode;
  switch(CONTROL)
  {
    //Open-Loop: load drives to set variable position (or end-stop) with variable speed (PWM)
    case MANUAL:
      SLIDER_MODE= SLIDER_POSITION; 
      POTENTIOMETER_MODE = POTENTIOMETER_PWM;
      break;
    //Motion Profile: load drives to end and back to home following a preset motion profile
    case PROFILE:
      SLIDER_MODE= SLIDER_DISABLED; 
      POTENTIOMETER_MODE = POTENTIOMETER_DISABLED;
      break;
    //Developer: user customised program functionality and motion
    case DEVELOPER:
      SLIDER_MODE= SLIDER_DISABLED; 
      POTENTIOMETER_MODE = POTENTIOMETER_DISABLED;
      break;
    //Closed-Loop: load drives to preset position (or end-stop) with set/variable speed (PWM)
    case AUTO:
    default:
      SLIDER_MODE= SLIDER_DISABLED; 
      POTENTIOMETER_MODE = POTENTIOMETER_PWM;
      break;
  }
  Serial.println();
}//END: mode

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  SETTINGS  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

// #Set Operation Mode
void set_operation_mode(String parameter)
{
  if(parameter == "0")
    CONTROL = MANUAL;
  else if(parameter == "1")
    CONTROL = AUTO;
  else if(parameter == "2")
    CONTROL = PROFILE;
  else if(parameter == "9")
    CONTROL = DEVELOPER;
  else
  {
    Serial.println(F("Invalid MODE....restarting menu"));
    menu_setting_config();  
  }
  mode(CONTROL);
}//END: set_operating_mode

// #Set Motion Profile
void set_motion_profile(String parameter)
{
  if(parameter == "0")
    MOTION = TRIANGLE;
  else if(parameter == "1")
    MOTION = TRAPEZOID;
  else if(parameter == "2")
    MOTION = S;
  else
  {
    Serial.println(F("Invalid PROFILE....restarting menu"));
    menu_setting_config();  
  }
  motor.motion_profile = parameter.toInt();
}//END: set_motion_profile

// #Set Cycles
void set_cycle(String parameter)
{
  CYCLES = parameter.toInt();
  if(CYCLES == 0)
  {
    Serial.println(F("Invalid CYCLE count...cycle count set to default value"));
    CYCLES = MAX_CYCLES;
  }
  else if(CYCLES < 0)
  {
    Serial.println(F("Invalid CYCLE count...cycle count cannot be negative"));
    CYCLES = abs(CYCLES);
  }
  else
  {
    Serial.println();
    Serial.println(F("~~~~~~~~~~~~~~ CYCLE ~~~~~~~~~~~~~~~~~~~"));
    Serial.print(F("CYCLE count set to: "));
    Serial.println(CYCLES);
    Serial.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    Serial.println();
  }
}//END: set_cycles

// #Set Start Position
void set_start_position(String parameter)
{
  START = parameter.toInt();
  if(START < 0)
  {
    Serial.println(F("Invalid START position...start position cannot be less than 0"));
    START = START_POSITION;
  }
  else if(START > 160)
  {
    Serial.println(F("Invalid START position...start position cannot be over than 160"));
    START = START_POSITION;
  }
  else
  {
    Serial.println();
    Serial.println(F("~~~~~~~~~~~~~~ START ~~~~~~~~~~~~~~~~~~~"));
    Serial.print(F("START position set to (mm): "));
    Serial.println(START);
    Serial.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    Serial.println();   
  }
}//END: set_start_position

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ PRINT ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

// #Print: Operation Mode
void print_operation_mode()
{
  Serial.println();
  Serial.println(F("~~~~~~~~~~~~ OPERATION ~~~~~~~~~~~~~~~~~"));
  Serial.println(F("(MANUAL = 0 ; AUTO = 1 ; PROFILE = 2 ; DEVELOPER = 9)"));
  Serial.print(F("~Mode: "));
  Serial.println(CONTROL);
  Serial.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
  Serial.println();
}//END: print_mode

// #Print: Motion Profile
void print_motion_profile()
{
  Serial.println();
  Serial.println(F("~~~~~~~~~~ MOTION PROFILE ~~~~~~~~~~~~~~"));
  Serial.println(F("(TRIANGLE = 0 ; TRAPEZOID = 1 ; S = 2)"));
  Serial.print(F("~Motion Profile: "));
  Serial.println(MOTION);
  Serial.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
  Serial.println();
}//END: print_motion_profile

// #Print: Slider Mode
void print_slider_mode()
{
  Serial.println();
  Serial.println(F("~~~~~~~~~~~~~~ SLIDER ~~~~~~~~~~~~~~~~~~"));
  Serial.println(F("(SLIDER_DISABLED = 0 ; SLIDER_POSITION = 1)"));
  Serial.print(F("~Slider: "));
  Serial.println(SLIDER_MODE);
  Serial.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
  Serial.println();
}//END: print_slider_mode

// #Print: Potentiometer Mode
void print_potentiometer_mode()
{
  Serial.println();
  Serial.println(F("~~~~~~~~~~~ POTENTIOMETER ~~~~~~~~~~~~~~~~"));
  Serial.println(F("(POTENTIOMETER_DISABLED = 0 ; POTENTIOMETER_PWM = 1)"));
  Serial.print(F("~Potentiometer: "));
  Serial.println(POTENTIOMETER_MODE);
  Serial.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
  Serial.println();
}//END: print_potentiometer_mode

// #Print: Potentiometer Mode
void print_cycle()
{
  //cycle start
  if(cycle_count == 0)
  {
    Serial.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    Serial.print(F("~Cycle START: "));
    Serial.print(CYCLES);
    Serial.println(F(" cycles"));
    Serial.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    Serial.println();
  }
  //cycle end
  else if(cycle_complete)
  {
    //TODO
    Serial.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    Serial.print(F("~Cycle END: "));
    Serial.print(CYCLES);
    Serial.println(F(" cycles"));
    Serial.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    Serial.println();
  }
  //current cycle 
  else
  {
    Serial.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    Serial.print(F("~Cycle "));
    Serial.print(cycle_count);
    Serial.print(F(" of "));
    Serial.print(CYCLES);
    Serial.println(F(" cycles"));
    Serial.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    Serial.println();
  }
}//END: print_cycle

// #Print position
void print_position()
{
    Serial.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    Serial.print(F("~Commanded Position (mm) "));
    Serial.println(command_position);
    Serial.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    Serial.println();
}//END: print_position

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ MENU ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

// #User Input
String user_input(String question)
{
  Serial.println(question);
  while(!Serial.available()) 
  {
    // wait for input
  }
  return Serial.readStringUntil(10);
}//END: user_input

// #Menu for user-setting
void menu_setting_config()
{
  String setting;
  String question;
  
  Serial.println(F("- - - - - - - - - - - - - - - - - - - - - - - -"));
  Serial.println("                 ##! MENU !##                    ");
  Serial.println(F("- - - - - - - - - - - - - - - - - - - - - - - -"));
  Serial.println();
  
  // - Operation Mode
  Serial.println(F("###########################################################"));
  question = "Set Operation Mode: (0=MANUAL, 1=AUTO, 2=PROFILE, 9=DEVELOPER)";
  setting = user_input(question);
  set_operation_mode(setting);
  print_operation_mode();
  delay(3000);

  if(CONTROL == PROFILE)
  {
    Serial.println(F("###########################################################"));
    question = "Set Motion Profile: (0=TRIANGLE, 1=TRAPEZOID, 2=S)";
    setting = user_input(question);
    set_motion_profile(setting);
    print_motion_profile();
  }
  else if(CONTROL == AUTO)
  {
    Serial.println(F("###########################################################"));
    question = "Set number of Cycles to complete: ";
    setting = user_input(question);
    set_cycle(setting);
  }
  else if(CONTROL == AUTO || CONTROL == MANUAL)
  {
    Serial.println(F("###########################################################"));
    question = "Set Start Position (mm): ";
    setting = user_input(question);
    set_start_position(setting);
  }
  
  // - Slider Mode
  print_slider_mode();
  delay(3000);

  // - Potentiometer Mode
  print_potentiometer_mode();
  delay(3000);
  
  // - Motor Settings
  motor.print_motor_settings();
  delay(3000);

  // - Encoder Settings
  encoder.init(MAGNETIC,CPR,DIAMETER,encoderA_pin,encoderB_pin);
  encoder.print_encoder();
  delay(3000);

  // - Load
  
  // - Limit Switch
  home_limit.init(limit_switch1_pin,OPTICAL,HOME);
  end_limit.init(limit_switch2_pin,MECHANICAL,ENDSTOP);
  home_limit.print_limit_switch();
  end_limit.print_limit_switch();
  
}//END: menu_setting_config

// #5 second safety delay
void warning_count()
{
  Serial.println(F("- - - - - - - - - - - - - - - - - - - - - - - -"));
  Serial.println(F("                !! CAUTION !!"));
  Serial.println(F("Motor motion in "));
  Serial.print(F("5..."));
  delay(1000);
  Serial.print(F("4..."));
  delay(1000);
  Serial.print(F("3..."));
  delay(1000);
  Serial.print(F("2..."));
  delay(1000);
  Serial.print(F("1..."));
  Serial.println();
  Serial.println(F("- - - - - - - - - - - - - - - - - - - - - - - -"));
  Serial.println();
  delay(1000);  
}//END: warning count

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ DRIVE ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

// #Drive Speed
void drive_speed(bool output, bool rotation, int rpm)
{
   //TODO
   if(rotation == REVERSE && motor.driver == UNIDIRECTIONAL)
    Serial.println(F("!~ Incorrect setting: Reverse drive disabled"));
   if(output == LOAD && rpm > motor.motor_settings.max_rated_load_speed)
    Serial.println(F("!~ Incorrect setting: Load Speed too high"));
   if(output == NO_LOAD && rpm > motor.motor_settings.max_no_load_speed)
    Serial.println(F("!~ Incorrect setting: No Load Speed too high"));
}//END: drive_speed

// #Drive By: drive motor by a milimeter value
void drive_mm(bool output, bool rotation,int mm)
{
  //TODO
   if(rotation == REVERSE && motor.driver == UNIDIRECTIONAL)
    Serial.println(F("!~ Incorrect setting: Reverse drive disabled"));
}//END: drive_by

// #Drive Rotation: drive motor by number of output rotations
void drive_rotation(bool output, bool rotation, int rot)
{
  //TODO
   if(rotation == REVERSE && motor.driver == UNIDIRECTIONAL)
    Serial.println(F("!~ Incorrect setting: Reverse drive disabled"));
}//END: drive_rotation

// #Drive  Degree: drive motor  by a degree value
void drive_degree(bool output, bool rotation,int deg)
{
  //TODO
   if(rotation == REVERSE && motor.driver == UNIDIRECTIONAL)
    Serial.println(F("!~ Incorrect setting: Reverse drive disabled"));
}//END: drive_degree

// #Drive to Position: drive motor to commanded position
void drive_to_position(int pos)
{
  //TODO
}//END: drive_to_position

// #Drive to Start: drive motor to initial start position
void drive_to_start()
{
  //TODO
  Serial.println(F("- - - - - - - - - - - - - - - - - - - - - - - -"));
  Serial.println(F("           !! POSITION RESET !!"));
  // - set homing speed to 100%
  motor.prev_pwm_duty = motor.get_duty();
  motor.set_duty(MAX_DUTY);
  // - drive motor to home for position reference
  drive_to_home();
  motor.brake();
  delay(2000);
  // - reset PWM duty
  motor.set_duty(motor.prev_pwm_duty);
  //PROFILE mode begins at HOME, other modes can begin elsewhere
  if(CONTROL == AUTO || CONTROL == MANUAL || CONTROL == DEVELOPER)
  {
    // - drive motor to start position
    //drive_to_position(START);
    //Serial.println(F("START position reached"));
  }
}//END: drive_to_start

// #Drive to Endstop: drive motor to end mechanical limit switch
void drive_to_endstop()
{
  Serial.println(F("DRIVE TO END"));
  while(!end_limit.limit_reached())
  {
    if(motor.driver == UNIDIRECTIONAL)
    {
      motor.drive();
    }
    else if(motor.driver == BIDIRECTIONAL)
    {
      motor.drive_forward();
    }
  }
  Serial.println(F("ENDSTOP reached"));
  motor.brake();
}//END: drive_to_endstop

// #Drive to Home: drive motor to home optical limit switch
void drive_to_home()
{
  Serial.println(F("DRIVE TO HOME"));
  while(home_limit.limit_reached())
  {
    if(motor.driver == UNIDIRECTIONAL)
    {
      motor.drive();
    }
    else if(motor.driver == BIDIRECTIONAL)
    {
      motor.drive_reverse();
    }
  }
  Serial.println(F("HOME reached"));
  motor.brake();
  encoder.set_count(0);
}//END: drive_to_home

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ PROFILE ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

// #TRIANGLE motion profile: 1/2 accel ; 1/2 decel
void triangle()
{
  //TODO
  int y; // PWM duty
  int x; // = current position 
  //Phase I:
//  if( current_position < MAX_POSITION/2 && forward)
//  {
//    y = tri_a1 * current_position;
//  }
//  else if( current_position < MAX_POSITION/2 && !forward)
//  {
//    y = tri_a2 * current_position + tri_b2;
//  }
//  //Phase II:
//  if( current_position > MAX_POSITION/2 && forward)
//  {
//    y = tri_a2 * current_position + tri_b2;
//  }
//  else if ( current_position > MAX_POSITION/2 && !forward)
//  {
//    y = tri_a1 * current_position;
//  }
//  motor.set_duty(y);
}//END: triangle

// #TRAPEZOID motion profile: 1/3 accel ; 1/3 max ; 1/3 decel
void trapezoid()
{
  //TODO
  int y; // PWM duty
  int x; // = current position 
//  //Phase I:
//  if( current_position < MAX_POSITION/3 && forward)
//  {
//    y = trap_a1 * current_position;
//  }
//  else if( current_position < MAX_POSITION/3 && !forward)
//  {
//    y = trap_a3 * current_position + trap_b3;
//  }
//  //Phase II:
//  else if( current_position > MAX_POSITION/3 && current_position > 2*MAX_POSITION/3 && forward)
//    y = MAX_DUTY;
//  else
//  {
//    y = trap_a3 * current_position + trap_b3;
//  }
//  else if( current_position > MAX_POSITION/3 && current_position > 2*MAX_POSITION/3 && !forward)
//  {
//    y = trap_a1 * current_position;
//  }
//  motor.set_duty(y);

}//END: trapezoid

// #S motion profile: 1/3 accel ; 1/3 max ; 1/3 decel
void s()
{
  //TODO
}//END: s

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ OPERATION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

// #MANUAL(0) mode: position and speed control by user input
void mode_MANUAL()
{
  //TODO
  print_position();
  if(command_position == MAX_POSITION/2)
    motor.brake();
  else if(command_position > MAX_POSITION/2)
    motor.drive_forward();
  else
    motor.drive_reverse();
    
  if(end_limit.limit_reached())
  {
    motor.brake();
    delay(2000);
    motor.drive_reverse();
  }
  if(!home_limit.limit_reached())
  {
    motor.brake();
    delay(2000);
    motor.drive_forward();
  }

//      //#drive to commanded position unless limit switches reached
//      if(!end_limit.limit_reached() || !home_limit.limit_reached())
//      {
//        //TODO
//        drive_to_position();
//        print_position();
//      }
}//END: mode_MANUAL

// #AUTO(1) mode: (DEFAULT) speed control cycle motions to end and home
void mode_AUTO()
{
  //#check if cycle is complete
  print_cycle();
  if(!cycle_complete)
  {
    drive_to_endstop();
    motor.brake();
    delay(5000);
    drive_to_home();
    motor.brake();
    delay(5000);
    //drive_to_start();
    //motor.brake();
    //delay(5000);
    cycle_count++;
    print_cycle();
  }
  if(cycle_count == CYCLES)
     cycle_complete = true;
}//END: mode_AUTO

// #PROFILE(2) mode: speed/acceleration/motion profile
void mode_PROFILE()
{
  //set PWM and Position
  switch(PROFILE)
  {
    case TRIANGLE:
      motor.motion_profile = TRIANGLE;
      triangle();
      break;
    case S:
      motor.motion_profile = S;
      s();
      break;
    case TRAPEZOID:
    default:
      motor.motion_profile = TRAPEZOID;
      trapezoid();
      break;
  }
  
  //drive motor load forward and reverse
  if(forward)
    motor.drive_forward();
  else
    motor.drive_reverse();
  //switch direction at limit switches
  if(!home_limit.limit_reached() || end_limit.limit_reached())
    forward = !forward;

}//END: mode_PROFILE

// #DEVELOPER(9) mode: custom demo program
void mode_DEVELOPER()
{  
  //motor.drive();
  //motor.drive_forward();
  //motor.drive_reverse()
  motor.set_duty(100);
  drive_to_endstop();
  //motor.brake();
  drive_to_home();
  //drive_to_position();
  //drive_to_start();
  //drive_forward_by(60);
  //drive_reverse_by(60);
}//END: mode_DEVELOPER
