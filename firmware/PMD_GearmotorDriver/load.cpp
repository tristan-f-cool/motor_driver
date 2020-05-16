/**********************************************
 * PRECISION MICRODRIVES
 * Gearmotor Driver
 * load.cpp
 * Tristan Cool
 * April 2020
 *********************************************/

 #include "load.h"

// #Constructor
Load::Load()
{
  // - Default Settings
  type                          = LINEAR;
  load_settings.my_name         = LOAD_ID;            //load reference id
  load_settings.max_speed       = MAX_SPEED;          //rpm
  load_settings.max_torque      = MAX_TORQUE;         //mNm
  load_settings.mass            = MASS;               //g
  load_settings.diameter        = DIAMETER;           //mm
  load_settings.min_position    = MIN_POSITION;       //mm
  load_settings.max_position    = MAX_POSITION;       //mm
  load_settings.start_position  = START_POSITION;     //mm
}

// #Init
void Load::init()
{
  
}//END: init

// #Print Load Settings
void Load::print_load_settings()
{
  Serial.println();
  Serial.println(F("~~~~~~~~~~~~~~ "));
  Serial.println(load_settings.my_name);
  Serial.print(F("Mass(g): "));
  Serial.print(load_settings.mass);
  Serial.print(F("Diameter(mm): "));
  Serial.print(load_settings.diameter);
  Serial.print(F("Max. Torque(mNm): "));
  Serial.print(load_settings.max_torque);
  Serial.print(F("Max Speed(rpm): "));
  Serial.print(load_settings.max_speed);
  Serial.print(F("Min. Position(mm): "));
  Serial.print(load_settings.min_position);
  Serial.print(F("Max. Position(mm): "));
  Serial.print(load_settings.max_position);
  Serial.print(F("Start Position(mm): "));
  Serial.print(load_settings.start_position);
  Serial.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
  Serial.println();
  
}//END: print_load_settings

// #Print Load
void Load::print_load()
{
  Serial.println();
  Serial.println(F("~~~~~~~~~~~~~~ LOAD ~~~~~~~~~~~~~~~~~~"));
  Serial.println(load_settings.my_name);
  Serial.print(F("Speed(rpm): "));
  Serial.print(my_speed);
  Serial.print(F("Position(mm): "));
  Serial.print(my_position);
  Serial.print(F("Torque(mNm): "));
  Serial.print(my_torque);
  Serial.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
  Serial.println();
  
}//END: print_load

// #Calculte Position
int Load::calculate_position()
{
  //TODO
}//END: calculate_position

// #Calculte Speed
int Load::calculate_speed(int motor_speed, int gear_ratio)
{
  //TODO
  int rpm = motor_speed/gear_ratio;
  set_speed(rpm);
  return rpm;
}//END: calculate_speed

// #Calculte Torque
int Load::calculate_torque(int motor_torque, int gear_ratio)
{
  //TODO
  int mNm = motor_torque*gear_ratio;
  set_torque(mNm);
  return mNm;
}//END: calculate_torque

// #Set Position (mm or degree)
void Load::set_position(int mm_deg)
{
  my_position = mm_deg;
}//END: set_position

// #Get Position
int Load::get_position()
{
  return my_position;
}//END: get_position

// #Set Speed
void Load::set_speed(int rpm)
{
  my_speed = rpm;
}//END: set_speed

// #Get Speed
int Load::get_speed()
{
  return my_speed;
}//END: get_speed
   
// #Set Torque
void Load::set_torque(int mNm)
{
  my_torque = mNm;
}//END: set_torque

// #Get Torque
int Load::get_torque()
{
  return my_torque;
}//END: get_torque
