/**********************************************
 * PRECISION MICRODRIVES
 * Gearmotor Driver
 * limit_switch.cpp
 * Tristan Cool
 * April 2020
 *********************************************/

 #include "limit_switch.h"

 // #Constructor
Limit_Switch::Limit_Switch()
{
  pinLimitSwitch = limit_switch1_pin;         //default D7
  pinMode(pinLimitSwitch, INPUT_PULLUP);

  //- Default Settings
  limit_switch_type     = MECHANICAL;
  limit_switch_position = ENDSTOP;
}

 // #Init
void Limit_Switch::init(uint8_t pin_limit_switch, int type, int pos)
{
  pinLimitSwitch = pin_limit_switch;
  pinMode(pinLimitSwitch, INPUT_PULLUP);    //user define pin
  limit_switch_type     = type;
  limit_switch_position = pos;
}//END: init

// #Print Limit Switch
void Limit_Switch::print_limit_switch()
{
  Serial.println();
  Serial.println(F("~~~~~~~~~~~~ LIMIT SWITCH~~~~~~~~~~~~~~"));
  Serial.println(F("(HOME = 0 ; ENDSTOP = 1)"));
  Serial.println(F("(MECHANICAL = 0 ; OPTICAL = 1)"));
  Serial.print(F("~Position: "));
  Serial.println(limit_switch_position);
  Serial.print(F("~Type: "));
  Serial.println(limit_switch_type);
  Serial.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
  Serial.println();
  
}//END: print_limit_switch

// #Limit Reached
bool Limit_Switch::limit_reached()
{
  limit_switch_val = digitalRead(pinLimitSwitch);
  //MECHANICAL and OPTICAL switches have inverse logic 
  if(limit_switch_type == MECHANICAL && !limit_switch_val)
    Serial.println(F("!~ LIMIT SWITCH REACHED ~!"));

  if(limit_switch_type == OPTICAL && limit_switch_val)
    Serial.println(F("!~ LIMIT SWITCH REACHED ~!"));
    
  return !limit_switch_val;
}//END: limit_reached
