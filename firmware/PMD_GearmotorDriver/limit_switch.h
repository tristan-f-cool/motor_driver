/**********************************************
 * PRECISION MICRODRIVES
 * Gearmotor Driver
 * limit_switch.h
 * Tristan Cool
 * April 2020
 *********************************************/

#include "Arduino.h"
#include "defs.h"

#ifndef include_limit_switch_h
#define include_limit_switch_h

class Limit_Switch
{
  public:
    Limit_Switch();

    // - Variables
    byte limit_switch_val;        // HIGH ; LOW 
    int  limit_switch_type;       // MECHANICAL = 0 ; OPTICAL = 1
    int  limit_switch_position;   // HOME = 0 ; ENDSTOP = 1

    // - Functions
    void init(uint8_t pin_limit_switch, int type, int pos);
    void print_limit_switch();
    bool limit_reached();
    
  private:
    uint8_t pinLimitSwitch;
  
};
#endif //include_limit_switch_h
