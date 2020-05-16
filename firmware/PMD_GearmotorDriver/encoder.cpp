/**********************************************
 * PRECISION MICRODRIVES
 * Gearmotor Driver
 * encoder.cpp
 * Tristan Cool
 * April 2020
 *********************************************/

#include "encoder.h"

// #Constructor
Encoder::Encoder()
{
  pinEncoderA = encoderA_pin;
  pinEncoderB = encoderB_pin;
  pinMode(pinEncoderA, INPUT);
  pinMode(pinEncoderB, INPUT);

  //- Default Settings
  type        = MECHANICAL;    //MAGENTIC = 0, OPTICAL =1
  cpr         = CPR;           //counts per revolution
  count       = 0;             //encoder value
  ppi         = 0;             //pulses per millimeter
  rotation    = FORWARD;       //forward = true, reverse = false
  stateA      = LOW;           //encoder A
  stateB      = LOW;           //encoder B
}

// #Init
void Encoder::init(int mode, int ppr,int diameter, uint8_t pinA, uint8_t pinB)
{
  type = mode;                  //MAGENTIC = 0, OPTICAL =1
  cpr = ppr;                    //counts per revolution
  ppi = cpr/diameter*PI;        //pulses per millimeter
  
  pinEncoderA = pinA;           //user define pin
  pinEncoderB = pinB;           //user define pin
  pinMode(pinEncoderA, INPUT);
  pinMode(pinEncoderA, INPUT);
}//END: init

// #Print Encoder
void Encoder::print_encoder()
{
  Serial.println();
  Serial.println(F("~~~~~~~~~~~~~~~~ ENCODER ~~~~~~~~~~~~~~~~"));
  Serial.print(F("TYPE: "));
  Serial.println(type);
  Serial.print(F("DIRECTION(Forward = 1 ; Reverse = 0): "));
  Serial.println(rotation);
  Serial.print(F("COUNT: "));
  Serial.println(count);
  Serial.print(F("A: "));
  Serial.println(stateA);
  Serial.print(F("B: "));
  Serial.println(stateB);
  Serial.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
  Serial.println();
}//END: print_encoder

// #Read Counts
void Encoder::read_counts()
{
  
}//END: read_counts

// #Set Count
void Encoder::set_count(long pulse)
{
  count = pulse;
}//END: set_count

// #Get Count
long Encoder::get_count()
{
  return count;
}//END: get_count

// #Convert to mm
int Encoder::convert_to_mm()
{
  int mm = count/ppi;
  return mm;
}//END: convert_to_mm

// #Convert to Degrees
int Encoder::convert_to_deg()
{
  //TODO
}//END: convert_to_deg

// #Convert to Counts
long Encoder::convert_to_counts(int load, int mm_deg)
{
  if(load == LINEAR)
  {
    long pulse = ppi*mm_deg;
    set_count(pulse);
  }
  else if(load == ROTARY)
  {
    //TODO
  }
  else
    Serial.println(F("Invalid load setting"));
}//END: convert_to_counts
