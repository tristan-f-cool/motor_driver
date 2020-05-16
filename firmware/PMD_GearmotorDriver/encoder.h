/**********************************************
 * PRECISION MICRODRIVES
 * Gearmotor Driver
 * encoder.h
 * Tristan Cool
 * April 2020
 *********************************************/

#include "Arduino.h"
#include "defs.h"

#ifndef include_encoder_h
#define include_encoder_h

class Encoder
{
  public:
    Encoder();

    // - Variables
    int type;               //MAGNETIC = 1, OPTICAL = 0
    int cpr;                //counts per revolution
    int ppi;                //pulses per index (mm)
    bool rotation;           //forward = true, reverse = false
    volatile byte stateA;   //encoder A
    volatile byte stateB;   //encoder B
    volatile long count;    //encoder value
    
    // - Functions
    void init(int type, int cpr, int diameter, uint8_t pinA, uint8_t pinB);
    void print_encoder();
    void read_counts();
    void set_count(long pulse);
    long get_count();
    int convert_to_mm();
    int convert_to_deg();
    long convert_to_counts(int load,int mm_deg);

  private:
    uint8_t pinEncoderA;
    uint8_t pinEncoderB;
  
};
#endif //include_encoder_h
