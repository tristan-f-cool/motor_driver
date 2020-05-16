/**********************************************
 * PRECISION MICRODRIVES
 * Gearmotor Driver
 * load.h
 * Tristan Cool
 * April 2020
 *********************************************/

#include "Arduino.h"
#include "defs.h"

#ifndef include_load_h
#define include_load_h

// - Load parameters
typedef struct {
    String my_name;         //load reference id
    int max_speed;          //rpm
    int max_torque;         //mNm
    int mass;               //g
    int diameter;           //mm
    int min_position;       //mm
    int max_position;       //mm
    int start_position;     //mm
} LoadInfo;

class Load
{
  public:
    Load();

    // - Default Settings
    LoadInfo load_settings;

    // - Variables
    int type;
  
    // - Functions
    void init();
    void print_load_settings();
    void print_load();
    int calculate_position();
    int calculate_speed(int motor_speed, int gear_ratio);
    int calculate_torque(int motor_torque, int gear_ratio);   
    void set_position(int mm_deg);
    int get_position(); 
    void set_speed(int rpm);
    int get_speed();
    void set_torque(int mNm);
    int get_torque();

  private:
    int my_position;
    int my_speed;
    int my_torque;
  
};
#endif //include_load_h
