/**********************************************
 * PRECISION MICRODRIVES
 * Gearmotor Driver
 * motor.h
 * Tristan Cool
 * April 2020
 *********************************************/

#include "Arduino.h"
#include "defs.h"

#ifndef include_motor_h
#define include_motor_h

// - Motor parameters
typedef struct {
  uint8_t pin;
  String motor_id;            //motor identification
  int gear_ratio;             //gear reduction :1
  int efficiency;             //%
  int voltage_supply;         //VDC
  int max_current;            //mA
  int rated_load;             //mNm
  int max_no_load_speed;      //rpm
  int max_rated_load_speed;   //rpm      
} MotorInfo;

class Motor
{
  public:
    Motor();

    // - Default Settings
    MotorInfo motor_settings;

    // - Variables
    int driver;                 //unidirectional (transistor) ; bi-directional (h-bridge gate)
    int motion_profile;         //Triangle, Trapezoid, S
    int prev_pwm_duty;          //%
    int min_duty;               //%
    int max_duty;               //%

    // - Functions
    void init(uint8_t pin_motor);
    void print_motor_settings();
    void print_motor();
    
    void set_duty(int pwm_duty);
    int  get_duty();
    int  convert_duty_to_percent(int duty);
    int  convert_duty_from_percent(int duty);
    void drive();
    void drive_forward();
    void drive_reverse();
    void brake();
    void restart();

    int calculate_speed(int encoder_count, int encoder_cpr);
    int calculate_torque();
    void set_speed(int rpm);
    int get_speed();
    void set_torque(int mNm);
    int get_torque();
      
  private:
    uint8_t pinMotor;           //unidirectional transistor
    uint8_t pinMotor2;          //bidirectional
    uint8_t pinMotorEnable;     //h-bridge enable
    int motor_pwm_duty;         //%
    int my_speed;               //rpm
    int my_torque;              //rpm
    
};

#endif //include_motor_h
