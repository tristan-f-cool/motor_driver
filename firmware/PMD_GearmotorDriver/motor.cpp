/**********************************************
 * PRECISION MICRODRIVES
 * Gearmotor Driver
 * motor.cpp
 * Tristan Cool
 * April 2020
 *********************************************/

#include "motor.h"

// #Constructor
Motor::Motor()
{
  pinMotor = motor_pin;                //default D9  - unidirectional transistor
  pinMotor2 = motor_pin2;              //default D10 - bidirectional h-bridge
  pinMotorEnable = motor_enable_pin;   //default D11 - h-bridge enable
  pinMode(pinMotor,  OUTPUT);
  pinMode(pinMotor2, OUTPUT);
  pinMode(pinMotorEnable, OUTPUT);
  
  // - Default Settings
  motor_settings.motor_id              = MOTOR_ID;              //motor identification
  motor_settings.gear_ratio            = GEAR_RATIO;            //gear reduction :1
  motor_settings.efficiency            = EFFICIENCY;            //%
  motor_settings.voltage_supply        = VOLTAGE_SUPPLY;        //VDC
  motor_settings.max_current           = MAX_CURRENT;           //mA
  motor_settings.rated_load            = RATED_LOAD;            //mNm
  motor_settings.max_no_load_speed     = MAX_NO_LOAD_SPEED;     //rpm
  motor_settings.max_rated_load_speed  = MAX_RATED_LOAD_SPEED;  //rpm

  driver                = BIDIRECTIONAL;      
  motion_profile        = TRAPEZOID;
  prev_pwm_duty         = 0;                 //%
  motor_pwm_duty        = MOTOR_PWM_DUTY;    //%
  min_duty              = MIN_DUTY;          //%
  max_duty              = MAX_DUTY;          //%
}

// #Init
void Motor::init(uint8_t pin_motor)
{
  pinMotor = pin_motor;               //user define pin
  pinMode(pinMotor, OUTPUT);
}//END: init

// #Print Motor Settings
void Motor::print_motor_settings()
{
  Serial.println();
  Serial.println(F("~~~~~~~~~~~~~~ MOTOR ~~~~~~~~~~~~~~~~~~"));
  Serial.print(F("~Motor ID: "));
  Serial.println(motor_settings.motor_id);
  Serial.print(F("~Gear ratio: "));
  Serial.println(motor_settings.gear_ratio);
  Serial.print(F("~Efficiency(%): "));
  Serial.println(motor_settings.efficiency);
  Serial.print(F("~Supply Voltage(V): "));
  Serial.println(motor_settings.voltage_supply);
  Serial.print(F("~Max. Current(mA): "));
  Serial.println(motor_settings.max_current);
  Serial.print(F("~Rated Load(mNm): "));
  Serial.println(motor_settings.rated_load);
  Serial.print(F("~Max. N/L Speed(rpm): "));
  Serial.println(motor_settings.max_no_load_speed);
  Serial.print(F("~Max. Rated Load Speed(rpm): "));
  Serial.println(motor_settings.max_rated_load_speed);
  Serial.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
  Serial.println();

}//END: print_motor_settings

// #Print Motor
void Motor::print_motor()
{
  Serial.println();
  Serial.println(F("~~~~~~~~~~~~~~ MOTOR ~~~~~~~~~~~~~~~~~~"));
  Serial.println(motor_settings.motor_id);
  Serial.print(F("Speed(rpm): "));
  Serial.print(my_speed);
  Serial.print(F("Torque(mNm): "));
  Serial.print(my_torque);
  Serial.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
  Serial.println();
  
}//END: print_motor

// #Set Duty (%)
void Motor::set_duty(int pwm_duty)
{
  //check for invalid pwm_duty
  if(pwm_duty < 0)
  {
    Serial.println(F("Invalid PWM duty: cannot be negative"));
    pwm_duty = 0;
  }
  if(pwm_duty > 255)
  {
    Serial.println(F("Invalid PWM duty: cannot be over 100%"));
    pwm_duty = 255;
  }

  motor_pwm_duty = pwm_duty;
  Serial.print(F("PWM duty(%): "));
  Serial.println(motor_pwm_duty);
  Serial.println();
}//END: set_duty

// #Get Duty
int Motor::get_duty()
{
  return motor_pwm_duty;
}//END: get_duty

// #Convert Duty to % (from Analog:0-255)
int Motor::convert_duty_to_percent(int duty)
{
  return map(duty,0,255,0,100);
}//END: convert_duty_to_percent

// #Convert Duty from % (to Analog:0-255)
int Motor::convert_duty_from_percent(int duty)
{
  return map(duty,0,100,0,255);
}//END: convert_duty_from_percent

// #Drive (PWM) - unidirectional transistor PWM
void Motor::drive()
{
  analogWrite(pinMotor,convert_duty_from_percent(motor_pwm_duty));
}//END: drive

// #Drive Forward (PWM) - bidirectional h-bridge gate PWM
void Motor::drive_forward()
{
  Serial.println(F("DRIVE FORWARD"));
  analogWrite(pinMotorEnable, convert_duty_from_percent(motor_pwm_duty));
  digitalWrite(pinMotor, LOW);
  digitalWrite(pinMotor2, HIGH);
}//END: drive_forward

// #Drive Reverse (PWM) - bidirectional h-bridge gate PWM
void Motor::drive_reverse()
{
  Serial.println(F("DRIVE REVERSE"));
  analogWrite(pinMotorEnable, convert_duty_from_percent(motor_pwm_duty));
  digitalWrite(pinMotor, HIGH);
  digitalWrite(pinMotor2, LOW);
}//END: drive_reverse

// #Brake (PWM=0)
void Motor::brake()
{
  Serial.println(F("!~ MOTOR BRAKE ~!"));
  if(driver = UNIDIRECTIONAL)
  {
    analogWrite(pinMotor,0);
  }
  else if(driver = BIDIRECTIONAL)
  {
  analogWrite(pinMotorEnable, convert_duty_from_percent(motor_pwm_duty));
  digitalWrite(pinMotor, LOW);
  digitalWrite(pinMotor2, LOW);
  }
}//END: brake

// #Restart (previous PWM)
void Motor::restart()
{
  Serial.println(F("!~ MOTOR RESTART ~!"));
  set_duty(prev_pwm_duty);
  if(driver = UNIDIRECTIONAL)
  {
    drive();
  }
  else if(driver = BIDIRECTIONAL)
  {
    //TODO
  }
}//END: restart

// #Calculate Speed (rpm)
int Motor::calculate_speed(int encoder_count, int encoder_cpr)
{
  //TODO
  int rpm = encoder_count*60/encoder_cpr;
  set_speed(rpm);
  return rpm;
}//END: calculate_speed

// #Calculate Torque (mNm)
int Motor::calculate_torque()
{
  //TODO
}//END: calculate_torque

// #Set Speed
void Motor::set_speed(int rpm)
{
  my_speed = rpm;
}//END: set_speed

// #Get Speed
int Motor::get_speed()
{
  return my_speed;
}//END: get_speed
   
// #Set Torque
void Motor::set_torque(int mNm)
{
  my_torque = mNm;
}//END: set_torque

// #Get Torque
int Motor::get_torque()
{
  return my_torque;
}//END: get_torque
