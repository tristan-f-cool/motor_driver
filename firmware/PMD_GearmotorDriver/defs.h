/**********************************************
 * PRECISION MICRODRIVES
 * Gearmotor Driver
 * defs.h
 * Tristan Cool
 * April 2020
 *********************************************/

#ifndef defs_h
#define defs_h
 
//###################################### PINS ########################################//

//****** Potentiometer and Buttons ********/
#define potentiometer1_pin    A0     //A0 - PWM/Speed potentiometer
#define potentiometer2_pin    A1     //A1 - position slider
#define button1_pin           12     //D12 - emergency stop (or attach interupt)
#define button2_pin           6      //D6 - position reset (or attach interupt)

//************* Motor ********************/
#define motor_pin             9      //D9  - unidirectional transistor
#define motor_pin2            10     //D10 - bidirectional h-bridge 
#define motor_enable_pin      11     //D11 - h-bridge enable

//************* Encoder ******************/
#define encoderA_pin          2      //D2 (or attach interupt)
#define encoderB_pin          4      //D3 (or attach interupt)

//************ Limit Switch **************/
#define limit_switch1_pin     7      //D7 (or attach interupt)
#define limit_switch2_pin     8      //D8 (or attach interupt)

//################################## VARIABLES #######################################//

//********* Operation Modes *************/
#define MANUAL                 0          //Open-Loop
#define AUTO                   1          //Closed-Loop
#define PROFILE                2          //Motion Profile
#define DEVELOPER              9          //Developer

#define DEBUG                  1          //True = 1 ; False = 0
#define MENU                   1          //True = 1 ; False = 0

//****** Potentiometer and Buttons *******/
#define SLIDER_DISABLED        0          //Slider disabled  
#define SLIDER_POSITION        1          //Slider Position mode

#define POTENTIOMETER_DISABLED 0          //Potentiometer disabled
#define POTENTIOMETER_PWM      1          //Potentiometer PWM/Speed mode

//****** Motor ***************************/
#define UNIDIRECTIONAL         0         //single direction transistor PWM drive
#define BIDIRECTIONAL          1         //dual direction H-bridge gate drive

// - Default parameters
#define MOTOR_ID              "212-405"  //motor identification
#define GEAR_RATIO            100        //gear reduction :1
#define EFFICIENCY            31         //%
#define VOLTAGE_SUPPLY        12         //VDC
#define MAX_CURRENT           240        //mA
#define RATED_LOAD            20         //mNm
#define MAX_NO_LOAD_SPEED     225        //rpm
#define MAX_RATED_LOAD_SPEED  184        //rpm

#define MOTOR_PWM_DUTY        60         //%
#define MIN_DUTY              0          //%
#define MAX_DUTY              100        //%

//****** Load ***************************/
#define NO_LOAD               0
#define LOAD                  1

#define ROTARY                0
#define LINEAR                1

#define FORWARD               1
#define REVERSE               0

// - Default Parameters
#define LOAD_ID           "Linear Rail"  //load reference id
#define MAX_SPEED             10         //rpm
#define MAX_TORQUE            200        //mNm
#define MASS                  500        //g
#define DIAMETER              16         //mm

// - Position
#define MIN_POSITION          0          //mm
#define START_POSITION        75         //mm
#define MAX_POSITION          150        //mm

// - Cycles
#define MAX_CYCLES            3

// - Motion Profiles
#define TRIANGLE              0          // 1/2 acceleration - 1/2 deceleration
#define TRAPEZOID             1          // 1/3 acceleration - 1/3 max speed - 1/3 deceleration
#define S                     2

//****** Encoder ************************/
#define MAGNETIC              0
#define OPTICAL               1

#define CPR                   3         // counts per revolution

//****** Limit Switch********************/
#define MECHANICAL            0
#define OPTICAL               1

#define HOME                  0
#define ENDSTOP               1

#endif // defs_h
