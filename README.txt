#!~ ~~~~~~~~ PRECISION MICRODRIVES - GEARMOTOR DRIVER ~~~~~~~~~~~~~~  ~!#
#!~			README  			              ~!#
#
# This firmware runs a brushed DC motor. Motor parameters can be set in 
# a menu or in the defs.h file.
#
# The motor drives a linear slide mass load along a lead screw.
#
# The system uses a potentiometer to set PWM to control speed.
# Position can be set with a linear slide potentiometer.
#
# The control is set by an operation mode. 
# AUTO (Closed-Loop) mode: 
# Motor drives back and forth from preset position or endstop switch/home sensor
# Encoder counts position.
# PWM potentiometer slider is active to control speed. 
# Position slider is disabled.
#
# MANUAL (Open-Loop) mode:
# Motor drives to a position set by linear potentiometer slide.
# Encoder counts position.
# PWM potentiometer slider is active to control speed.
# Endstop and home sensors are active.
#
#

