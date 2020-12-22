
## Background
This firmware is intended to be used with the popular TSDZ2\
electric bike mid mount motor manufactured by Tongsheng.

It is based on the work of casainho and others.\
https://github.com/OpenSourceEBike/TSDZ2-Smart-EBike

## Purpose
This firmware can be used to run the the TSDZ2 motor with the minimum amount\
of external components in order to possibly decrease the risk of downtime,\
giving less maintenance and a cleaner lock.

When running this firmware the following components are not used:\
* Display
* Speed Sensor

Throttle is optional.

Control of the ebike operation is done by reusing the remote keypad\
that comes with the kit.

A custom cable harness has to be manufactured in order to connect\
the remote keypad to the main control cable of the motor.

See ... for instruction on how this is to be made.


## Operation
The motor is controlled though the remote keypad through a set of key combinations.\

Available operation modes:\
* Torque adjusted pedal assist
* Cruise

Pedal assist is the selected operation mode on startup.\
Throttle is disabled by default and can be enabled by entering\
cruise mode and then reverting back to pedal assist.

The brake signal is connected to both the info and power button on the keypad.\
This is a combo button which is also used to switch between the available modes. \
When any of the brake keys are pressed the motor will stop immediatelly.

### Torque adjusted pedal assist
This is the way the motor operates with the original firmware.\

There are 5 assist levels that can be tweaked in the code, see config.h.\
Assist level is adjusted by the (+) and (-) buttons on the keypad.\

This mode is entered by holding down info/power combo button for 2s\
followed by pressing the (-) button.

### Cruise
Cruise is a lazy thottle mode which should be used with care\
and is useful for e.g. daily commute.

The (+) and (-) buttons selects between 7 different power levels\
where power is feed to the motor independently if you are pedaling or not.\

In this mode the motor will stop when either of the brake buttons are\
pressed or the throttle is touched. If any of those events occur it will\
enter paused mode where no power is feed to the motor.

There are two ways to exit paused cruise mode:\
* Press (+) button to return to previously selected power level.
* Press (-) button to return to first power level (i.e. 0, no power).

This mode is entered by holding down info/power combo button for 2s\
followed by pressing the (+) button.

### Reset
If for any reason you want to return to startup mode, hold down\
brake button for 10 seconds and it will revert back to pedal assist\
with throttle disabled.

## Code Changes from Base
* Code refactored into more clear units.
* Removed all global state variables.
* Removed all code related to display communication.
* Removed all code related to configuration.
* Removed code related to speed sensor.
* Implement logic for operation control from remote keypad.
