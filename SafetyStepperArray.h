/* Active Safety Stepper Array  -- Header File
 * Larson Rivera (a.k.a bredbord)
 * Last Modified: 3/18/2021
 * Version 1.0
*/

#ifndef SAFETY_STEPPER_ARRAY_H
#define SAFETY_STEPPER_ARRAY_H

#include "Arduino.h"

#include <AccelStepperExtended.h>

#define MAX_SIZE 16
#define MOTION_HOLD_TIMEOUT 1000

#define HARDWARE_CATCHUP_MILLIS 10

class SafetyStepperArray {
  
  private:

    //Steppers
    AccelStepperExtended *_stepper[MAX_SIZE];
    int _stepperPositions[MAX_SIZE];  // target stepper positions
    int _limitPins[MAX_SIZE];         // motion limit pins
    int _stepperSafePositions[MAX_SIZE];  // stepper safe positions for disable
    
    bool _steppersEnabled;    // whether or not the steppers are enabled
    bool _timeout;           // wheter or not we are timed out
    
    byte _ePin;         // enable and sleep pins
    byte _sPin;
    byte _numSteppers;  // number of steppers
    
    unsigned int _kStepperTimeout;
    elapsedMillis _stepperTime;
    elapsedMillis _motionHoldTime;
    elapsedMillis _hardwareCatchupTime;

    int _maximumSpeed;
    int _maximumAcceleration;

    int _homeSpeed;

    void enableSteppers(bool);  // set stepper states
    byte getDirection(byte);
    

  public:

    // 'STRUCTORS===============
    SafetyStepperArray(byte, byte, int, int); //enable, sleep, max speed, max accel
    ~SafetyStepperArray(); // Destructor

    // MUTATORS================
    // Configuration
    bool addStepper(byte, byte, byte); // dir, step, stop
    void begin(); // initializer

    // Parameter Control
    bool setStepperPosition(byte, int); // stepper number, target position in usteps
    bool setStepperAcceleration(byte, int);  // set a stepper's acceleration
    bool setStepperSpeed(byte, int);  // set a stepper's max speed
    bool setStepperSafePosition(byte, int);  // set a stepper's safe position
    bool setHomeSpeed(int); // set the homing speed
    void setTimeoutMillis(unsigned int); // set the timeout Clock
    void reverseSteppers(bool); // reverse steppers

    // Motion and Homing
    bool homeSteppers(byte, byte, int);  // home from and to (inclusive) on a timer
    bool homeAll(int);
    void runSteppers();

    // OBSERVERS===============
    int getStepperPosition(byte);
    bool isEnabled();
    bool isHome();

    void emergencyStop();

};

#endif
