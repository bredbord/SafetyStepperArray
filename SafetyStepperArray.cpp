/* Active Safety Stepper Array  -- Header File
 * Larson Rivera (a.k.a bredbord)
 * Last Modified: 3/18/2021
 * Version 1.0
*/

#include "SafetyStepperArray.h"

// CONSTRUCTORS###########################################################################################

SafetyStepperArray::SafetyStepperArray(byte enablePin, byte sleepPin, int maxStepperSpeed, int maxStepperAccel) {
  _stepper[MAX_SIZE] = nullptr;
  _stepperPositions[MAX_SIZE] = {};
  _limitPins[MAX_SIZE] = {};
  _stepperSafePositions[MAX_SIZE] = {};
  
  _steppersEnabled = false;    // enabled
  _ePin = enablePin;
  _sPin = sleepPin;
  _numSteppers = 0;
    
  _kStepperTimeout = 3000;
  _stepperTime = 0;
  _timeout = false;

  _maximumSpeed = maxStepperSpeed;
  _maximumAcceleration = maxStepperAccel;

  _homeSpeed = 0.3 * _maximumSpeed;  // defualt home at 30% of max speed
}

SafetyStepperArray::~SafetyStepperArray() {
  for (short s = 0; s < _numSteppers; s++) {
    delete _stepper[s];
    _stepper[s] = nullptr;
  }
}

// MUTATORS ################################################################################################

// Configration===========================================================================

bool SafetyStepperArray::addStepper(byte stepPin, byte dirPin, byte limitPin) { //attempt to add a stepper on the given pins
  
  if (_numSteppers < MAX_SIZE) {
    //allocate stepper
    AccelStepper *newStepper = new AccelStepper(1, stepPin, dirPin);
    
    if(newStepper == nullptr) return false;

    _stepper[_numSteppers] = newStepper;
    _limitPins[_numSteppers] = limitPin;

    _numSteppers++; // add at the index, and increase the count

    return true;
  } 
  
  return false;
}

void SafetyStepperArray::begin() { // Initalizes all necessary pins and sets default motion 
  
  //Pin setupS
  pinMode(_sPin, OUTPUT);
  pinMode(_ePin, OUTPUT);

  //Stepper setup
  for (short i = 0; i < _numSteppers; i++) { // Set default speed, accel, and configure limit pins
    _stepper[i]->setMaxSpeed(_maximumSpeed);
    _stepper[i]->setAcceleration(_maximumAcceleration);
    
    pinMode(_limitPins[i], INPUT_PULLUP);
    } 

  //Enable the steppers
  this->enableSteppers(true);
}


// Parameter Control======================================================================

bool SafetyStepperArray::setStepperPosition(byte stepNum, int pos) {  // Update a stepper's target position
  if(_stepperPositions[stepNum-1] != pos) {  // if the position is different
    _stepperPositions[stepNum-1] = pos;  // update the position
    _stepperTime = 0;  // reset the timeout clock -- we have a new place to be

    return true;
  }
  return false;
}

bool SafetyStepperArray::setStepperAcceleration(byte stepNum, int accel) {  // set a stepper's acceleration
  if (accel > _maximumAcceleration) return false;
  _stepper[stepNum-1]->setAcceleration(accel); return true;
}

bool SafetyStepperArray::setStepperSpeed(byte stepNum, int speed) {  // set a stepper's max speed
  if (speed > _maximumSpeed) return false;
  _stepper[stepNum-1]->setMaxSpeed(speed); return true;
}

bool SafetyStepperArray::setStepperSafePosition(byte stepNum, int pos) {  // set a stepper's safe position
  
  if (pos < 0) return false;
  _stepperSafePositions[stepNum-1] = pos; return true;
}

bool SafetyStepperArray::setHomeSpeed(int speed) {  // set the homing speed
  if (speed > _maximumSpeed) return false;
  _homeSpeed = speed; return true;
}

void SafetyStepperArray::setTimeoutMillis(unsigned int timeoutMillis) { _kStepperTimeout = timeoutMillis; }  // set hardware timeout clock

void SafetyStepperArray::reverseSteppers(bool mode) { for (short s = 0; s < _numSteppers; s++) { this->_stepper[s]->setPinsInverted(mode, false, false); } }

// Motion and Homing======================================================================

void SafetyStepperArray::enableSteppers(bool state) { // Enable stepper states
  
  if (state) {
    digitalWrite(_sPin, HIGH);
    digitalWrite(_ePin, LOW);
    _steppersEnabled = true;
    _stepperTime = 0; //reset the clocks
    _hardwareCatchupTime = 0;
    
  } else {
    digitalWrite(_sPin, LOW);
    digitalWrite(_ePin, HIGH);
    _steppersEnabled = false;
  }
}

bool SafetyStepperArray::homeSteppers(byte startStep, byte stopStep, int homeTimeMillis) {  // home steppers
  
  startStep--; stopStep--;
  unsigned long currentTime = millis();  // time of homing start
  bool allHome;  // all home flag

  for (short s = startStep; s <= stopStep; s++) { // set all steppers to run backward to an arbitrailly far back position
    AccelStepper *currentStepper = _stepper[s];
    currentStepper->setMaxSpeed(-_homeSpeed); 
    currentStepper->setAcceleration(_maximumAcceleration); 
    currentStepper->moveTo(-9999999);
  } 

  do {
    allHome = true;  // assume home unless proven otherwise
    
    for (byte s = startStep; s <= stopStep; s++) {  // check each stepper
      if (digitalRead(_limitPins[s]) == LOW) _stepper[s]->setCurrentPosition(0);  // for triggered limit switch
      else { _stepper[s]->run(); allHome = false; }
    }
    
  } while (!allHome && millis() - currentTime < homeTimeMillis);  // while all fixtures are not homed, and we are within the alotted time

  if (allHome) {
    for (byte s = startStep; s <= stopStep; s++) { _stepperPositions[s] = 0; _stepper[s]->setMaxSpeed(_maximumSpeed); }  // Zero out stepper positions.
    _stepperTime = 0;  // reset stepper timer
    return true;  // return success
  }

  else return false;  // otherwise, we timed out.
}

bool SafetyStepperArray::homeAll(int homeTimeMillis) { return this->homeSteppers(1, _numSteppers, homeTimeMillis); } // home all steppers

void SafetyStepperArray::run() { // RUN FUNCTION
  
  // TIMEOUT CHECKING
  if (_stepperTime > _kStepperTimeout) _timeout = true;  // if we've timed out, flag the timeout bool
  else _timeout = false;  // otherwise, we are within operational time limits


  // POSITIONAL UPDATE BASED ON TIMEOUT STATUS
  for (short s = 0; s < _numSteppers; s++) {
    if (_timeout) {_stepper[s]->moveTo(_stepperSafePositions[s]); }  // if timeout occured, request motion to nearest safe point
    else _stepper[s]->moveTo(_stepperPositions[s]);  // otherwise, operate on user data
  }


  // CEASE OF MOTION CHECK BASED ON REMAINING DISTANCE
  bool motionHold = true; // flag for all steppers have reached target positions. assume all are safe to disable unless proven otherwise

  //if a stepper still needs to move, flag the global holding variable that motion is still required, and set holding timer to zero
  for (short s = 0; s < _numSteppers; s++) { if (_stepper[s]->distanceToGo() != 0) { motionHold = false; _motionHoldTime = 0; } }


  // HARDWARE ENABLE AND DISABLE
  //We can disable IF and ONLY IF we are in a timeout state, we are holding position, and we've been holding for two seconds
  if (_timeout && motionHold && (_motionHoldTime > MOTION_HOLD_TIMEOUT)) { this->enableSteppers(false); }
  else if (!_steppersEnabled) { this->enableSteppers(true); }

  // HARDWARE RUNNING
  if (_steppersEnabled && _hardwareCatchupTime > HARDWARE_CATCHUP_MILLIS) { 
    for(short s = 0; s < _numSteppers; s++) {
      AccelStepper *currentStepper = _stepper[s];
      // if we have hit the endstop and are requesting to move to a lesser position, don't run.
      if (! (!digitalRead(_limitPins[s]) && (currentStepper->targetPosition() < currentStepper->currentPosition())) ) currentStepper->run();
    }
  }
  
}



// Observers###############################################################################################

int SafetyStepperArray::getStepperPosition(byte stepperNum) { return _stepper[stepperNum-1]->currentPosition(); }

bool SafetyStepperArray::isEnabled() { return _steppersEnabled; }
bool SafetyStepperArray::isHome() { for (short s = 0; s < _numSteppers; s++) { if (digitalRead(_limitPins[s])) return false; } return true; } // if any switches are high, we are not home

void SafetyStepperArray::emergencyStop() { enableSteppers(false); while (true) delay(1000); }  //dive into a while loop for emergency
