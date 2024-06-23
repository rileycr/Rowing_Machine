#include <Arduino.h>

#include "Doar.h"

// Constructors
Doar::Doar(bool isStarboard)
{
   Doar::isStarboard = isStarboard;

   // Set cycle values
   _numCycleSteps = 1000;
   _increment = 7;
   _currentStep = 0;

   // Set servo values
   _liftCenter = 110;
   _driveCenter = 90;
   _liftRange = 25;
   _driveRange = 45;
}

int Doar::getLiftAngle()
{
   // Convert the Cycle step to an angle in radians
   auto cycleAngle = _currentStep / (double)_numCycleSteps * TWO_PI;

   auto sign = isStarboard ? 1.0 : -1.0;
   auto b = 3.0;                                            // Squareness of the wave (1 = regular, 10 = very square)
   auto p = 5.0;                                            // Factor on period adjustment (1 = large adjustment, 10 = small adjustment)

   // Calculate factor from function (returns -1 to 1)
   auto factor = sign * cos((-1 / (b * p)) * sqrt(pow(b, 2) + 1) * asin(b * sin(cycleAngle) / sqrt(pow(b, 2) + 1)) + cycleAngle) * sqrt((1 + pow(3, 2)) / (1 + pow(3, 2) * pow(cos((-1 / (b * p)) * sqrt(pow(b, 2) + 1) * asin(b * sin(cycleAngle) / sqrt(pow(b, 2) + 1)) + cycleAngle), 2)));

   // Set the servo angle with center and range values.
   double angle = _liftCenter + factor * _liftRange;

   return angle;
}

int Doar::getDriveAngle(double amplitude)
{
   // Convert the Cycle step to an angle in radians
   double cycleAngle = _currentStep / (double)_numCycleSteps * TWO_PI;

   auto magnitude = amplitude * (isStarboard ? 1.0 : -1.0);
   auto b = 2.0;                           // Squareness of the wave (1 = regular, 10 = very square)

   // Calculate factor from function (returns -1 to 1)
   auto factor = magnitude * sin(cycleAngle) * sqrt((1 + pow(b, 2)) / (1 + pow(b, 2) * pow(sin(cycleAngle), 2)));

   // Set the servo angle with center and range values.
   double angle = _driveCenter + factor * _driveRange;

   return angle;
}

void Doar::determineState(double forwardInput, double turnInput)
{
   auto deadzone = 0.25;

   // Determine Forward/Backwards component
   if (abs(forwardInput) <= deadzone)
   {
      vesselDirection = direction::stopped;
   }
   else if (forwardInput > deadzone)
   {
      vesselDirection = direction::forward;
      lastVesselDirection = vesselDirection;
   }
   else if (forwardInput < -deadzone)
   {
      vesselDirection = direction::reverse;
      lastVesselDirection = vesselDirection;
   }

   // Determine Left/Right component
   if (abs(turnInput) <= deadzone)
   {
      vesselTurning = turning::straight;
   }
   else if (turnInput > deadzone)
   {
      vesselTurning = turning::right;
      lastVesselTurning = vesselTurning;
   }
   else if (turnInput < -deadzone)
   {
      vesselTurning = turning::left;
      lastVesselTurning = vesselTurning;
   }
}

// Returns value from 0.0 to 1.0
double Doar::calculateTurningFactor(double turnInput)
{
   auto factor = 1.0;

   if (isStarboard)
   {
      if (turnInput >= 0)
      {
         factor -= turnInput;
      }
   }
   else
   {
      if (turnInput <= 0)
      {
         factor += turnInput;
      }
   }

   return factor;
}

void Doar::attachPins(int liftPin, int drivePin)
{
   liftServo.attach(liftPin);
   driveServo.attach(drivePin);
}

void Doar::step(double forwardInput, double turnInput)
{
   determineState(forwardInput, turnInput);

   auto turningFactor = calculateTurningFactor(turnInput);

   double amplitude = 0.0;

   if (vesselDirection == direction::stopped && vesselTurning == turning::straight)
   {
      if (_currentStep <= _increment || _currentStep >= (_numCycleSteps - _increment))
      {
         _currentStep = 0;
         amplitude = 0.0;
      }
      else
      {
         amplitude = _lastAmplitude;
         turningFactor = _lastTurningFactor;
      }
   }
   else if (vesselDirection == direction::stopped && vesselTurning != turning::straight)
   {
      // Reset turning factor to have equal amplitude
      turningFactor = 1.0;

      amplitude = turnInput * (isStarboard ? -1.0 : 1.0);
   }
   else if (vesselDirection == direction::forward || vesselDirection == direction::reverse)
   {
      amplitude = forwardInput;
   }

   // Save values for next loop
   _lastAmplitude = amplitude;
   _lastTurningFactor = turningFactor;

   // Modify the amplitude if the boat is turning
   amplitude *= turningFactor;

   // Set _currentStep for next loop
   auto nextStep = _currentStep + _increment;
   // Add and get remainder to ensure result is a positive number
   _currentStep = (nextStep + _numCycleSteps) % _numCycleSteps;

   // Write servo angles
   liftServo.write(getLiftAngle());
   driveServo.write(getDriveAngle(amplitude));
}

void Doar::reset()
{
   _currentStep = 0;
   liftServo.write(getLiftAngle());
   driveServo.write(getDriveAngle(0.0));
}