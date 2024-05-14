#ifndef DOAR_H
#define DOAR_H

#include <Servo.h>

// Doar - Digital Oar class
class Doar
{
private:
   enum direction
   {
      reverse = -1,
      stopped = 0,
      forward = 1
   };

   enum turning
   {
      left = -1,
      straight = 0,
      right = 1
   };

   bool isStarboard;

   int _numCycleSteps;
   int _increment;
   int _currentStep;

   int _liftCenter;
   int _liftRange;
   int _driveCenter;
   int _driveRange;

   double _lastAmplitude;
   double _lastSpeed;
   double _lastTurningFactor;

   direction vesselDirection;
   direction lastVesselDirection;

   turning vesselTurning;
   turning lastVesselTurning;

   Servo driveServo;
   Servo liftServo;

   // Calculates the angle for the Lift servo
   int getLiftAngle();

   // Calculates the angle for the Drive servo
   int getDriveAngle(double amplitude);

   // Sets the vesselDirection variable based on the inputs
   void determineState(double forwardInput, double turnInput);

   // Using the State of the vessel, get an amplitude factor
   double calculateTurningFactor(double turnInput);

public:
   // Constructor
   Doar(bool isStarboard);

   // Connect Servo objects to Arduino pins
   void attachPins(int liftPin, int drivePin);

   // Increment the cycle step based on the inputs (values -1.0 to 1.0)
   void step(double forwardInput, double turnInput);

   // Move oar to starting position
   void reset();
};

#endif
