#include <Arduino.h>
#include <Servo.h>
#include <PinChangeInterrupt.h>

#include "Doar.h"

#ifdef ARDUINO_AVR_ATTINYX4
// Define PWM Servo pins on Arduino
#define STARBOARD_LIFT PIN_PB2
#define STARBOARD_DRIVE PIN_PA7
#define PORT_LIFT PIN_PA6
#define PORT_DRIVE PIN_PA5

// Define PWM RX pins on Arduino
#define RX_FORWARD PIN_PA1
#define RX_TURN PIN_PA2

// Define debug LED
#define DEBUG_LED PIN_PA3

#elif defined(ARDUINO_AVR_UNO)
// Define PWM Servo pins on Arduino
#define STARBOARD_LIFT 3
#define STARBOARD_DRIVE 5
#define PORT_LIFT 6
#define PORT_DRIVE 9

// Define PWM RX pins on Arduino
#define RX_FORWARD 10
#define RX_TURN 11
#define DEBUG_LED LED_BUILTIN
#endif

// Define Logic frequency, set frame length in ms
// 22ms ~ 45.45 frames/s
// 10ms = 100 frames/s
const int FRAME_FREQUENCY = 100;
const int FRAME_LENGTH = 1000 / FRAME_FREQUENCY;

// Frame Timing
unsigned long frameExecuteTime = 0;

// Declare the Digital Oar objects
Doar starboard(true);
Doar port(false);

// Debug LED pin
bool ledOn = true;

// RX input signals
unsigned long forwardSignalAvg;
unsigned long turnSignalAvg;

volatile int rxTurnPulse;
volatile int rxFwdPulse;

volatile unsigned long turnStartTime;
volatile unsigned long fwdStartTime;

void updateTurn()
{
   if (digitalRead(RX_TURN) == HIGH)
   {
      turnStartTime = micros();
   }
   else
   {
      rxTurnPulse = micros() - turnStartTime;
   }
}

void updateFwd()
{
   if (digitalRead(RX_FORWARD) == HIGH)
   {
      fwdStartTime = micros();
   }
   else
   {
      rxFwdPulse = micros() - fwdStartTime;
   }
}

float filter(float prevValue, float currentValue, int filter)
{
   float lengthFiltered = (prevValue + (currentValue * filter)) / (filter + 1);
   return lengthFiltered;
}

int deadzoneThreshold(int pos)
{
   // get zero centre position (1000 to 2000 -> -500 to 500)
   pos = pos - 1500;

   // threshold value for control sticks
   auto threshold = 50;
   if (pos > threshold)
   {
      pos = pos - threshold;
   }
   else if (pos < -threshold)
   {
      pos = pos + threshold;
   }
   else
   {
      pos = 0;
   }

   pos = map(pos, -500 + threshold, 500 - threshold, 1000, 2000);
   pos = constrain(pos, 1000, 2000);

   return pos;
}

bool inputsValid(int turnInput, int fwdInput)
{
   if (fwdInput > 2050 || fwdInput < 950 || turnInput > 2050 || turnInput < 920)
   {
      return false;
   }
   return true;
}

void setup()
{
   // Debug LED pin
   pinMode(DEBUG_LED, OUTPUT);

   // Configure RX input pins
   pinMode(RX_FORWARD, INPUT);
   pinMode(RX_TURN, INPUT);

   // Initialize the Oar objects
   starboard.attachPins(STARBOARD_LIFT, STARBOARD_DRIVE);
   port.attachPins(PORT_LIFT, PORT_DRIVE);

   attachPinChangeInterrupt(digitalPinToPCINT(RX_TURN), updateTurn, CHANGE);
   attachPinChangeInterrupt(digitalPinToPCINT(RX_FORWARD), updateFwd, CHANGE);

   // Initialze to center
   forwardSignalAvg = 1500;
   turnSignalAvg = 1500;

   // Set oars to center
   starboard.reset();
   port.reset();

   // Turn on debug LED
   digitalWrite(DEBUG_LED, HIGH);
   delay(1000);
}

void loop()
{
   // Execute logic at start of the next frame
   auto currentMilis = millis();
   if (currentMilis >= frameExecuteTime)
   {
      // Check for valid input data
      if (!inputsValid(rxFwdPulse, rxTurnPulse))
      {
         return;
      }

      // Apply deadzone to inputs
      auto forwardSignal = deadzoneThreshold(rxFwdPulse);
      auto turnSignal = deadzoneThreshold(rxTurnPulse);

      // Calculate filtered average.
      forwardSignalAvg = filter(forwardSignalAvg, forwardSignal, 5);
      turnSignalAvg = filter(turnSignalAvg, turnSignal, 5);

      // If raw rx signals are low (no signal), reset oars
      // Else, normal operation
      if (forwardSignalAvg < 500 || turnSignalAvg < 500)
      {
         starboard.reset();
         port.reset();

         // Turn on debug LED
         digitalWrite(DEBUG_LED, HIGH);
      }
      else
      {
         // Map RX input to -1.0 <-> +1.0
         auto forwardInput = (forwardSignalAvg - 1500.0) / 500.0;
         auto turnInput = (turnSignalAvg - 1500.0) / 500.0;

         // Move the oars to the next step based on input
         starboard.step(forwardInput, turnInput);
         port.step(forwardInput, turnInput);

         // Turn off debug LED
         digitalWrite(DEBUG_LED, LOW);
      }

      // Set execute time to next milestone
      frameExecuteTime = currentMilis + FRAME_LENGTH;
   }
}
