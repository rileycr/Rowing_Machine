#include <Arduino.h>
#include <Servo.h>

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
const int FRAME_FREQUENCY = 45;
const int FRAME_LENGTH = 1000 / FRAME_FREQUENCY;

// Frame Timing
unsigned long currentTime = 0;
unsigned long frameExecuteTime = 0;

// Declare the Digital Oar objects
Doar starboard(true);
Doar port(false);

// Debug LED pin
bool ledOn = true;

// RX input signals
unsigned long forwardSignalAvg;
unsigned long turnSignalAvg;

void setup()
{
   // Initialize input values
   forwardSignalAvg = 1500;
   turnSignalAvg = 1500;

   // Frame timing
   frameExecuteTime = millis();

   // Configure RX input pins
   pinMode(RX_FORWARD, INPUT);
   pinMode(RX_TURN, INPUT);

   // Debug LED pin
   pinMode(DEBUG_LED, OUTPUT);

   // Initialize the Oar objects
   starboard.attachPins(STARBOARD_LIFT, STARBOARD_DRIVE);
   port.attachPins(PORT_LIFT, PORT_DRIVE);
}

void loop()
{
   // Average the last X frames
   auto weight = 5;

   // Getting RX signals, values 1000 to 2000
   auto rxForwardSignal = pulseIn(RX_FORWARD, HIGH);
   auto rxTurnSignal = pulseIn(RX_TURN, HIGH);

   // Calculate running average.
   forwardSignalAvg = (forwardSignalAvg * (weight - 1) + rxForwardSignal) / weight;
   turnSignalAvg = (turnSignalAvg * (weight - 1) + rxTurnSignal) / weight;

   // Execute logic at start of the next frame
   if (millis() >= frameExecuteTime)
   {
      //if (ledOn)
      //{
      //   digitalWrite(DEBUG_LED, LOW);
      //   ledOn = false;
      //}
      //else
      //{
      //   digitalWrite(DEBUG_LED, HIGH);
      //   ledOn = true;
      //}

      // If raw rx signals are low (no signal), reset oars
      // Else, normal operation
      if (rxForwardSignal < 500 || rxTurnSignal < 500)
      {
         starboard.reset();
         port.reset();
      }
      else
      {
         // Map RX input to -1.0 <-> +1.0
         auto forwardInput = (forwardSignalAvg - 1500.0) / 500.0;
         auto turnInput = (turnSignalAvg - 1500.0) / 500.0;

         // Move the oars to the next step based on input
         starboard.step(forwardInput, turnInput);
         port.step(forwardInput, turnInput);
      }

      // Set execute time to next milestone
      frameExecuteTime += FRAME_LENGTH;
   }
}
