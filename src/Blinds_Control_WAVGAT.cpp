/*

  Blinds_Control (WAVGAT version)

    8/19/19 - Previous error caused by my using ints to record elapsed times - they were rolling over to negative values and messing up the
    logic. Now have remote control working also.

    8/16/19 - Some kind of logic error is happening. After calibration, open/close buttons will work OK a few times, but eventually they stop
    working. Something similar happens if I press OPEN after an open operation has finished (may be the same error).

    8/14/19 - This is a work-in-progress. Modifying code to work with breadboarded WAVGAT UNO R3 (Chinese Arduino copy). Not much is working
    yet - right now trying to get pushbuttons to trigger properly.
*/
#include "CodeSelect.h"
#ifdef WAVGAT

// #define DEBUG_COMMAND
#include <Arduino.h>
#include <IRremote.h>

// function declarations
void TraceButton(uint8_t buttonID);

// pin assignments
const uint8_t latchPin = 2;  // pin 12 on the 75HC595
const uint8_t clockPin = 5;  // pin 11 on the 75HC595
const uint8_t dataPin  = 7;  // pin 14 on the 75HC595
const uint8_t calibrateLEDPin = 3;
const uint8_t closeButton = 8;
const uint8_t openButton  = 6;
const uint8_t calibrateButton = 4;
const uint8_t IRRecvPin = 9;

// Motor drive modes
enum CommandState {
  OPEN,
  CLOSE,
  AUTO_OPEN,
  AUTO_CLOSE,
  STOP
};

CommandState command = STOP;

unsigned long autoTimeInterval; // this is the measured time to move from opened to closed and vice-versa
unsigned long autoStartTime;    // time at which a manual OPEN is started - used to measure required auto interval

/*
  If loop is too fast, stepper motors may not be able to keep up. This delay is added to alleviate that.
  It will need to be tuned for optimum operation.
*/
unsigned long motorSequenceDelay = 7;  // milliseconds

bool calibrating = false;
unsigned long calibrateFlashTime;
bool calibrateLEDOn = false;
unsigned long lastLoopTime = 0;

// Values for two-phase, full-step operation (AB->BC->CD->DA)
const int fullStepValueCount = 4;
int TF_stepperValues1[4]   = {  12,     6,    3,    9};
int TF_stepperValues2[4]   = { 192,    96,   48,  144};
int TF_stepperValues3[4]   = {3072,  1536,  768, 2304};
int TF_stepperValues12[4]  = { 204,   102,   51,  153};
int TF_stepperValues123[4] = {3276,  1638,  819, 2457};
int stepIndex;

bool isOpening = true;

// define IR receiver and results objects
IRrecv irrecv(IRRecvPin);
decode_results results;
unsigned long IRvalue;
// unsigned long lastIRvalue;
unsigned long IRCmdClose = 0x2FD807F;
unsigned long IRCmdOpen = 0x2FD40BF;

//////////////////////////////////////////////////////////////////////////////
// If in the middle of a motor sequence and command becomes stop, we could be
// leaving one set of coils in the motors energized indefinitely. This method
// turns all coils off to ensure that doesn't happen.
void DeenergizeMotors() {
  digitalWrite(latchPin, LOW);              // ready shift register for data
  shiftOut(dataPin, clockPin, MSBFIRST, 0); // set all coil bits to 0
  digitalWrite(latchPin, HIGH);             // latch shift register (sends bits to motor(s))
} // DeenergizeMotors()

//////////////////////////////////////////////////////////////////////////////
// Sends next bit pattern to motor(s). Moves value index according to specified direction.
void SendNextSequence(boolean opening) {
  // Note: using a global for sequence (step) index so I can start it at 0 in setup()
  static uint16_t serialValue;
  // moving this delay to the main loop!  delay(10);  // delay for stepper motor...may need adjustment in final version
  // Note: will need to replace next statement with computation: add to final value
  // for each enabled motor; right now just driving motor 1
  serialValue = TF_stepperValues123[stepIndex];  // get next value (bit pattern) to output
  digitalWrite(latchPin, LOW);  // ready shift register for data
  shiftOut(dataPin, clockPin, MSBFIRST, serialValue); // send value to shift register
  digitalWrite(latchPin, HIGH);  // latch shift register (sends bits to motor(s))
  if (opening) {
    stepIndex++;
    if (stepIndex >= fullStepValueCount) {
      stepIndex = 0;
    }
  }
  else {
    stepIndex--;
    if (stepIndex < 0) {
      stepIndex = fullStepValueCount - 1;
    }
  }
} // SendNextSequence()

//////////////////////////////////////////////////////////////////////////////
void setup() {

  // Serial.begin(9600);

  pinMode(closeButton, INPUT);
  pinMode(openButton, INPUT);
  pinMode(calibrateButton, INPUT);

  pinMode(dataPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(calibrateLEDPin, OUTPUT);

  irrecv.enableIRIn();

  command = STOP;
  calibrateFlashTime = 125; // mS
  digitalWrite(calibrateLEDPin, LOW);
  stepIndex = 0;
} // setup()

unsigned long autoOpTime;
unsigned long calibrationStartTime;
boolean autoOpInProgress = false;
boolean calibrationInProgress = false;

int calibrateButtonState = 0;
int lastCalibrateButtonState = 0;
int closeButtonState = 0;
int lastCloseButtonState = 0;
int openButtonState = 0;
int lastOpenButtonState = 0;

#ifdef MY_DEBUG
  static long loopCount = 0;
#endif

//////////////////////////////////////////////////////////////////////////////
void loop() {

#ifdef MY_DEBUG
  // Serial.print("Loop: ");
  // Serial.println(loopCount++);
  // delay(500);
#endif

  // read IR code if there is one
  IRvalue = 0;
  if (irrecv.decode(&results)) {
    IRvalue = results.value;
    // Serial.println(results.value, HEX);
    irrecv.resume();
  }

  // debug button states
#ifdef MY_DEBUG
  TraceButton(closeButton);
  TraceButton(openButton);
  TraceButton(calibrateButton);
#endif
  //
  //
  // Check state of calibrate button
  //
  calibrateButtonState = digitalRead(calibrateButton);
  if (calibrateButtonState != lastCalibrateButtonState) {
    // Serial.println("CALIBRATE BUTTON");
    if (calibrateButtonState == HIGH) {
      calibrating = !calibrating;
#ifdef MY_DEBUG
      Serial.print("CALIBRATING: ");
      Serial.println(calibrating);
#endif
    }
  }
  lastCalibrateButtonState = calibrateButtonState;
  //
  //
  // Flash calibration LED if in calibrate mode
  //
  if (calibrating) {
    if (lastLoopTime == 0) {
      lastLoopTime = millis();
    }
    else if ((millis() - lastLoopTime) > calibrateFlashTime) {
      if (!(calibrateLEDOn)) {
        calibrateLEDOn = true;
        digitalWrite(calibrateLEDPin, HIGH);
      }
      else {
        calibrateLEDOn = false;
        digitalWrite(calibrateLEDPin, LOW);
      }
      lastLoopTime = 0;
    }
  }
  else {
    calibrateLEDOn = false;
    digitalWrite(calibrateLEDPin, LOW);
  }
  //
  //
  // Read CLOSE button. If in calibrate mode, keep command at
  // CLOSE as long as button is pressed (HIGH), set command to
  // STOP on release. If in normal mode, if command is not
  // AUTO_CLOSE, set it to that, ignore release.
  //
  closeButtonState = digitalRead(closeButton);
  if (closeButtonState != lastCloseButtonState) {
    // Serial.println("CLOSE BUTTON");
    if (calibrating) {
      if (closeButtonState == HIGH) {
        command = CLOSE;
        // Serial.println("COMMAND set to CLOSE");
      }
      else {
        command = STOP;
        // Serial.println("COMMAND set to STOP");
      }
    }
    else { // else not calibrating
      if ((closeButtonState == HIGH) && (command != AUTO_CLOSE)) {
        command = AUTO_CLOSE;
        // Serial.println("COMMAND set to AUTO_CLOSE");
      }
    }
  }
  lastCloseButtonState = closeButtonState;
  //
  //
  // Read OPEN button. If in calibrate mode and button pressed,
  // start calibration if not already started.
  //
  openButtonState = digitalRead(openButton);
  if (calibrating) {
    if (openButtonState == HIGH) { // if button is pressed
      if (!calibrationInProgress) { // if not measuring yet
        calibrationInProgress = true;
        calibrationStartTime = millis();
      } // end if not measuring yet
      command = OPEN;
      // Serial.println("COMMAND set to OPEN");
    } // if button is pressed
    else { // else button is not pressed
      if (calibrationInProgress) { // if currently measuring
        autoOpTime = millis() - calibrationStartTime;
        command = STOP;
        // Serial.println("COMMAND set to STOP");
        calibrationInProgress = false;
        calibrating = false;
      } // end if currently measuring
    } // end else button is not pressed
  } // end if calibrating
  else { // else if not calibrating
    if (openButtonState != lastOpenButtonState) { // if button just changed state
      if ((openButtonState == HIGH) && (command != AUTO_OPEN)) { // if button pressed and we're not already in AUTO_OPEN mode
        command = AUTO_OPEN;
      }
    } // end if button just changed state
  } // end else not calibrating
  lastOpenButtonState = openButtonState;
  //
  //
  // Check for IR command
  //
  if (!calibrating) { // ignore IR commands when calibrating
    if (IRvalue != 0) { // if IR command received
      if ((IRvalue == IRCmdClose) && (command != AUTO_CLOSE)) {
        command = AUTO_CLOSE;
      }
      else if ((IRvalue == IRCmdOpen) && (command != AUTO_OPEN)) {
        command = AUTO_OPEN;
      }
    } // if IRvalue has changed
  } // if (!calibrating)
  //
  //
  // Command processing. Note we only measure time during an OPEN
  // operation.
  //
  if (command == STOP) {
    DeenergizeMotors();
  }
  else if (command == AUTO_CLOSE) {
    if (!(autoOpInProgress)) { // if this is new auto command
      autoOpInProgress = true;
      autoStartTime = millis();
      SendNextSequence(false);
    } // if this is new auto command
    else { // else an auto open/close is already in progress
      if ((millis() - autoStartTime) >= autoOpTime) { // if auto time satisfied
        // Serial.print("autoStartTime = ");
        // Serial.println(autoStartTime);
        // Serial.print("autoOpTime = ");
        // Serial.println(autoOpTime);

        autoOpInProgress = false;
        autoStartTime = 0;
        command = STOP;
        // Serial.println("COMMAND set to STOP");
      }
      else {
        SendNextSequence(false);
      }
    }
  } // if AUTO_CLOSE
  else if (command == AUTO_OPEN) {
    if (!(autoOpInProgress)) { // if this is new auto command
      autoOpInProgress = true;
      autoStartTime = millis();
      SendNextSequence(true);
    } // if this is new auto command
    else { // else an auto open/close is already in progress
      if ((millis() - autoStartTime) >= autoOpTime) { // if auto time satisfied
        autoOpInProgress = false;
        autoStartTime = 0;
        command = STOP;
        // Serial.println("COMMAND set to STOP");
      }
      else {
        //      Serial.println("Continuing auto op");
        SendNextSequence(true);
      }
    }
  } // if AUTO_OPEN
  else if (command == OPEN) {
    SendNextSequence(true);
  }
  else if (command == CLOSE) {
    SendNextSequence(false);
  }
  // Note: had a delay in here because the steppers were skipping. Increased the
  // stepper power voltage from 5 to 12V. No delay is necessary now (the stepper
  // motors can keep up with the code.)
  delay(motorSequenceDelay);
#ifdef MY_DEBUG
  //Serial.print("At bottom of loop, COMMAND = ");
  //Serial.println(command);
#endif
} // loop()

/**
 * Output button state to serial (for debugging)
 */
void TraceButton(uint8_t buttonID) {
  int buttonState = digitalRead(buttonID);
  String buttonStateText = buttonState ? "HIGH" : "LOW";
  // Serial.print("Button pin ");
  // Serial.print(buttonID);
  // Serial.print(" is ");
  // Serial.println(buttonStateText);
}
#endif