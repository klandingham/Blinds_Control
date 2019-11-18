/*

  Blinds_Control (NodeMCU version)

  Controls 28BYJ-48 stepper motors via shift registers to
  open/close window blinds. See history.txt

  NOTE: For NodeMCU (and other boards) pin mapping, see:

    https://github.com/esp8266/Arduino/blob/master/doc/boards.rst#pin-mapping

  STATUS: IR commands now working. Note: due to the speed of IR reads, the open/close
  IR buttons have to work as toggles (open/close start/stop).

*/
#include "CodeSelect.h"
#ifdef NODE_MCU

// #define TRACE

#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>

// pin assignments
const int latchPin = D2;  // to pin 12 on the 75HC595
const int clockPin = D5;  // to pin 11 on the 75HC595
const int dataPin  = D7;  // to pin 14 on the 75HC595
const int calibrateLEDPin = D1;
const int closeButton = D8;
const int openButton  = D6;
const int calibrateButton = D0;
// const int IRpin = D3;

// An IR detector/demodulator is connected to GPIO pin 0(D3 on a NodeMCU board).
const uint16_t kRecvPin = 0;
IRrecv irrecv(kRecvPin);
decode_results results;
// unsigned long IRvalue;
uint64_t IRvalue;
// unsigned long lastIRvalue;
unsigned long IRCmdClose = 0x2FD807F;     // remote button 1
unsigned long IRCmdOpen = 0x2FD40BF;      // remote button 2
unsigned long IRCmdCalibrate = 0x2FDC03F; // remote button 3

// Motor drive modes
enum CommandState {
  OPEN,
  CLOSE,
  AUTO_OPEN,
  AUTO_CLOSE,
  STOP
};

// Last remove button pressed
enum IRLastPressed {
  IR_OPEN,
  IR_CLOSE,
  IR_CALIBRATE,
  IR_NONE
};

IRLastPressed lastIRButton = IR_NONE;

CommandState command = STOP;

int autoTimeInterval; // this is the measured time to move from opened to closed and vice-versa
int autoStartTime;    // time at which a manual OPEN is started - used to measure required auto interval

bool calibrating = false;
bool IRcalibrating = false;
bool IRcommandReceived = false;
unsigned calibrateFlashTime;
bool calibrateLEDOn = false;
unsigned lastLoopTime = 0;

// Values for two-phase, full-step operation (AB->BC->CD->DA)
const int fullStepValueCount = 4;
int TF_stepperValues1[4]   = {  12,     6,    3,    9};
int TF_stepperValues2[4]   = { 192,    96,   48,  144};
int TF_stepperValues3[4]   = {3072,  1536,  768, 2304};
int TF_stepperValues12[4]  = { 204,   102,   51,  153};
int TF_stepperValues123[4] = {3276,  1638,  819, 2457};
int stepIndex;

bool isOpening = true;

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

  Serial.begin(9600);

  irrecv.enableIRIn();  // Start the IR receiver
  while (!Serial)  // Wait for the serial connection to be established.
    delay(50);

  pinMode(closeButton, INPUT_PULLUP);
  pinMode(openButton, INPUT);
  pinMode(calibrateButton, INPUT);

  pinMode(dataPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(calibrateLEDPin, OUTPUT);

  command = STOP;
  calibrateFlashTime = 125; // mS
  digitalWrite(calibrateLEDPin, LOW);
  stepIndex = 0;
} // setup()

unsigned autoOpTime;
int calibrationStartTime;
boolean autoOpInProgress = false;
boolean calibrationInProgress = false;

int calibrateButtonState = 0;
int lastCalibrateButtonState = 0;
int closeButtonState = 0;
int lastCloseButtonState = 0;
int openButtonState = 0;
int lastOpenButtonState = 0;

//////////////////////////////////////////////////////////////////////////////
void loop() {

  // first thing, check for IR command
  IRvalue = -1;
  if (irrecv.decode(&results)) { // if IR code received
    IRvalue = results.value;
#ifdef TRACE
    // print() & println() can't handle printing long longs. (uint64_t)
    serialPrintUint64(results.value, HEX);
    Serial.println("");
#endif
    irrecv.resume();  // Receive the next value
    if (!IRcalibrating) {
      if (IRvalue == IRCmdOpen) {
        lastIRButton = IR_OPEN;
        command = AUTO_OPEN;
      }
      else if (IRvalue == IRCmdClose) {
        lastIRButton = IR_CLOSE;
        command = AUTO_CLOSE;
      }
      else if (IRvalue == IRCmdCalibrate) {
        IRcalibrating = true;
      }
#ifdef TRACE
      else if (IRvalue == 0xFFFFFFFFFFFFFFFF) {
        Serial.println("-- HOLD --");
      }
#endif
      else {
        lastIRButton = IR_NONE;
      }
    }
    /**
     * Note: I can't use "no value" to detect the release of a button, because
     * when holding down a remote button, "no value" reads are intermixed with
     * "FF..." values (i.e., the software is reading the IR sensor faster than
     * the remote can send out "FF..."'s). So, when in calibration mode, for now
     * I'll have to treat the remote open/close buttons as toggles (press once
     * to begin opening/closing, press again to stop).
     */ 
    else { // in calibration mode
      if (IRvalue == IRCmdCalibrate) {
        IRcalibrating = false; // toggle calibration mode off
      }
      else if (IRvalue == IRCmdClose) {
        if (command != CLOSE) {
          command = CLOSE;
        }
        else
        {
          command = STOP;
        }
      }
      else if (IRvalue == IRCmdOpen) {
        if (command != OPEN) { // we haven't been opening, start calibration
#ifdef TRACE
          Serial.println("Starting calibration...");
#endif
          calibrationInProgress = true;
          calibrationStartTime = millis();
          command = OPEN;
        }
        else {
#ifdef TRACE
          Serial.println("Ending calibration...");
#endif
          autoOpTime = millis() - calibrationStartTime;
          command = STOP;
          calibrationInProgress = false;
          IRcalibrating = false;
        }
      }
    } // in calibration mode
  } // if IR code received
  

  static int autoStartTime; // the time at which an auto-open/close op begins

  //
  //
  // Check state of calibrate button
  //
  calibrateButtonState = digitalRead(calibrateButton);
  if (calibrateButtonState != lastCalibrateButtonState) {
#ifdef TRACE
    Serial.println("CALIBRATE BUTTON");
#endif
    if (calibrateButtonState == HIGH) {
      calibrating = !calibrating;
    }
  }
  lastCalibrateButtonState = calibrateButtonState;
  //
  //
  // Flash calibration LED if in calibrate mode
  //
  if ((calibrating) || (IRcalibrating)) {
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
#ifdef TRACE
    Serial.println("CLOSE BUTTON");
#endif
    if (calibrating) {
      if (closeButtonState == HIGH) {
        command = CLOSE;
      }
      else {
        command = STOP;
      }
    }
    else { // else not calibrating
      if ((closeButtonState == HIGH) && (command != AUTO_CLOSE)) {
        command = AUTO_CLOSE;
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
    } // if button is pressed
    else { // else button is not pressed
      if (calibrationInProgress) { // if currently measuring
        autoOpTime = millis() - calibrationStartTime;
        command = STOP;
        calibrationInProgress = false;
        calibrating = false;
      } // end if currently measuring
    } // end else button is not pressed
  } // end if calibrating
  else { // else if not calibrating
    if (openButtonState != lastOpenButtonState) { // if button just changed state
#ifdef TRACE
      Serial.println("OPEN BUTTON");
#endif
      if ((openButtonState == HIGH) && (command != AUTO_CLOSE)) { // if button pressed and we're not already in AUTO_OPEN mode
        command = AUTO_OPEN;
      }
    } // end if button just changed state
  } // end else not calibrating
  lastOpenButtonState = openButtonState;

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
        autoOpInProgress = false;
        autoStartTime = 0;
        command = STOP;
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
      }
      else {
#ifdef TRACE
        Serial.println("Continuing auto op");
#endif
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
  // Note: If stepper driver lights come on, but motor does not turn, try adding
  // a delay here.
  delay(3);
} // loop()

#endif