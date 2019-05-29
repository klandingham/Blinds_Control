/*

  Blinds_Control

  v0.6: Woo-hoo! Button logic FINALLY working:
    To calibrate:
      1. Press and hold CLOSE button until blinds are closed.
      2. Press CALIBRATE button (LED will blink).
      3. Press and hold OPEN button until blinds are open. (LED stops)
    To use:
      Press either OPEN or CLOSE button to open/close blinds!

  NEXT: Need to figure out how to select motors. Presents a problem because
  I want to show which are selected with LED's, but I don't have enough open
  pins left on the NodeMCU. Thinking about using some sort of external logic
  chip to do the selection sequence, but it's not going to be binary...it will
  need to be something like:

        0 0 0  (I guess this would be "all motors off"...entire thing disabled)
        0 0 1
        0 1 0
        1 0 0
        0 1 1
        1 1 0
        1 1 1
  Also considering a simpler hardware solution: a small toggle switch for each
  motor.
        

  v0.5: Replacing excessively complex button logic with simpler approach:
  Add one more button "Calibrate"...press this to go into calibrate (CAL) mode
  (a CAL LED flashes), optional: press and hold CLOSE to move blinds to closed,
  press and hold OPEN until blinds at desired position, release OPEN button (I save
  duration), press CAL button to exit calibration mode. From this point onward, OPEN
  and CLOSE buttons use the measure duration. Addition: keep track of OPEN or CLOSED
  state. When not in CAL mode, if blinds are OPEN, ignore OPEN button, likewise for
  CLOSE button.

  v0.4: Added button processing logic - but getting bad interactions between the buttons.
  The button processing with long-press detection is just too complex. I've decided to
  use a different approach. See next version.

  v0.3: Decided that using interrupts to handle button presses is overkill.
  Restructured the code so that the main loop handles controls (buttons,
  etc.) processing, and refactoring the stepper driver code. Added function
  to process button states at top of main loop. Loop currently contains
  debugging code for this function.

  v0.2: Added pushbutton interrupt handler. Currently changes direction
  of stepper motor, just as a test.

  v0.1: Basic stepper drive logic. Still has code from base example
  that needs to be cleaned out.

  This sketch controls 28BYJ-48 stepper motors via a shift register to
  open/close window blinds.

*/

// 74HC595 control pins
const int latchPin = D2;  // pin 12 on the 75HC595
const int clockPin = D5;  // pin 11 on the 75HC595
const int dataPin  = D7;  // pin 14 on the 75HC595

// CALibrate related pins
const int calibrateLEDPin = D1;

// Motor drive modes
enum CommandState {
  OPEN,
  CLOSE,
  AUTO_OPEN,
  AUTO_CLOSE,
  STOP
};

CommandState command = STOP;

int autoTimeInterval; // this is the measured time to move from opened to closed and vice-versa
int autoStartTime;    // time at which a manual OPEN is started - used to measure required auto interval

const int closeButton = D8;
const int openButton  = D6;
const int calibrateButton = D0;

boolean calibrating = false;
int calibrateFlashTime;
boolean calibrateLEDOn = false;
int lastLoopTime = 0;

// Values for two-phase, full-step operation (AB->BC->CD->DA)
const int fullStepValueCount = 4;
int TF_stepperValues1[4] = {  12,     6,    3,    9};
int TF_stepperValues2[4] = { 192,    96,   48,  144};
int TF_stepperValues3[4] = {3072,  1536,  768, 2304};
int stepIndex;

boolean isOpening = true;

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
  Serial.print("SendNextSequence: stepIndex = ");
  Serial.println(stepIndex);
  // Note: using a global for sequence (step) index so I can start it at 0 in setup()
  static uint16_t serialValue;
  // moving this delay to the main loop!  delay(10);  // delay for stepper motor...may need adjustment in final version
  // Note: will need to replace next statement with computation: add to final value
  // for each enabled motor; right now just driving motor 1
  serialValue = TF_stepperValues1[stepIndex];  // get next value (bit pattern) to output
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

  Serial.begin(115200);

  pinMode(closeButton, INPUT_PULLUP);
  pinMode(openButton, INPUT_PULLUP);
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

int autoOpTime;
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
  static int autoStartTime; // the time at which an auto-open/close op begins

  //
  //
  // Check state of calibrate button
  //
  calibrateButtonState = digitalRead(calibrateButton);
  if (calibrateButtonState != lastCalibrateButtonState) {
    if (calibrateButtonState == HIGH) {
      calibrating = !calibrating;
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


  // TODO: handle OPEN button (must do timing)

/*
  // while calibrating, don't need to know if button changed,
  // just need to know if it's pressed or not
  if calibrating
    if button HIGH (pressed)
      if already measuring
        open -> command
      else
        true -> measuring
        set start time
        open -> command
      fi
    else // button is LOW
      if already measuring // always true?
        duration = now - start time
        stop -> command
        false -> measuring
        false -> calibrating
      fi
      // ? need case for not already measuring?
    end else button is LOW
  end if calibrating
  else // not calibrating
    false -> measuring
    0 -> start time
    handle OPEN button as in CLOSE case
*/

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
      //    Serial.println("New AUTO_OPEN started");
      autoOpInProgress = true;
      autoStartTime = millis();
      //    Serial.print("autoStartTime = ");
      //    Serial.println(autoStartTime);
      SendNextSequence(true);
    } // if this is new auto command
    else { // else an auto open/close is already in progress
      if ((millis() - autoStartTime) >= autoOpTime) { // if auto time satisfied
        //      Serial.println("Stopping auto op");
        autoOpInProgress = false;
        autoStartTime = 0;
        command = STOP;
      }
      else {
        //      Serial.println("Continuing auto op");
        SendNextSequence(true);
      }
    }
  } // if AUTO_OPEN
  else if (command == OPEN) {
    // TODO: need to add logic here to measure duration. Maybe use a "measuringDuration" boolean
    // to detect the first loop through here.
    SendNextSequence(true);
  }
  else if (command == CLOSE) {
    SendNextSequence(false);
  }
  delay(10);
} // loop()

  /*
  // + + + DEBUG
  switch (command) {
    case STOP:
      Serial.println("STOP");
      break;
    case OPEN:
      Serial.println("OPEN");
      break;
    case CLOSE:
      Serial.println("CLOSE");
      break;
    case AUTO_OPEN:
      Serial.println("AUTO_OPEN");
      break;
    case AUTO_CLOSE:
      Serial.println("AUTO_CLOSE");
      break;
    default:
      Serial.print("ERROR! command has undefined value: ");
      Serial.println(command);
  } // end switch(command)
  // - - - DEBUG
  */
