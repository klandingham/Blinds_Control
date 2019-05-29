/*

  Blinds_Control

  v0.4: Added button processing logic - but getting bad interactions between the buttons.
  The button processing with long-press detection is just too complex. I've decided to
  use a different approach: add one more button "Calibrate"...press this to go into
  calibrate mode, then press and hold open/close button to move and begin timer. When
  open/close button released, stop motor, save duration, and exit calibration mode.
  While in calibration mode, a LED should blink.

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

  TODO:

  - Move stepper drive code into callable routines:

      StepMotors(int milliseconds, enum direction)
        milliseconds: how long to move (measured elsewhere)
        direction: OPEN or CLOSE

      StartMotors(enum direction)

      StopMotors()

      Note: Use one global to hold open time. May be able to also use
      one global to keep track of current output index.

  First pass at button processing logic:

  - use global flag for constant motor movement

  loop()
      // Algorithm: uses a separate debouncing method that returns PRESSED, LONG_PRESS_START, LONG_PRESS_END, NO_CHANGE

      // main loop processing
      buttonState = getButtonState(closeButton)
      if (buttonState == PRESS)
  closeBlinds()
      else if (buttonState == LONG_PRESS_START)
  startContinuousClose()
      else if (buttonState == LONG_PRESS_END)
  stopContinuousClose()
*/

// 74HC595 control pins
const int latchPin = D2;  // pin 12 on the 75HC595
const int clockPin = D5;  // pin 11 on the 75HC595
const int dataPin  = D7;  // pin 14 on the 75HC595

// Possible button states
enum ButtonState {
  PRESSED,
  LONG_PRESS_START,
  LONG_PRESS_END,
  NO_CHANGE
};

// Motor drive modes
enum CommandState {
  OPEN,
  CLOSE,
  AUTO_OPEN,
  AUTO_CLOSE,
  STOP
};

// Button currently "active" (being processed)
enum ActiveButton {
  OPEN_BUTTON,
  CLOSE_BUTTON,
  NONE
};

CommandState command = STOP;
CommandState previousCommand = STOP;

int autoTimeInterval; // this is the measured time to move from opened to closed and vice-versa
int autoStartTime;    // time at which a manual OPEN is started - used to measure required auto interval

const int closeButtonPin = D8;
const int openButtonPin  = D6;
ActiveButton activeButton = NONE;

ButtonState closeButtonState;
ButtonState openButtonState;

boolean processInput;  // used to inhibit input processing during timed operations

// Values for two-phase, full-step operation (AB->BC->CD->DA)
const int fullStepValueCount = 4;
int TF_stepperValues1[4] = {  12,     6,    3,    9};
int TF_stepperValues2[4] = { 192,    96,   48,  144};
int TF_stepperValues3[4] = {3072,  1536,  768, 2304};
uint8_t stepIndex;

boolean isOpening = true;

//////////////////////////////////////////////////////////////////////////////
// Sends next bit pattern to motor(s). Moves value index according to specified direction.
void SendNextSequence(CommandState command) {
  // Note: using a global for sequence (step) index so I can start it at 0 in setup()
  static uint16_t serialValue;
  delay(10);  // delay for stepper motor...may need adjustment in final version
  // Note: will need to replace next statement with computation: add to final value
  // for each enabled motor; right now just driving motor 1
  serialValue = TF_stepperValues1[stepIndex];  // get next value (bit pattern) to output
  digitalWrite(latchPin, LOW);  // ready shift register for data
  shiftOut(dataPin, clockPin, MSBFIRST, serialValue); // send value to shift register
  digitalWrite(latchPin, HIGH);  // latch shift register (sends bits to motor(s))
  if (command == OPEN) {
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

  pinMode(closeButtonPin, INPUT_PULLUP);
  pinMode(openButtonPin, INPUT_PULLUP);

  pinMode(dataPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);

  previousCommand = STOP;
  processInput = true;
  stepIndex = 0;
} // setup()

int openButtonValue;
int openButtonPressStart;
int openButtonPressDuration;
boolean openButtonPressed;

int closeButtonValue;
int closeButtonPressStart;
int closeButtonPressDuration;
boolean closeButtonPressed;


int autoOpTime;
boolean autoOpInProgress = false;

//////////////////////////////////////////////////////////////////////////////
void loop() {
  static int autoStartTime; // the time at which an auto-open/close op begins

  // Button processing
  // TODO: this code needs to be refactored into one procedure that handles any button
  // Open button:
  if (activeButton != CLOSE_BUTTON) {
    openButtonValue = digitalRead(openButtonPin);
    if (openButtonValue == LOW) { // if button pressed
      activeButton = OPEN_BUTTON;
      openButtonPressed = true;
      // start timer if not already running
      if (openButtonPressStart == 0) {
        //      Serial.println("Starting timer!");
        openButtonPressStart = millis();  // start timer
        //      Serial.print("openButtonPressStart = ");
        //      Serial.println(openButtonPressStart);
      }
      else if (openButtonPressStart > 0) { // if timer running
        if ((millis() - openButtonPressStart) >= 1000) { // if button has been LOW long enough
          command = OPEN;
          //        openButtonPressStart = 0;
        } // if button has been LOW long enough
      }
    } // if button pressed
    else if (openButtonValue == HIGH) { // if button released
      activeButton = NONE;
      if (command == OPEN) {  // if we are opening right now, stop
        //      Serial.println("command = OPEN when button released");
        autoOpTime = millis() - openButtonPressStart;
        //      Serial.print("Auto open time = ");
        //      Serial.print(autoOpTime);
        //      Serial.println(" milliseconds.");
        command = STOP;
        openButtonPressStart = 0;
        openButtonPressed = false;
      }  // if we are opening right now, stop
      else if (openButtonPressed) { // if transitioning to release from previous press
        //      Serial.println("Button release detected");
        openButtonPressed = false;
        openButtonPressDuration = millis() - openButtonPressStart;
        //      Serial.print("openButtonPressStart = ");
        //      Serial.println(openButtonPressStart);
        //      Serial.print("openButtonPressDuration = ");
        //      Serial.println(openButtonPressDuration);
        if (openButtonPressDuration < 1000) { // if press-to-release time less than long press time
          command = AUTO_OPEN;
          openButtonPressStart = 0;
        } // if press-to-release time less than long press time
      } // else
    } // if button released
  } // if activeButton != CLOSE_BUTTON

  // Close button:
  // Note: bad interactions occur if I simply read the close button without checking if I'm in
  // the middle of processing the open button.  Need to gate processing based on state.
  //
  if (activeButton != OPEN_BUTTON) {
    closeButtonValue = digitalRead(closeButtonPin);
    if (closeButtonValue == LOW) { // if button pressed
      activeButton = CLOSE_BUTTON;
      closeButtonPressed = true;
      // start timer if not already running
      if (closeButtonPressStart == 0) {
        closeButtonPressStart = millis();  // start timer
      }
      else if (closeButtonPressStart > 0) { // if timer running
        if ((millis() - closeButtonPressStart) >= 1000) { // if button has been LOW long enough
          command = CLOSE;
        } // if button has been LOW long enough
      }
    } // if button pressed
    else if (closeButtonValue == HIGH) { // if button released
      activeButton = NONE;
      if (command == CLOSE) {  // if we are closing right now, stop
        autoOpTime = millis() - closeButtonPressStart;
        command = STOP;
        closeButtonPressStart = 0;
        closeButtonPressed = false;
      }  // if we are closing right now, stop
      else if (closeButtonPressed) { // if transitioning to release from previous press
        closeButtonPressed = false;
        closeButtonPressDuration = millis() - closeButtonPressStart;
        if (closeButtonPressDuration < 1000) { // if press-to-release time less than long press time
          command = AUTO_CLOSE;
          closeButtonPressStart = 0;
        } // if press-to-release time less than long press time
      } // else
    } // if button released
  } // if activeButton != OPEN_BUTTON





  // Command processing
  if (command == AUTO_CLOSE) {
    if (!(autoOpInProgress)) { // if this is new auto command
      autoOpInProgress = true;
      autoStartTime = millis();
      SendNextSequence(CLOSE);
    } // if this is new auto command
    else { // else an auto open/close is already in progress
      if ((millis() - autoStartTime) >= autoOpTime) { // if auto time satisfied
        autoOpInProgress = false;
        autoStartTime = 0;
        command = STOP;
      }
      else {
        SendNextSequence(CLOSE);
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
      SendNextSequence(OPEN);
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
        SendNextSequence(OPEN);
      }
    }
  } // if AUTO_OPEN
  else if (command == OPEN) {
    SendNextSequence(OPEN);
  }
  else if (command == CLOSE) {
    SendNextSequence(CLOSE);
  }
  /*
    else if (command = STOP) {
    if (previousCommand == OPEN) {
    autoTimeInterval = millis() - autoStartTime;
    autoStartTime = 0;
    previousCommand = STOP;
    }
    }
  */
} // loop()

// // Sends next bit pattern to motor(s). Moves value index according to specified direction.
// void SendNextSequence(CommandState command) {
//   // Note: using a global for sequence (step) index so I can start it at 0 in setup()
//   static uint16_t serialValue;
//   delay(10);  // delay for stepper motor...may need adjustment in final version
//   // Note: will need to replace next statement with computation: add to final value
//   // for each enabled motor; right now just driving motor 1
//   serialValue = TF_stepperValues1[stepIndex];  // get next value (bit pattern) to output

//   shiftOut(dataPin, clockPin, MSBFIRST, serialValue); // send value to shift register
//   digitalWrite(latchPin, HIGH);  // latch shift register (sends bits to motor(s))
//   if (command == OPEN) {
//     stepIndex++;
//     if (stepIndex >= fullStepValueCount) {
//       stepIndex = 0;
//     }
//   }
//   else {
//     stepIndex--;
//     if (stepIndex < 0) {
//       stepIndex = fullStepValueCount - 1;
//     }
//   }
// } SendNextSequence()



// Processes specified button, returning a ButtonState enum value, adds debouncing.
// ButtonState ProcessButton(int buttonPin) {
//   static int debounceTime = 50;
//   static int longPressTime = 1000;
//   static int lastPushTime;
//
//   static boolean longPushFlag;    // TRUE => long press in progress
//   static boolean normalPushFlag; // TRUE => normal press in progress
//   static boolean longPressInProgress; // TRUE => a long press has already started
//   int pressDuration;    // how long has button been pressed
//
//   int buttonState = digitalRead(buttonPin);
//   if (buttonState == LOW) {   // if button not pressed
//     lastPushTime = 0;
//     if (normalPushFlag) {
//       normalPushFlag = false;
//       longPushFlag = false;
//       if (longPressInProgress) {
//         longPressInProgress = false;
//         return LONG_PRESS_END;
//       }
//       else {
//         return PRESSED;
//       }
//     }
//     else if (longPushFlag) {
//       longPushFlag = false;
//       normalPushFlag = false;
//       longPressInProgress = false;
//       return LONG_PRESS_END;
//     }
//     return NO_CHANGE;
//   } // if button not pressed
//   // from this point onward, button is pressed
//   if (lastPushTime == 0) {  // if button was just pressed
//     lastPushTime = millis();
//     return NO_CHANGE;
//   }
//   else if (lastPushTime > 0) {  // if button was in pressed state at last call
//     pressDuration = millis() - lastPushTime;
//     if (pressDuration > longPressTime) { // if start of a long press
//       longPushFlag = true;
//       if (longPressInProgress == false) {
//         longPressInProgress = true;
//         return LONG_PRESS_START;
//       }
//       return NO_CHANGE;
//     }
//     else if (pressDuration > debounceTime) { // if start of a normal press
//       longPushFlag = false;
//       normalPushFlag = true;
//       longPressInProgress = false;
//       return NO_CHANGE;
//     }
//   } // if button was in pressed state at last call
//   return NO_CHANGE;
// } // ProcessButton

//////////////////////////////////////////////////////////////////////////////
// void temp_move_closed() {
// int stepIndex = 0;
// uint16_t serialValue;
//   delay(10);  // this value seems right after testing
//   // delay(500); // uncomment to debug stepper signals
//   // drive using one-phase full step first
//   serialValue = TF_stepperValues1[stepIndex];  // get next value (bit pattern) to output
//   // send value to shift register
//   digitalWrite(latchPin, LOW);
//   shiftOut(dataPin, clockPin, MSBFIRST, serialValue);
//   digitalWrite(latchPin, HIGH);
//   // move stepper value index according to current direction mode
//   if (isOpening) {
//     stepIndex++;
//     if (stepIndex >= fullStepValueCount) {
//       stepIndex = 0;
//     }
//   } // if opening blinds
//   else { // we must be closing the blinds
//     stepIndex--;
//     if (stepIndex < 0) {
//       stepIndex = fullStepValueCount - 1;
//     }
//   }
// } // temp_move_closed
