/*

  Blinds_Control

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

  - Add button processor (will need to debounce, see example). Maybe try
    interrupt routines.

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

      // debouncer:
      check state of specified button
      if unpushed
  lastPushTime = 0
  if longPushFlag == true
    longPushFlag == false
    return LONG_PRESS_END
  fi
  return NO_CHANGE
      fi
      // from here on, the button is pressed
      if lastPushTime == 0 // button just pressed
  lastPushTime = currentTime
  return NO_CHANGE
      else if lastPushTime > 0
  if (currentTime -lastPushTime) > longPushTime // start of a long press
    longPushFlag = true
    return LONG_PRESS_START
  else if (currentTime - lastPushTime> > debounceTime // a normal press
    longPushFlag = false
    return PRESSED
  fi
  return NO_CHANGE
      if
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

const int closeButtonPin = D8;
int closeButtonState;
int closeButtonPreviousState = HIGH;
int closeButtonMillis;
int currentMillis;
// these should go into button processing function as statics
const int debounceMillis = 50;
const int longPressMillis = 1500;

// Values for two-phase, full-step operation (AB->BC->CD->DA)
const int fullStepValueCount = 4;
int TF_stepperValues1[4] = {  12,     6,    3,    9};
int TF_stepperValues2[4] = { 192,    96,   48,  144};
int TF_stepperValues3[4] = {3072,  1536,  768, 2304};

boolean isOpening = true;
//////////////////////////////////////////////////////////////////////////////
void setup() {

  Serial.begin(115200);

  pinMode(closeButtonPin, INPUT_PULLUP);

  pinMode(dataPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
} // setup()

//////////////////////////////////////////////////////////////////////////////
void loop() {

  // process button here for now, will move into separate procedure later.
  // closeButtonState = digitalRead(closeButtonPin);
  closeButtonState = ProcessButton(closeButtonPin);
  if (closeButtonState == LONG_PRESS_START) {
    Serial.println("LONG_PRESS_START");
  }
  else if (closeButtonState == LONG_PRESS_END) {
    Serial.println("LONG_PRESS_END");
  }
  else if (closeButtonState == PRESSED) {
    Serial.println("PRESSED");
  }
  // delay(500);




} // loop()
//////////////////////////////////////////////////////////////////////////////
void temp_move_closed() {
int stepIndex = 0;
uint16_t serialValue;
  delay(10);  // this value seems right after testing
  // delay(500); // uncomment to debug stepper signals
  // drive using one-phase full step first
  serialValue = TF_stepperValues1[stepIndex];  // get next value (bit pattern) to output
  // send value to shift register
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, serialValue);
  digitalWrite(latchPin, HIGH);
  // move stepper value index according to current direction mode
  if (isOpening) {
    stepIndex++;
    if (stepIndex >= fullStepValueCount) {
      stepIndex = 0;
    }
  } // if opening blinds
  else { // we must be closing the blinds
    stepIndex--;
    if (stepIndex < 0) {
      stepIndex = fullStepValueCount - 1;
    }
  }
} // temp_move_closed

// Processes specified button, returning a ButtonState enum value, adds debouncing.
ButtonState ProcessButton(int buttonPin) {
  static int debounceTime = 50;
  static int longPressTime = 1000;
  static int lastPushTime;

  static boolean longPushFlag;    // TRUE => long press in progress
  static boolean normalPushFlag; // TRUE => normal press in progress
  static boolean longPressInProgress; // TRUE => a long press has already started
  int pressDuration;    // how long has button been pressed

  int buttonState = digitalRead(buttonPin);
  if (buttonState == LOW) {   // if button not pressed
    lastPushTime = 0;
    if (normalPushFlag) {
      normalPushFlag = false;
      longPushFlag = false;
      if (longPressInProgress) {
        longPressInProgress = false;
        return LONG_PRESS_END;
      }
      else {
        return PRESSED;
      }
    }
    else if (longPushFlag) {
      longPushFlag = false;
      normalPushFlag = false;
      longPressInProgress = false;
      return LONG_PRESS_END;
    }
    return NO_CHANGE;
  } // if button not pressed
  // from this point onward, button is pressed
  if (lastPushTime == 0) {  // if button was just pressed
    lastPushTime = millis();
    return NO_CHANGE;
  }
  else if (lastPushTime > 0) {  // if button was in pressed state at last call
    pressDuration = millis() - lastPushTime;
    if (pressDuration > longPressTime) { // if start of a long press
      longPushFlag = true;
      if (longPressInProgress == false) {
        longPressInProgress = true;
        return LONG_PRESS_START;
      }
      return NO_CHANGE;
    }
    else if (pressDuration > debounceTime) { // if start of a normal press
      longPushFlag = false;
      normalPushFlag = true;
      longPressInProgress = false;
      return NO_CHANGE;
    }
  } // if button was in pressed state at last call 
  return NO_CHANGE;
} // ProcessButton

