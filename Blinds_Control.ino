/*

  Blinds_Control

        New issue: with the 12V power supply I chose, when the circuit is first
        switched on, it apparently is drawing a large enough initial surge of
        current to trip the overcurrent protection inside the wall transformer.
        The result is that the power begins to pulse on and off (about once
        every two seconds) and the circuit doesn't function. I've noticed that
        when first switched on, all the stepper driver board LED's flash which
        suggests that they are all being switched on at power up. I also noticed
        that once the PS goes into its power-trip cycle, if I disconnect Vcc
        from the first 74595, the cycling stops...I can they reconnect the shift
        register, and the circuit behaves normally. So it seems I need some sort
        of delay to enable the shift register outputs once the circuitry is
        stable after power up.

        Right now I have the output enable (OE) lines (active low) pins hard-wired
        to ground (i.e. the register outputs are instantly enabled). I'm going
        to try using one of the NodeMCU pins to enable these pins on startup,
        perhaps after one second. Hopefully this will get rid of this initial
        surge issue. THIS DIDN'T WORK...I'll just add a power switch for each
        stepper driver board - was going to do that anyway.

  v0.9: Finally got to test this on a real window blind! Two issues surfaced:

        (1) I want to use 8-conductor Ethernet cable to get signals and power to
        each stepper module (cheap). As an initial test, I reconnected one of
        the modules using a 6 foot length of this cable. Unfortunately, the
        conductor gauge used in this cable, although adequate to carry network
        signals a considerable distance, falls short when asked to carry any
        appreciable amount of current. I found that while driving the stepper,
        the 5V supplied at the board had dropped to around 3.5V by at the
        stepper module (a considerable loss), and that is with only a 6 ft
        length. Cable to the farthest window will result in an even greater
        amount of voltage loss. At this reduced voltage, the stepper motor
        was VERY weak. To solve this, I am using a two-fold approach. First,
        I have replaced the 5V power supply with a 12V one (the maximum
        rated input voltage for the stepper modules). Second, when assembling
        the longer cables, since the cable I am using has 8 conductors and I
        am only using 6, I will double-up the cables on the 2 power pins (that
        is, crimp 2 conductors into each of the 2 power connectors).

        (2) This particular issue could be a show-stopper! Testing the current
        implementation with a real window blind, I quickly realized that the
        stepper motors are rotating far too slowly to be practical. The number
        of complete rotations necessary to open the blinds seems to be around
        4 to 5, but with my current implementation that could take several
        minutes of holding down a button - not usable. I am going to try to
        modify the timings in the sketch to try to get the steppers to rotate
        faster, BUT I realize that there is a limit to how fast they will rotate
        reliably. Hopefully upping the power supply voltage (see #1 above) will
        help. If I CANNOT get the steppers to rotate fast enough to be usable,
        I will have to go back to using servo motors - that will require not
        only changing the code but also creating new parts!

        UPDATE: Increasing the stepper power supply voltage from 5 to 12V seems
        to have solved the speed issue - I have been able to remove the loop
        delay and all 3 steppers seem able to keep up with the code.

  v0.8: Modified sequence output values to drive 3rd motor.
  
  v0.7: Decided on using toggle switches to select motors.
        Modified sequence output values to drive 2nd motor.

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
int TF_stepperValues1[4]   = {  12,     6,    3,    9};
int TF_stepperValues2[4]   = { 192,    96,   48,  144};
int TF_stepperValues3[4]   = {3072,  1536,  768, 2304};
int TF_stepperValues12[4]  = { 204,   102,   51,  153};
int TF_stepperValues123[4] = {3276,  1638,  819, 2457};
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
} // loop()
