/*

  Blinds_Control

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





  For details on usage, wiring, etc. see link below.
  http://projectsfromtech.blogspot.com/2014/03/arduino-shift-register-stepper-motor.html

  Last Edited: 3/1/2014
  by Matthew https://projectsfromtech.blogspot.com

  Thanks to DrLuke from http://bildr.org/2011/02/74hc595/
  for Shift Register Control code
*/

const int dataPin = D7;   //pin 14 on the 75HC595
const int latchPin = D2;  //pin 12 on the 75HC595
const int clockPin = D5; //pin 11 on the 75HC595

const byte interruptPin = 15; // ESP8266 pin 16 = NodeMCU pin D8

//How many of the shift registers - change this
//#define number_of_74hc595s 1

//do not touch
//#define numOfRegisterPins number_of_74hc595s * 8

//boolean registers[numOfRegisterPins];


// uint8_t GPIO_Pin = D2;
// 
// void setup() {
//  Serial.begin(9600);
//  attachInterrupt(digitalPinToInterrupt(GPIO_Pin), IntCallback, RISING);
// }
// 
// void loop() {
// }

// void IntCallback(){
//  Serial.print("Stamp(ms): ");
//  Serial.println(millis());
// }




/*
   Stepper Driver Values

   These values will be converted to bits (shiftOut) and sent to 74HC595
   which is then latched applying the bit pattern to the stepper controller

   these values use 16 bits (4 per stepper). for more steppers, cascade
   another 74HC595 and increase to 12 bits.

   Note: use addition to move multiple steppers at once, i.e. 4+12, etc.)

   Based on research, two-phase full-step mode gives the most torque.
*/

const int fullStepValueCount = 4;
const int halfStepValueCount = 8;

// Values for one-phase, full-step operation (A->B->C->D)
int OF_stepperValues1[4] = {   8,     4,    2,    1};
int OF_stepperValues2[4] = { 128,    64,   32,   16};
int OF_stepperValues3[4] = {2048,  1024,  512,  256};

// Values for two-phase, full-step operation (AB->BC->CD->DA)
int TF_stepperValues1[4] = {  12,     6,    3,    9};
int TF_stepperValues2[4] = { 192,    96,   48,  144};
int TF_stepperValues3[4] = {3072,  1536,  768, 2304};

// Values for one/two-phase, half-step operation (AB->B->BC->C->CD->D->DA->A)
int TH_stepperValues1[8] = {  12,    4,    6,    2,    3,    1,     9,     8};
int TH_stepperValues2[8] = { 192,   64,   96,   32,   48,   16,   144,   128};
int TH_stepperValues3[8] = {3072, 1024, 1536,  512,  768,  256,  2304,  2048};

boolean isOpening = true;

void handleInterrupt() {
//  interruptCounter++;
  // Serial.print("Stamp(ms): ");
  // Serial.println(millis());
  // test: reverse direction
  isOpening = !isOpening;
}

//void testPBHandler() {
//  Serial.print("Stamp(ms): ");
//  Serial.println(millis());
//}

//////////////////////////////////////////////////////////////////////////////
void setup() {

  Serial.begin(115200);

  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);

  pinMode(dataPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);


} // setup()

int stepIndex = 0;
uint16_t serialValue;
//////////////////////////////////////////////////////////////////////////////
void loop() {
  // potValue = analogRead(potPin);     // read the value of the potentiometer
  ///potValue = map(potValue, 25, 1023, 0, 200);
  // Serial.println(potValue);          // View full range from 0 - 1024 in Serial Monitor
  // delayMicroseconds(potValue);
  // delay(potValue); // leaving in for now in case I need to adjust final version
  delay(10);  // this value seems right after testing
  // delay(500); // uncomment to debug stepper signals
  // drive using one-phase full step first
  serialValue = TF_stepperValues1[stepIndex];	// get next value (bit pattern) to output
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
  Serial.print("stepIndex = ");
  Serial.println(stepIndex);
} // loop()




  /*
    potValue = analogRead(potPin);     // read the value of the potentiometer
    Serial.println(potValue);          // View full range from 0 - 1024 in Serial Monitor
    if (potValue < 535){               // if potentiometer reads 0 to 535 do this
      motorSpeed = (potValue/15 + 5);  //scale potValue to be useful for motor
      clockwise();                     //go to the ccw rotation function
    }
    else {                             //value of the potentiometer is 512 - 1024
      motorSpeed = ((1024-potValue)/15 + 5); //scale potValue for motor speed
      counterclockwise(); //go the the cw rotation function
    }
  */

