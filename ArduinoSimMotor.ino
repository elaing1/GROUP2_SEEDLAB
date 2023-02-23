//hardware:
// pin 2      , CLK: data A
// pin 3      , DT: data B
// pin 4      , SW: button when pressed
// 5V         , +
// GND        , GND:


// ki kp 
// put put block, pwm, motor
#include "Arduino.h"

#define pinA 2
#define pinB 3
#define eButton 4
//#define pin 10
#define CCW 1
#define CW 0
// sampling rate 5 ms
#define samplingRate 5
#define fixedCycleTime 2350
#define CPR 1600.0

#define maxVolts 7.7
//motor pins
#define PWM 10
#define DIR 8


double angularVelocity = 0.00;
double position = 0.00;
double newPosition = 0.00;
uint32_t start_time, stop_time;

// variables for serial communication
bool docounting = false;
String InputString = "";  // a string to hold incoming data
bool StringComplete = false;

void setup() {

  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  pinMode(eButton, INPUT_PULLUP);
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);
  attachInterrupt(0, encoderI, CHANGE);
  writeMotor(0, CW);

  Serial.begin(115200);
  InputString.reserve(200);
  Serial.println("Ready!");
  start_time = millis();
}
// PWM pins 3,5,6,9,10,11 (5 and 6 980 Hz, everything else is 490 Hz)
//analogWrite(pin, value) writes an analog value to a pin
volatile int64_t turnValue = 0;
int64_t pastTurnValue = 0;
/*
int64_t readEncoder() {
  int64_t temp;
  cli();
  temp = turnValue;
  sei();
  return temp;
}
*/
double motorVoltage = 0.0;
void writeMotor(int speed, bool direction) {
  analogWrite(PWM, speed);
  digitalWrite(DIR, direction);
  if (direction) {
    motorVoltage = -maxVolts * speed / 255.0;
  } else {
    motorVoltage = maxVolts * speed / 255.0;
  }
}

void loop() {

  // Change behavior based on serial input
  if (StringComplete) {
    switch (InputString.charAt(0)) {
      case 'S':
        docounting = true;
        break;
      case 'E':
        docounting = false;
        break;
    }
    StringComplete = false;
  }

  if (docounting) {
    stop_time = millis();

    if ((stop_time - start_time) >= samplingRate) {  // Loop each 5 ms

      // radius of the wheel 71.91mm
      double position = turnValue / CPR * 2 * 3.141592;

      // if you want to reset position @ 2 pi to 0
      /*
      if (abs(position) >= 2 * 3.141592) {
        newPosition = abs(position) - abs(int(position));
        if (position > 0) {
          position = newPosition;
        }
        else {
          position = -newPosition;
        }
      }
     */

      // Calculate velocity [radian / sec]
      double velocity = ((turnValue - pastTurnValue) / 10e-3) / CPR * 2 * 3.141592;

      // Update past value
      pastTurnValue = turnValue;

      // Reset start_time
      start_time = stop_time;

      // print
      Serial.print(position);
      Serial.print("\t");
      Serial.print(velocity);
      Serial.print("\t");
      Serial.print(motorVoltage);
      Serial.print("\t");
      Serial.println(millis());
      motorVoltage = 0;
    }

    if (millis() >= fixedCycleTime) {
      Serial.println("Finished");
      docounting = false;
      writeMotor(0, CCW);
      delay(100);
      // stops program from printing anymore
      while (true) {
        continue;
      }
    } 
  }
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    InputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      StringComplete = true;
    }
  }
}

// encoder.h library
void encoderI() {
  if (digitalRead(pinA) == CCW) {
    if (digitalRead(pinB) == CCW) {
      turnValue--;
    } else {
      turnValue++;
    }
  } else {
    if (digitalRead(pinB) == CCW) {
      turnValue++;
    } else {
      turnValue--;
    }
  }
}
