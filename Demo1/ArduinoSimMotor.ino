//hardware:
// pin 2      , CLK: data A
// pin 3      , DT: data B
// pin 4      , SW: button when pressed
// 5V         , +
// GND        , GND:


// ki kp
// put put block, pwm, motor
#include "Arduino.h"
#include "DualMC33926MotorShield.h"
#include "Encoder.h"

#define pinA 2
#define pinB 5
#define pinA2 3
#define pinB2 6
#define eButton 4
//#define pin 10
#define CCW 0
#define CW 1
// sampling rate 5 ms
#define samplingRate 5
#define fixedCycleTime 2350
// motor 1 1600.0
// motor 2 1660.0
#define CPR 1600.0
#define CPR2 1600.0

#define maxVolts 7.7
//motor pins 10 8
#define PWM 10
#define DIR 8
#define PWM2 9
// original 12
#define DIR2 7

DualMC33926MotorShield md;
Encoder motor1(pinA, pinB);
Encoder motor2(pinA2, pinB2);

double position = 0.00;
double position2 = 0.00;

double velocity = 0.00;
double velocity2 = 0.00;

double forwardV = 0.00;
double rotationalV = 0.00;

uint32_t start_time, stop_time;

// variables for serial communication
bool docounting = false;
String InputString = "";  // a string to hold incoming data
bool StringComplete = false;

double motorVoltage = 0.0;
double motorVoltage2 = 0.0;
void writeMotor(int motor, int speed, bool direction) {

  if (motor == 1) {
    analogWrite(PWM, speed);
    digitalWrite(DIR, direction);
    if (!(direction)) {
      motorVoltage = -maxVolts * speed / 255.0;
    } else {
      motorVoltage = maxVolts * speed / 255.0;
    }
  } else {
    analogWrite(PWM2, speed);
    digitalWrite(DIR2, direction);
    if (!(direction)) {
      motorVoltage2 = maxVolts * speed / 255.0;
    } else {
      motorVoltage2 = -maxVolts * speed / 255.0;
    }
  }
}

void setup() {

  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  pinMode(eButton, INPUT_PULLUP);
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);
  writeMotor(1, 0, CW);
  writeMotor(2, 0, CW);

  //initialize motor
  md.init();

  Serial.begin(115200);
  InputString.reserve(200);
  Serial.println("Ready!");
  start_time = millis();
}
// PWM pins 3,5,6,9,10,11 (5 and 6 980 Hz, everything else is 490 Hz)
//analogWrite(pin, value) writes an analog value to a pin
volatile int64_t turnValue = 0;
int64_t pastTurnValue = 0;
volatile int64_t turnValue2 = 0;
int64_t pastTurnValue2 = 0;



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

      turnValue = -motor1.read();
      turnValue2 = -motor2.read();

      double position = double(turnValue) / CPR * 2 * 3.141592;
      double position2 = -double(turnValue2) / CPR2 * 2 * 3.141592;

      // Calculate velocity [radian / sec]
      double velocity = ((turnValue - pastTurnValue) / 10e-3) / CPR * 2 * 3.141592;
      double velocity2 = -((turnValue2 - pastTurnValue2) / 10e-3) / CPR2 * 2 * 3.141592;

      // vector V
      double sumV = velocity - velocity2;
      // delta V
      double differenceV = velocity - velocity2;
      // radius of the wheel 71.91mm = 0.2359252 ft
      // calculate the forward velocity for the robot
      forwardV = 0.235952 * (velocity + velocity2) / 2;

      // 10 11/16ths distance between the wheels
      // 0.890625 ft
      rotationalV = 0.235952 * (velocity - velocity2) / 0.890625;

      // Update past value
      pastTurnValue = turnValue;
      pastTurnValue2 = turnValue2;

      // Reset start_time
      start_time = stop_time;

      // print motor 1 info
      Serial.print(position);
      Serial.print("\t");
      Serial.print(velocity);
      Serial.print("\t");
      Serial.print(motorVoltage);
      Serial.print("\t");
      // print motor 2 info
      Serial.print(position2);
      Serial.print("\t");
      Serial.print(velocity2);
      Serial.print("\t");
      Serial.print(motorVoltage2);
      Serial.print("\t");
      Serial.print(forwardV);
      Serial.print("\t");
      Serial.print(rotationalV);
      Serial.print("\t");
      Serial.print(sumV);
      Serial.print("\t");
      Serial.print(differenceV);
      Serial.print("\t");
      Serial.println(millis());
      motorVoltage = 0;
    }

    if (millis() >= fixedCycleTime) {
      Serial.println("Finished");
      docounting = false;
      writeMotor(1, 0, CCW);
      writeMotor(2, 0, CCW);
      delay(100);
      // stops program from printing anymore
      while (true) {
        continue;
      }
    } else {
      writeMotor(1, 127, CW);
      writeMotor(2, 127, !(CW));
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