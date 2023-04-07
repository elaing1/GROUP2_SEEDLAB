#include "Encoder.h"
#include "DualMC33926MotorShield.h"

//PI controller values
// 6
#define kpR 7
//20
#define kpPhi 25

#define maxVolts 7.9


DualMC33926MotorShield md;
//0.2395
const double radius = 0.25;  //FEET

double setDistance = 0;  // FEET
double setPhi = 0;       //RAD

double distanceFromPi = 0;
double phiFromPi = 0;

double currentR = 0;
double currentPhi = 0;

//int count = 0;

//Setting the encoder pins
int pin1A = 2;
int pin2A = 3;

int pin1B = 5;
int pin2B = 6;

//Setting the wheels as an encoder obeject
Encoder wheel1(pin1A, pin1B);
Encoder wheel2(pin2A, pin2B);



//Time
int startTime = 0;

//Encoder values and theta
int count1 = 0;
int count2 = 0;



double Vrotational = 0;
double Vdistance = 0;
double Vleft = 0;
double Vright = 0;
double PWML = 0;
double PWMR = 0;

double i = 0;

double IPhi = 0;
double ePhi = 0;
double IR = 0;
double eR = 0;




//COmm with PI

String data;
bool DataRead;
String strAngle;
bool arucoDetected = false;
String strDistance;
double angle;
double distance;

#include <Arduino.h>

float float1, float2;
String inputString = "";
bool stringComplete = false;

void serialEvent() {                    // This function is called when data is received over the serial port
  while (Serial.available()) {          // While there is data available
    char inChar = (char)Serial.read();  // Read the incoming character
    inputString += inChar;              // Add the character to the input string
    if (inChar == '>') {                // If the character is a closing angle bracket
      stringComplete = true;            // Set the string complete flag
    }
  }
}

void setup() {
  // put your setup code here, to run once:

  pinMode(pin1A, INPUT_PULLUP);
  pinMode(pin1B, INPUT_PULLUP);
  pinMode(pin2A, INPUT_PULLUP);
  pinMode(pin2B, INPUT_PULLUP);

  pinMode(4, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(12, INPUT);

  //Start serial monitor
  Serial.begin(115128);
  inputString.reserve(32);

  //initialize motor
  md.init();
}
// finite state machine
int state = 3;
const int RECEIVE = 0;
const int ANGLE = 1;
const int DISTANCE = 2;
const int idle = 3;
const int stop = 4;


// flags to communicate with the pi

void loop() {
  // put your main code here, to run repeatedly:
  while (1) {
    serialEvent();
    if (stringComplete) {                                              // If a complete string has been received
      if (inputString.startsWith("<") && inputString.endsWith(">")) {  // Check if the string is enclosed in angle brackets
        inputString.remove(0, 1);                                      // Remove the opening angle bracket
        inputString.remove(inputString.length() - 1);                  // Remove the closing angle bracket
        int commaIndex = inputString.indexOf(',');                     // Find the position of the comma separating the floats

        if (commaIndex > 0) {                                                // If a comma is found
          phiFromPi = inputString.substring(0, commaIndex).toFloat();        // Extract the first float
          distanceFromPi = inputString.substring(commaIndex + 1).toFloat();  // Extract the second float
          arucoDetected = true;
          if (phiFromPi == 100 && distanceFromPi == 100) {
            arucoDetected = false;

            Serial.println("aruco not detected");
          }
          // Use the received float values
          // Serial.print("Float 1: ");
          // Serial.println(phiFromPi, 2);
          // Serial.print("Float 2: ");
          // Serial.println(distanceFromPi, 2);
        }
      }
      inputString = "";        // Clear the input string
      stringComplete = false;  // Reset the string complete flag
    }
    // aruco detected == true


    switch (state) {
      case (RECEIVE):

        serialEvent();


        //communication with PI
        count1 = 0;
        count2 = 0;

        setPhi = phiFromPi * PI / 180 + currentPhi;
        setDistance = distanceFromPi + currentR;
        // if currentPhi != setPhi
        if (!((currentPhi >= setPhi - 0.03) && (currentPhi <= setPhi + 0.03))) {
          state = ANGLE;
          // if currentPhi == setPhi
        } else if ((currentPhi >= setPhi - 0.03) && (currentPhi <= setPhi + 0.03) && distanceFromPi > 0.5) {
          state = DISTANCE;
        } else {
          currentR = setDistance;
          analogWrite(9, 0);
          analogWrite(10, 0);
          state = stop;
        }

        break;

      case (ANGLE):

        setDistance = distanceFromPi + currentR;

        count1 = wheel1.read();
        count2 = -wheel2.read();
        currentR = -((double(count2) / 3210) + (double(count1) / 3210)) * PI * radius;
        currentPhi = radius * (-double(count2) / 3210 * 2 * PI + double(count1) / 3210 * 2 * PI) / ((11.625 / 12.0));
        Vrotational = ((setPhi - currentPhi) * kpPhi);
        Vdistance = ((setDistance - currentR) * kpR);


        Vright = (Vdistance - Vrotational);
        Vleft = (Vdistance + Vrotational);

        PWML = (Vleft / maxVolts) * 255;
        PWMR = (Vright / maxVolts) * 255;

        if (PWML > 128) {
          PWML = 128;
        } else if (PWML < -128) {
          PWML = -128;
        }
        // 1.12 for both
        // 0.989
        if (PWMR > 128) {
          PWMR = 128 * 0.95;
        } else if (PWMR < -128) {
          PWMR = -128 * 0.95;
        }

        if ((currentPhi >= setPhi - 0.03) && (currentPhi <= setPhi + 0.03) && (setDistance > 0.5)) {
          state = RECEIVE;
        } else if ((currentPhi >= setPhi - 0.03) && (currentPhi <= setPhi + 0.03)) {
          analogWrite(9, 0);
          analogWrite(10, 0);
          state = idle;
        }
        break;

      case (DISTANCE):
        serialEvent();


        if (distanceFromPi < 1) {
          //setPhi = phiFromPi * PI / 180 + currentPhi;
          setPhi = currentPhi;
          setDistance = distanceFromPi + currentR;
        } else {
          setPhi = phiFromPi * PI / 180 + currentPhi;
          setDistance = distanceFromPi + currentR;
        }

        count1 = wheel1.read();
        count2 = -wheel2.read();
        currentR = -((double(count2) / 3210) + (double(count1) / 3210)) * PI * radius;
        currentPhi = radius * (-double(count2) / 3210 * 2 * PI + double(count1) / 3210 * 2 * PI) / ((11.625 / 12.0));
        Vrotational = ((setPhi - currentPhi) * kpPhi);
        Vdistance = ((setDistance - currentR) * kpR);
        Vright = (Vdistance - Vrotational);
        Vleft = (Vdistance + Vrotational);

        PWML = (Vleft / maxVolts) * 255;
        PWMR = (Vright / maxVolts) * 255;

        if (PWML > 128) {
          PWML = 128;
        } else if (PWML < -128) {
          PWML = -128;
        }
        // 1.12 for both
        // 0.989
        if (PWMR > 128) {
          PWMR = 128 * 1.2;
        } else if (PWMR < -128) {
          PWMR = -128 * 1.2;
        }

        //delay(20);

        //Serial.println(currentR);
        // Serial.println(setDistance);


        if (arucoDetected == false) {
          Serial.println("made it");
          PWMR = 0;
          PWML = 0;
          analogWrite(9, 0);
          analogWrite(10, 0);
          currentR = setDistance;
          state = idle;
        } else if (currentR >= setDistance) {
          PWMR = 0;
          PWML = 0;
          analogWrite(9, 0);
          analogWrite(10, 0);
          setDistance = setDistance;
          state = stop;
        }
        // } else if (currentR >= setDistance - 1) {
        //   serialEvent();
        //   currentR = 0;
        //   setDistance = distanceFromPi;
        //   state = DISTANCE;
        // }
        break;

      case (idle):

        //communication with PI
        // currentPhi = 0;
        currentR = setDistance;
        // delay(100);
        // setPhi = 0;
        // setDistance = 0;
        analogWrite(9, 0);
        analogWrite(10, 0);
        //delay(20);
        if (arucoDetected == true) {
          analogWrite(9, 0);
          analogWrite(10, 0);
          setDistance = 0;
          state = RECEIVE;
          i = 0;
        } else if ((arucoDetected == false) && (i > 10000)) {

          setPhi += PI / 6;
          setDistance = 0;
          state = ANGLE;

          i = 0;
        }
        i += .5;
        break;

      case (stop):
        PWMR = 0;
        PWML = 0;
        analogWrite(9, 0);
        analogWrite(10, 0);
        state = stop;
        break;
    }


    if (PWMR >= 0) {

      analogWrite(9, PWMR);
      digitalWrite(7, 1);
    } else {

      analogWrite(9, -PWMR);
      digitalWrite(7, 0);
    }

    if (PWML >= 0) {
      analogWrite(10, PWML);
      digitalWrite(8, 0);
    } else {
      analogWrite(10, -PWML);
      digitalWrite(8, 1);
    }

    //    Serial.print(currentR);
    //    Serial.print("\t");
    //    Serial.print(setDistance);
    //    Serial.print("\t");
    //    //
    //    Serial.print(currentPhi);
    //    Serial.print("\t");
    //    //
    //    Serial.print(setPhi);
    //    Serial.print("\t");
    //    Serial.print(PWML);
    //    Serial.print("\t");
    //    Serial.println(PWMR);
  }
}
