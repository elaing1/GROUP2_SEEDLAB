#include "Encoder.h"
#include "DualMC33926MotorShield.h"

//PI controller values
// 6
#define kpR 6
//20
#define kpPhi 20

#define maxVolts 8.0


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

int i = 0;

double IPhi = 0;
double ePhi = 0;
double IR = 0;
double eR = 0;


//Function to spin motors

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
  Serial.begin(9600);

  //initialize motor
  md.init();
}
// time
bool arucoDetected = false;
bool sendData = false;
void loop() {
  // put your main code here, to run repeatedly:
  while (1) {
    // aruco detected == true
    switch (state) {
      case (RECEIVE):
        currentPhi = 0;
        currentR = 0;
        delay(100);
        if (sendData) {
          setPhi = phiFromPi;
          setDistance = distanceFromPi;
          if (setPhi != 0) {
            state = ANGLE;
          } else if (setPhi == 0) {
            state = DISTANCE;
          } else {
            state = IDLE;
          }
        }
        break;
      case (ANGLE):
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

        if (PWML > 150) {
          PWML = 150;
        } else if (PWML < -150) {
          PWML = -150;
        }
        // 1.12 for both
        // 0.989
        if (PWMR > 150) {
          PWMR = 150 * 0.95;
        } else if (PWMR < -150) {
          PWMR = -150 * 0.95;
        }

        if ((currentPhi > setPhi) && (setDistance != 0)) {
          state = DISTANCE;
        } else if (currentPhi > setPhi) {
          sendData = true;
          //send the data to the pi
          Serial.println(sendData);
          sendData = false;
          state = RECEIVE;
        }
        break;
      case (DISTANCE):
        count1 = wheel1.read();
        count2 = -wheel2.read();
        currentR = -((double(count2) / 3210) + (double(count1) / 3210)) * PI * radius;
        Vrotational = ((setPhi - currentPhi) * kpPhi);
        Vdistance = ((setDistance - currentR) * kpR);
        Vright = (Vdistance - Vrotational);
        Vleft = (Vdistance + Vrotational);

        PWML = (Vleft / maxVolts) * 255;
        PWMR = (Vright / maxVolts) * 255;

        if (PWML > 150) {
          PWML = 150;
        } else if (PWML < -150) {
          PWML = -150;
        }
        // 1.12 for both
        // 0.989
        if (PWMR > 150) {
          PWMR = 150 * 0.95;
        } else if (PWMR < -150) {
          PWMR = -150 * 0.95;
        }

        delay(20);
        if (arucoDetected == false) {
          state = IDLE;
        } else {
          state = RECEIVE;
        }
        break;
      case (ILDE):
        currentPhi = 0;
        currentR = 0;
        delay(100);
        setPhi = 0;
        setDistance = 0;
        setDistance = 0;
        analogWrite(9, 0);
        analogWrite(10, 0);
        if (arucoDetected == true) {
          state = RECEIVE;
        }
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

    // Serial.print(currentR);
    // Serial.print("\t");
    // Serial.print(setDistance);
    // Serial.print("\t");
    // //
    // Serial.print(currentPhi);
    // Serial.print("\t");
    // //
    // Serial.print(setPhi);
    // Serial.print("\t");
    // Serial.print(PWML);
    // Serial.print("\t");
    // Serial.println(PWMR);
  }
}