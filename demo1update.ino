#include "Encoder.h"
#include "DualMC33926MotorShield.h"

DualMC33926MotorShield md;

const double radius = .2395;  //FEET

const double DISTANCEHELPER = (10.5 / 12) / 2 * PI;
const double PHIERROR = 1 / 8;
double setDistance = 0;  // FEET
double setPhi = 0;       //RAD

double currentD = 0;
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

//PI controller values


//Time
int currentTime = 0;

//Encoder values and theta
int count1 = 0;
int count2 = 0;
double theta = 0;

//Feedback loop values
double kpP = 17.15668;
double kdP = 0.673531006182646;
double DP = 0;
double e_pastP = 0;
double Ts = 0;
double Tc = currentTime;
double eP = 0.0;
double uP = 0;

double kpPhi = 6.50202867560687;
double kdPhi = 1.1968030832634;
double DPhi = 0;
double e_pastPhi = 0;
double ePhi = 0;
double uPhi = 0;
bool angle = true;

//Function to spin motors
void spinRight(int PWM) {

  if (PWM >= 0) {
    analogWrite(9, PWM);
    digitalWrite(8, 0);
  }
  if (PWM < 0) {
    analogWrite(9, -PWM);
    digitalWrite(8, 1);
  }
}

void spinLeft(int PWM) {

  if (PWM >= 0) {
    analogWrite(10, PWM);
    digitalWrite(7, 1);
  }
  if (PWM < 0) {
    analogWrite(10, -PWM);
    digitalWrite(7, 0);
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
  Serial.begin(9600);

  //initialize motor
  md.init();
}

void loop() {
  // put your main code here, to run repeatedly:

  currentTime = millis();


// *1.3333333
  setDistance = 0 *1.3333333;           // FEET
  //1.0625 *
  setPhi = 1.0625 *2*PI;  //RAD

  while (1) {

    currentTime = millis();

    count1 = -wheel2.read();
    count2 = wheel1.read();
    currentD = (double(count2) / 3210 + double(count1) / 3210) * PI * radius;
    currentPhi = radius * (-double(count2) / 3210 * 2 * PI + double(count1) / 3210 * 2 * PI) / ((10.5 / 12));

    eP = setDistance - currentD;

    ePhi = setPhi - currentPhi;

    if (Ts > 0) {
      DPhi = (ePhi - e_pastPhi) / Ts;
      e_pastPhi = ePhi;

      DP = (eP - e_pastP) / Ts;
      e_pastP = eP;
    } else {
      DPhi = 0;
      DP = 0;
    }

    uPhi = 255 * (kpPhi * ePhi + kdPhi * DPhi);
    uP = 255 * (kpP * eP + kdPhi * DP);

    if (uP > 255) {
      uP = 255;
    }
    if (uPhi > 255) {
      uPhi = 255;
    }
    if (uP < -255) {
      uP = -255;
    }
    if (uPhi < -255) {
      uPhi = -255;
    }

  
    if (setDistance != 0 && setPhi != 0 && abs(ePhi) > 0.05) {
      uP = 0;
      spinRight(-1.2 * (uP - uPhi) / 2);
      spinLeft(-1.6 * (uP + uPhi) / 2);
      

    } else if (abs(ePhi) > 0.01 && abs(eP) < 0.1) {
      spinRight(-1.2 * (uP - uPhi) / 2);
      spinLeft(-1.6 * (uP + uPhi) / 2);
      DP = 0;

    } else if (abs(ePhi) < 0.2 && abs(eP) > 0.1) {
      spinRight(0.84 * (uP - uPhi) / 2);
      spinLeft(0.84 * (uP + uPhi) / 2);

    } else {
      spinRight(0);
      spinLeft(0);
      eP = 0;
      ePhi = 0;
      currentD = setDistance;
      currentPhi = setPhi;
      DP = 0;
      DPhi = 0;
    }


    Serial.print(setPhi);
    Serial.print("\t");
    Serial.print(currentPhi);
    Serial.print("\t");
    Serial.print(ePhi);
    Serial.print("\t");
    Serial.print(uPhi);
    Serial.print("\t");
    Serial.print("\t");

    Serial.print(setDistance);
    Serial.print("\t");
    Serial.print(currentD);
    Serial.print("\t");
    Serial.print(eP);
    Serial.print("\t");
    Serial.println(uP);


    Ts = currentTime - Tc;
    Tc = currentTime;
  }
}
