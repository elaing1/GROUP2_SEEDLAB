#include "Encoder.h"
#include "DualMC33926MotorShield.h"

DualMC33926MotorShield md;

const double radius = .2395;  //FEET

double setDistance = 0;  // FEET
double setPhi = 0;       //RAD

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
Encoder wheel2(pin2A,pin2B);

//PI controller values


//Time
int startTime = 0;

//Encoder values and theta
int count1 = 0;
int count2 = 0;
#define maxVolts 8.0
//Feedback loop values
#define kpR 20
// 50
#define kpPhi 28

double Vrotational = 0;
double Vdistance = 0;
double Vleft = 0;
double Vright = 0;
double PWML = 0;
double PWMR = 0;
int i = 0;
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

void loop() {
  // put your main code here, to run repeatedly:

  while (1) {
    if (i > 250) {
      // fudge factor + 0.25
      setPhi = 0;
    }
    if (i > 500) {
      setDistance = 7;
    }

    count1 = wheel1.read();
    count2 = -wheel2.read();
    currentR = ((double(count2) / 3210) + -(double(count1) / 3210)) * PI * radius;
    currentPhi = radius * (double(count2) / 3210 * 2 * PI + double(count1) / 3210 * 2 * PI) / ((10.5 / 12));

    Vrotational = (setPhi - currentPhi) * kpPhi;
    Vdistance = (setDistance - currentR) * kpR;

    Vleft = Vdistance - Vrotational;
    Vright = Vdistance + Vrotational;

    PWML = (Vleft / maxVolts) * 255;
    PWMR = (Vright / maxVolts) * 255;

    if (PWML > 128) {
      PWML = 128;
    } else if (PWML < -128) {
      PWML = -128;
    }
    PWMR = PWMR * 1.08;
    if (PWMR > 128) {
      PWMR = 128;
    } else if (PWMR < -128) {
      PWMR = -128;
    }

    if (PWMR >= 0) {

      analogWrite(9, PWMR);
      digitalWrite(8, 0);
    } else {

      analogWrite(9, -PWMR);
      digitalWrite(8, 1);
    }

    if (PWML >= 0) {
      analogWrite(10, PWML);
      digitalWrite(7, 1);
    } else {
      analogWrite(10, -PWML);
      digitalWrite(7, 0);
    }



    delay(20);
    Serial.print(currentR);
    Serial.print("\t");
    Serial.print(setDistance);
    Serial.print("\t");
    Serial.print(currentPhi);
    Serial.print("\t");
    Serial.println(setPhi);

    if (currentR == setDistance) {
      analogWrite(9, 0);
      analogWrite(10, 0);
      break;
    }
    i++;
  }
}