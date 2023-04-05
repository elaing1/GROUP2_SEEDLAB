#include "Encoder.h"
#include "DualMC33926MotorShield.h"

//PI controller values
// 6
#define kpR 6
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

int i = 0;

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


void serialEvent(){
  if (Serial.available() > 0) {
    data = Serial.readStringUntil('\n');
  }
  if(data[0] == 'a'){
    for (int i = 1; i < data.length(); i++) {
      strAngle += data.charAt(i);
    }
    Serial.print("angle");
    phiFromPi = strAngle.toDouble() * PI / 180;
  }
  if(data[0] == 'd'){
    for (int i = 1; i < data.length(); i++) {
      strDistance += data.charAt(i);
    }
    Serial.print("distance");
    distanceFromPi = strDistance.toDouble();
  }
  if(data[0] == '1'){
    Serial.print("true");
    arucoDetected = true;
  }
  else if(data[0] == '0'){
    Serial.print("false");
    arucoDetected = false;
  }
  DataRead = true;
  Serial.flush();
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
  Serial.begin(115200);

  //initialize motor
  md.init();
}
// finite state machine
int state = 3;
const int RECEIVE = 0;
const int ANGLE = 1;
const int DISTANCE = 2;
const int idle = 3;


// flags to communicate with the pi

bool sendData = false;
void loop() {
  // put your main code here, to run repeatedly:
  while (1) {


    
    // aruco detected == true
    switch (state) {
      case (RECEIVE):
      serialEvent();
        //communication with PI
       
        currentR = 0;
        if (sendData == false) {
          setPhi = currentPhi + phiFromPi;
          setDistance = distanceFromPi;
          if (setPhi != currentPhi) {
            state = ANGLE;
          } else if (setPhi == currentPhi) {
            state = DISTANCE;
          } else {
            analogWrite(9, 0);
            analogWrite(10, 0);
            state = idle;            
          }
        }
        break;

      case (ANGLE):
        Serial.println(setPhi);
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

        if ((currentPhi >= setPhi - 0.03) && (currentPhi <= setPhi + 0.03) && (setDistance != 0)) {
          state = DISTANCE;
        } else if ((currentPhi >= setPhi - 0.03) && (currentPhi <= setPhi + 0.03)) {
          sendData = true;
          //send the data to the pi
          Serial.println(sendData);
          sendData = false;
          analogWrite(9, 0);
          analogWrite(10, 0);
          state = idle;
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
          analogWrite(9, 0);
          analogWrite(10, 0);
          state = idle;
        } else {
          state = RECEIVE;
        }
        break;

      case (idle):

        //communication with PI
        serialEvent();
      
        // currentPhi = 0;
        currentR = 0;
        // delay(100);
        // setPhi = 0;
        // setDistance = 0;
        analogWrite(9, 0);
        analogWrite(10, 0);
        if (arucoDetected == true) {
          state = RECEIVE;
          i = 0;
        } else if (i > 31000) {
          setPhi += PI / 6;
          setDistance = 0;
          state = ANGLE;
          i = 0;
        }
        ++i;
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
