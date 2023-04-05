String data;
bool DataRead;
String strAngle;
bool arucoDetected = false;
String strDistance;
double angle;
double distance;
void setup() {
  Serial.begin(115200);
}
void loop() {
  if (DataRead) {
    Serial.println(angle, 6);
     DataRead = false;
  }
}
void serialEvent(){
  if (Serial.available() > 0) {
    data = Serial.readStringUntil('\n');
  }
  if(data[0] == 'a'){
    for (int i = 1; i < data.length(); i++) {
      strAngle += data.charAt(i);
    }
    angle = strAngle.toDouble();
  }
  else if(data[0] == 'd'){
    for (int i = 1; i < data.length(); i++) {
      strDistance += data.charAt(i);
    }
    distance = strDistance.toDouble();
  }
  else if(data[0] == '1'){
    arucoDetected = true;
  }
  else if(data[0] == '0'){
    arucoDetected = false;
  }
  DataRead = true;
  Serial.flush();
}
