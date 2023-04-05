#include <Arduino.h>

float float1, float2;
String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);
  inputString.reserve(32);
}

void loop() {
  if (stringComplete) {  // If a complete string has been received
    if (inputString.startsWith("<") && inputString.endsWith(">")) {  // Check if the string is enclosed in angle brackets
      inputString.remove(0, 1);  // Remove the opening angle bracket
      inputString.remove(inputString.length() - 1);  // Remove the closing angle bracket
      int commaIndex = inputString.indexOf(',');  // Find the position of the comma separating the floats
      
      if (commaIndex > 0) {  // If a comma is found
        float1 = inputString.substring(0, commaIndex).toFloat();  // Extract the first float
        float2 = inputString.substring(commaIndex + 1).toFloat();  // Extract the second float
        if (float1 == 100 && float2 == 100){
          Serial.println("aruco not detected");
        }
        // Use the received float values
        Serial.print("Float 1: ");
        Serial.println(float1, 2);
        Serial.print("Float 2: ");
        Serial.println(float2, 2);
      }
    }
    inputString = "";  // Clear the input string
    stringComplete = false;  // Reset the string complete flag
  }
}

void serialEvent() {  // This function is called when data is received over the serial port
  while (Serial.available()) {  // While there is data available
    char inChar = (char)Serial.read();  // Read the incoming character
    inputString += inChar;  // Add the character to the input string
    if (inChar == '>') {  // If the character is a closing angle bracket
      stringComplete = true;  // Set the string complete flag
    }
  }
}
