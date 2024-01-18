#include <Servo.h>

Servo myservo;  // create servo object to control a servo

int pos = 0;    // variable to store the servo position
int servoPin = 9; // Pin where the servo control wire is connected. Change this if needed.

void setup() {
  myservo.attach(servoPin);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600);       // initialize serial communication
  Serial.println("Enter angle between 0 and 1800 (for 5 turns):"); // prompt the user
}

void loop() {
  if (Serial.available()) { // check if data has been sent from the computer
    pos = Serial.parseInt(); // read it and store in 'pos'

    if (pos >= 0 && pos <= 1800) { // 5 turns * 360 degrees = 1800 degrees
      myservo.write(map(pos, 0, 1800, 0, 180)); // map 0-1800 to 0-180 and set the servo position
      Serial.print("Moved to: ");
      Serial.print(pos);
      Serial.println(" degrees");
    } else {
      Serial.println("Invalid angle. Enter a value between 0 and 1800.");
    }
  }
}
