#include <Servo.h>

// records of the distance between sensor and the object
float sensorValue, cm, x, y;

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  Serial.begin(9600);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  pinMode(A1, INPUT);

  myservo.write(0);
}

void loop() {
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    
    sensorValue = analogRead(A1);
    cm = 10650.08 * pow(sensorValue, -0.935) - 10;
    if(cm > 200)
      Serial.println("Out of range.");
    else {
      calculatePosition(cm);    
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'

    sensorValue = analogRead(A1);
    cm = 10650.08 * pow(sensorValue, -0.935) - 10;
    if(cm > 200)
      Serial.println("Out of range.");
    else {
      calculatePosition(cm);
      delay(15);                       // waits 15ms for the servo to reach the position
    }    
  }
}

// calculate the x, y position of the object
void calculatePosition(float f) {
    y = sin(cm) * cm;
    x = cos(cm) * cm;

    if(y < 20)
      Serial.println("Danger!");
    else {
      Serial.print("Position: cm: ");
      Serial.print(cm);
      Serial.print("; x: ");
      Serial.print(x);
      Serial.print("; y: ");
      Serial.println(y);
    }    
  }
