#include <Servo.h>

Servo wheels; // servo for turning the wheels
Servo esc; // not actually a servo, but controlled like one!
double IROffset = 20;

float IRFront, IRBack, cmFront, cmBack;

// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  Serial.begin(9600);
  wheels.attach(7); // initialize wheel servo to Digital IO Pin #7
  esc.attach(8); // initialize ESC to Digital IO Pin #8
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

  myservo.write(0); //initialization
}

void loop() {
    IRFront = analogRead(A1);
    IRBack  = analogRead(A2);
    
    cmFront = 10650.08 * pow(IRFront, -0.935) - 10;
    cmBack = 10650.08 * pow(IRBack, -0.935) - 10;
    
    if(cmFront > 150)
      Serial.println("Out of range.");
    else {
      threePointTurn(cmFront,cmBack);    
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    esc.write(0);
    wheels.write(90);
    delay(1);
}


// calculate the x, y position of the object
void threePointTurn(float f, float b) {
    //make some difference between 2 lidar
    wheels.write(0);
    esc.write(80);
    delay(100);
    
    while(f > IROffset){// and two lidar value not equals
      wheels.write(0); //turn left
      esc.write(80); //slow forward
    }
    while(b > IROffset){// and two lidar value not equals 
      wheels.write(180);//turn right
      esc.write(100);//slow backward  
    }
    while(true){// lidar find the car is parallel to the wall
      wheels.write(0);
      esc.write(80);
    }
  }

