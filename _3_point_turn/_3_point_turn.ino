#include <Servo.h>

Servo wheels; // servo for turning the wheels
Servo esc; // servo for motors

double IROffset = 20;

float IRFront, IRBack, cmFront, cmBack;

void setup() {
  Serial.begin(9600);
  wheels.attach(7); // initialize wheel servo to Digital IO Pin #7
  esc.attach(8); // initialize ESC to Digital IO Pin #8
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

}

void loop() {
    IRFront = analogRead(A1);
    cmFront = 10650.08 * pow(IRFront, -0.935) - 10;
    IRBack  = analogRead(A2);
    cmBack = 10650.08 * pow(IRBack, -0.935) - 10;
    
    if(cmFront > 150){
      Serial.println("Out of range.");
      esc.write(0);
      wheels.write(90);
    }
    else {
      wheels.write(0);
      esc.write(80);
      delay(100);
    
    while(cmFront > IROffset){// and two lidar value not equals
      IRFront = analogRead(A1);
      cmFront = 10650.08 * pow(IRFront, -0.935) - 10;
      wheels.write(0); //turn left
      esc.write(0); 
    }
    while(cmBack > IROffset){// and two lidar value not equals 
      IRBack  = analogRead(A2);
      cmBack = 10650.08 * pow(IRBack, -0.935) - 10;
      wheels.write(180);//turn right
      esc.write(180);  
    }
    while(true){// lidar find the car is parallel to the wall
      IRFront = analogRead(A1);
      cmFront = 10650.08 * pow(IRFront, -0.935) - 10;
      wheels.write(0);
      esc.write(0);
    }
  }

}

