
// Two LIDARs on one side

#include <Wire.h>
#include <math.h>
#include <PID_v1.h>
#include <Servo.h>

#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

//set up servos
Servo steeringServo;
Servo motorServo;
const int steeringPin = 9;
const int motorPin = 8;

//define PID variables
double Setpoint_theta, Input_theta, Output_theta, Setpoint_dist, Input_dist, Output_dist;
//specify initial PID links and tuning
int KpT = 2;
int KiT = 0;
int KdT = 1;
int KpD = 2;
int KiD = 0;
int KdD = 0;
PID PID_theta(&Input_theta, &Output_theta, &Setpoint_theta, KpT, KiT, KdT, DIRECT);
PID PID_dist(&Input_dist, &Output_dist, &Setpoint_dist, KpD, KiD, KdD, DIRECT);

void getDist(double (&distArray)[4]);
// Variables to store last state
double lastDist0;
double lastDist1;
// Variables to store current distances
double dist0;
double dist1;

//Spacing between LIDARs on vehicle
double LIDARspacing = 20;

// Power enable switches
const int switch0 = 12;
const int switch1 = 13;

// # of LIDAR samples
int samples = 5;

// Set pi
const float pi = 3.142;

//---------------------------------------------------------//
void setup() {
  // Initialize linked variables
  Input_theta = 0;
  Input_dist = 100;
  Setpoint_theta = 0;
  Setpoint_dist = 90;

  PID_theta.SetMode(AUTOMATIC);
  PID_dist.SetMode(AUTOMATIC);

// Servo range limits
  PID_theta.SetOutputLimits(-45, 45);
  PID_dist.SetOutputLimits(-45, 45);

  steeringServo.attach(steeringPin);
  motorServo.attach(motorPin);

  // Initialize power enable pins
  pinMode(switch0, OUTPUT);
  pinMode(switch1, OUTPUT);
  // Initialize LIDAR0 on and LIDAR1 off
  digitalWrite(switch0, HIGH);
  digitalWrite(switch1, LOW);

  // Begin serial
  Serial.begin(9600);
  // Open and join irc bus as master
  Wire.begin();
  delay(100);

}

void loop() {
  double distArr[4];
  getDist(distArr);
  for (int k = 0; k < 4 ; k++) {
    Serial.println(distArr[k]);
    delay(1);
  }
  Serial.println("end");
  
  // Run the PID loops
  Input_theta = distArr[2];
  Input_dist = distArr[3];
  PID_theta.Compute();
  PID_dist.Compute();
  
  motorServo.write(90  );
  steeringServo.write(90 - Output_theta); // - Output_theta - Output_dist
  delay(15);
}


//---------------------------------------------------------//
void getDist(double (&distArray)[4]) {

  for (int i = 0; i < 2; i++) {
    switch (i) {
      // case 0 is LIDAR0, case 1 is LIDAR1
      case 0:
        digitalWrite(switch0, LOW);
        digitalWrite(switch1, LOW);
        delay(5);
        digitalWrite(switch0, HIGH);
        delay(5);
        break;
      case 1:
        digitalWrite(switch0, LOW);
        digitalWrite(switch1, LOW);
        delay(5);
        digitalWrite(switch1, HIGH);
        delay(5);
        break;
    }
    //Error checking
    //    Serial.print("case: ");
    //    Serial.println(i);
    //--

    // sum distances so that we can average multiple samples
    int distSum = 0;
    int distance;

    for (int j = 0; j < samples; j++) {
      distance = 0;
      // This section from LIDAR Lite Wire example
      //-----------------//
      Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
      Wire.write((int)RegisterMeasure); // sets register pointer to  (0x00)
      Wire.write((int)MeasureValue); // sets register pointer to  (0x00)
      Wire.endTransmission(); // stop transmitting

      delay(20); // Wait 20ms for transmit

      Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
      Wire.write((int)RegisterHighLowB); // sets register pointer to (0x8f)
      Wire.endTransmission(); // stop transmitting

      delay(20); // Wait 20ms for transmit

      Wire.requestFrom((int)LIDARLite_ADDRESS, 2); // request 2 bytes from LIDAR-Lite

      if (2 <= Wire.available()) // if two bytes were received
      {
        distance = Wire.read(); // receive high byte (overwrites previous reading)
        distance = distance << 8; // shift high byte to be high 8 bits
        distance |= Wire.read(); // receive low byte as lower 8 bits
      }
      //-----------------//
      //low pass filter
      switch (i) {
        case 0:
          if (distance < 0 || distance > 4000) {
            distance = lastDist0;
          }
          distance = 0.7 * distance + 0.3 * lastDist0;
          lastDist0 = distance;
          break;
        case 1:
          if (distance < 0 || distance > 4000) {
            distance = lastDist1;
          }
          distance = 0.7 * distance + 0.3 * lastDist1;
          lastDist1 = distance;
          break;
      }
      distSum = distSum + distance;
      delay(10);
    }
    if (i == 0) {
      dist0 = distSum / samples;
      //Error checking
      Serial.print("dist0: ");
      Serial.println(dist0);
      //--
      //      distArray[0] = dist0 * 1.0;
    } else if (i == 1) {
      dist1 = distSum / samples;
      // Distance calibration for sensor 2
      //      dist1 = 46 + ((88 - 46) / (82 - 39)) * (dist1 - 39);
      dist1 = 6.5 + dist1 * (0.98);
      //Error checking
      Serial.print("dist1: ");
      Serial.println(dist1);
      //--
      //      distArray[1] = dist1 * 1.0;
    }
    double angle = atan((dist0 - dist1) / LIDARspacing);
    //Error checking
    //    Serial.print("angle: ");
    //    Serial.println(angle);
    //--
    distArray[2] = angle;
    double perpDist0 = dist0 * cos(angle);
    distArray[0] = perpDist0;
    double perpDist1 = dist1 * cos(angle);
    distArray[1] = perpDist1;
    double centerDist = (perpDist0 + perpDist1) / 2;
    distArray[2] = angle * 180 / 3.142;
    //Error checking
    //    Serial.print("centerDist: ");
    //    Serial.println(centerDist);
    //--
    distArray[3] = centerDist;
    delay(10);
  }
}

void calcDistAngle(double dist0, double dist1) {
  double angle = atan((dist0 - dist1) / LIDARspacing);
  double centerDist = ((dist0 + dist1) * cos(angle)) / 2;
  double distAngle [2];
  distAngle[0] = angle;
  distAngle[1] = centerDist;
  return distAngle;
}

