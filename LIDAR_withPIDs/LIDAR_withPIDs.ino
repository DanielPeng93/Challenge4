
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
  
  motorServo.write(90  -20 );
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


// //#include <Servo.h>
// #include <Wire.h>

// #define LIDARLITE_ADDR 0x62
// #define LIDARLITE_CMD_CTRL_ADDR 0x00
// #define LIDARLITE_TRIG_VAL 0x04
// #define LIDARLITE_RANGE_ADDR 0x8f

// //Servo wheel_speed_servo, wheel_direction_servo;
// const int lidar_pwr_en[] = {
//   12, 13
// };    // PWR_EN pins to put lidars to sleep
// int enabled_lidar = 0;
// int lidar_state = 0;    // {0:Disabled, 1:Enabled, 2:Triggered, 3:Requested data}
// int state_cycle_count = 0;
// double dist[2];

// void read_lidars() {
//   switch (lidar_state) {
//     case 0:
//       for (int i = 0; i < 2; i++) digitalWrite(lidar_pwr_en[i], LOW);
//       digitalWrite(lidar_pwr_en[enabled_lidar], HIGH);
//       lidar_state = 1;
//       state_cycle_count = 0;
//       break;
//     case 1:
//       Wire.beginTransmission(LIDARLITE_ADDR);
//       Wire.write(LIDARLITE_CMD_CTRL_ADDR);
//       Wire.write(LIDARLITE_TRIG_VAL);
//       if (Wire.endTransmission() == 0) lidar_state = 2;
//       else state_cycle_count++;
//       break;
//     case 2:
//       Wire.beginTransmission(LIDARLITE_ADDR);
//       Wire.write(LIDARLITE_RANGE_ADDR);
//       if (Wire.endTransmission() == 0) lidar_state = 3;
//       else state_cycle_count++;
//       break;
//     case 3:
//       if (Wire.requestFrom(LIDARLITE_ADDR, 2) >= 2) {
//         uint16_t val = Wire.read() << 8 | Wire.read();
//         if (val < 500 && val > 10) dist[enabled_lidar] = val;
//         enabled_lidar = 1 - enabled_lidar;
//         lidar_state = 0;
//       } else state_cycle_count++;
//       break;
//   }
//   if (state_cycle_count > 30) {
//     lidar_state = 0;
//     enabled_lidar = 1 - enabled_lidar;
//     state_cycle_count = 0;
//   }
// }

// unsigned long serial_timestamp, now;


// void setup() {
//   Wire.begin();
//   Serial.begin(57600);
//   Serial.println("Started");
//   for (int i = 0; i < 2; i++)
//     pinMode(lidar_pwr_en[i], OUTPUT);

//   for (int i = 0; i < 2; i++) {
//     for (int j = 0; j < 2; j++)  digitalWrite(lidar_pwr_en[j], LOW);
//     digitalWrite(lidar_pwr_en[i], HIGH);
//     while (1) {
//       Wire.beginTransmission(LIDARLITE_ADDR);
//       Wire.write(0x01);
//       Wire.write(0xA0);
//       if (Wire.endTransmission() == 0) break;
//     }
//   }
//   //  wheel_speed_servo.attach(8);
//   //  wheel_direction_servo.attach(9);
//   //  calibrate_servo(wheel_direction_servo, 0, 180);
// }

// void loop() {
//   read_lidars();
//   now = millis();
//   if(now - serial_timestamp > 20){
//     serial_timestamp = now;
//     Serial.print(dist[0]);
//     Serial.print('\t');
//     Serial.println(dist[1]);
//   }
//   delay(1);
// }
// //void calibrate_servo(Servo & servo, int minVal, int maxVal) {
// //  const int calibration_delay = 1000;
// //  const int calibration_angles[] = {
// //    maxVal, minVal, (minVal + maxVal) / 2, (minVal + maxVal) / 2
// //  };
// //  for (int i = 0; i < sizeof(calibration_angles) / sizeof(*calibration_angles); i++) {
// //    servo.write(calibration_angles[i]);
// //    delay(calibration_delay);
// //  }
// //}



