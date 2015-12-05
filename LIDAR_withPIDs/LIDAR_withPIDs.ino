// Two LIDARs on one side

#include <Wire.h>
#include <math.h>
#include <PID_v1.h>
#include <Servo.h>
#include <XBee.h>
#include <QueueArray.h>


#define LIDARLITE_ADDR 0x62
#define LIDARLITE_CMD_CTRL_ADDR 0x00
#define LIDARLITE_TRIG_VAL 0x04
#define LIDARLITE_RANGE_ADDR 0x8f

// timestamps
unsigned long now, timestamp;

const uint8_t MSG_TRIP1 = 0xB1,
              MSG_TRIP2 = 0xB2,
              MSG_TRIP3 = 0xB3,
              MSG_TRIP4 = 0xB4;

const int lidar_pwr_en[] = {
  12, 13
};    // PWR_EN pins to put lidars to sleep
int enabled_lidar = 0;
int lidar_state = 0;    // {0:Disabled, 1:Enabled, 2:Triggered, 3:Requested data}
int state_cycle_count = 0;
double dist[2];

int pos = 90;
int speed = 90;
bool trigger[] = {true, false, false, false, false}; // first true is to allow first beacon to trip
bool complete = false;
int forwardTime = 2000;
int pauseTime = 500;
int servoMoveTime = 600;
unsigned long timeStamp = millis();
int count = 1;

//set up servos
Servo steeringServo, motorServo;
const int steeringPin = 9, motorPin = 8;

//define PID variables
double Setpoint_theta = 0, Input_theta, Output_theta, Setpoint_dist = 90, Input_dist, Output_dist, Setpoint_speed = 60, Input_speed, Output_speed;
//specify initial PID links and tuning
int KpT = 1.7, KiT = 0, KdT = 0.01;  // {1.7, 0, 0.01}
int KpD = 1.1, KiD = 0.075, KdD = 0.97;   // {1, 0, 0}
//int KpS = 0.5, KiS = 0, KdS = 0;
PID PID_theta(&Input_theta, &Output_theta, &Setpoint_theta, KpT, KiT, KdT, DIRECT);
PID PID_dist(&Input_dist, &Output_dist, &Setpoint_dist, KpD, KiD, KdD, DIRECT);
//PID PID_speed(&Input_speed, &Output_speed, &Setpoint_speed, KpS, KiS, KdS, DIRECT);

//Spacing between LIDARs on vehicle
double LIDARspacing = 22.5;

// Set pi
const float pi = 3.142;

// Wheel encoder pulse counter variable
//const int encoder_pin = 2;
//volatile int encoder_count = 0;
//double wheel_rpm;
//#define ENCODER_SCALING 833.33 // Convert from pulses/ms to rpm 60000/72

bool state_paused = true;

XBee xbee = XBee();
SoftwareSerial xbeeSerial(2, 3);
ZBRxResponse rxResponse = ZBRxResponse();
ZBTxRequest txRequest;

//---------------------------------------------------------//
void setup() {
  delay(3000);
  PID_theta.SetMode(AUTOMATIC);
  PID_dist.SetMode(AUTOMATIC);
//  PID_speed.SetMode(AUTOMATIC);

  // Servo range limits
  PID_theta.SetOutputLimits(-45, 45);
  PID_dist.SetOutputLimits(-45, 45);
//  PID_speed.SetOutputLimits(-20, 20);

  steeringServo.attach(steeringPin);
  motorServo.attach(motorPin);

  // Begin serial
  Serial.begin(9600);
  xbeeStream.begin(Serial);
  xbeeStream.write("Serial begin");
  // Open and join irc bus as master
  Wire.begin();
  delay(100);

  // Set encoder pin as input and enable internal pull-up resistor
//  pinMode(encoder_pin, INPUT);
//  digitalWrite(encoder_pin, HIGH);
//  attachInterrupt(digitalPinToInterrupt(encoder_pin), encoder_ISR, CHANGE);

  // Set up the lidar pins and change the lidar acquisition count register value
  for (int i = 0; i < 2; i++)   pinMode(lidar_pwr_en[i], OUTPUT);
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++)  digitalWrite(lidar_pwr_en[j], LOW);
    digitalWrite(lidar_pwr_en[i], HIGH);
    while (1) {
      Wire.beginTransmission(LIDARLITE_ADDR);
      Wire.write(0x01);
      Wire.write(0xFF);
      if (Wire.endTransmission() == 0) break;
    }
  }

}

void loop() {
  if (xbeeStream.available() && xbeeStream.read() == 0x00) {
    state_paused = !state_paused;
    if (state_paused)  motorServo.write(90);
  }
  if (!state_paused) {
    now = millis();
    read_lidars();
    if (now - timestamp > 40) {
//      wheel_rpm = ENCODER_SCALING * encoder_count / (now - timestamp);
//      Input_speed = wheel_rpm;
//      encoder_count = 0;
      timestamp = now;
      //    dist[0] = 4.5 + dist[0] * (0.98); //scaling equation
      Input_theta = calcAngle(dist[0], dist[1]);
      Input_dist = calcPerpDist(dist[0], dist[1], Input_theta);
      Input_theta = Input_theta * 180 / pi;
      //    Setpoint_theta = constrain(0.5*(Input_dist - Setpoint_dist), -45, 45);

      // Run the PID loops
      //      xbeeStream.write(String(String(dist[0], 1) + ',' + String(dist[1], 1) + ',' + String(Input_theta, 1) + ',' + String(Input_dist, 1)));
      PID_theta.Compute();
      PID_dist.Compute();
//      PID_speed.Compute();

      motorServo.write(90  - 30);
      //      if (abs(Output_dist - Setpoint_dist) < 10 && abs(Output_theta - Setpoint_theta) < 5) {
      //        steeringServo.write(90 - Output_theta);
      //      } else {
      steeringServo.write(90 + Output_theta - Output_dist); // + Output_theta - Output_dist
      //      } // (50/abs(Input_dist - Setpoint_dist)) *
    }
  }
  delay(2);
}


//---------------------------------------------------------//

double calcAngle(double dist0, double dist1) {
  return  atan((dist0 - dist1) / LIDARspacing);
}

double calcPerpDist(double dist0, double dist1, double angle) {
  return ((dist0 + dist1) * cos(angle)) / 2;
}

//void encoder_ISR() {
//  encoder_count++;
//}


void read_lidars() {
  //  Serial.print(lidar_state);
  switch (lidar_state) {
    case 0:
      for (int i = 0; i < 2; i++) digitalWrite(lidar_pwr_en[i], LOW);
      digitalWrite(lidar_pwr_en[enabled_lidar], HIGH);
      lidar_state = 1;
      state_cycle_count = 0;
      break;
    case 1:
      Wire.beginTransmission(LIDARLITE_ADDR);
      Wire.write(LIDARLITE_CMD_CTRL_ADDR);
      Wire.write(LIDARLITE_TRIG_VAL);
      if (Wire.endTransmission() == 0) lidar_state = 2;
      else state_cycle_count++;
      break;
    case 2:
      Wire.beginTransmission(LIDARLITE_ADDR);
      Wire.write(LIDARLITE_RANGE_ADDR);
      if (Wire.endTransmission() == 0) lidar_state = 3;
      else state_cycle_count++;
      break;
    case 3:
      if (Wire.requestFrom(LIDARLITE_ADDR, 2) >= 2) {
        uint16_t val = Wire.read() << 8 | Wire.read();
        if (enabled_lidar == 0) val = 6.5 + 0.98 * val;
        if (val < 500 && val > 10) dist[enabled_lidar] = 0.2 * dist[enabled_lidar] + 0.8 * val;

        enabled_lidar = 1 - enabled_lidar;
        lidar_state = 0;
      } else state_cycle_count++;
      break;
  }
  if (state_cycle_count > 30) {
    lidar_state = 0;
    enabled_lidar = 1 - enabled_lidar;
    state_cycle_count = 0;
    // Serial.println("RST");
  }
}

