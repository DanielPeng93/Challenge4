
// Two LIDARs on one side

#include <Wire.h>
#include <math.h>
#include <PID_v1.h>
#include <Servo.h>

#define LIDARLITE_ADDR 0x62
#define LIDARLITE_CMD_CTRL_ADDR 0x00
#define LIDARLITE_TRIG_VAL 0x04
#define LIDARLITE_RANGE_ADDR 0x8f

// timestamps
unsigned long now, timestamp;

const int lidar_pwr_en[] = {
  12, 13
};    // PWR_EN pins to put lidars to sleep
int enabled_lidar = 0;
int lidar_state = 0;    // {0:Disabled, 1:Enabled, 2:Triggered, 3:Requested data}
int state_cycle_count = 0;
double dist[2];

//set up servos
Servo steeringServo;
Servo motorServo;
const int steeringPin = 9;
const int motorPin = 8;

//define PID variables
double Setpoint_theta, Input_theta, Output_theta, Setpoint_dist, Input_dist, Output_dist;
//specify initial PID links and tuning
int KpT = 1.7;    //1.7
int KiT = 0;      //0
int KdT = 0.01;   //0.01
int KpD = 1.1;      //1
int KiD = 0.075;    //0
int KdD = 0.97;   //0
PID PID_theta(&Input_theta, &Output_theta, &Setpoint_theta, KpT, KiT, KdT, DIRECT);
PID PID_dist(&Input_dist, &Output_dist, &Setpoint_dist, KpD, KiD, KdD, DIRECT);

//Spacing between LIDARs on vehicle
double LIDARspacing = 22.5;

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
  PID_theta.SetOutputLimits(-50, 50);
  PID_dist.SetOutputLimits(-50, 50);

  steeringServo.attach(steeringPin);
  motorServo.attach(motorPin);

  // Begin serial
  Serial.begin(57600);
  Serial.println("Serial begin");
  // Open and join irc bus as master
  Wire.begin();
  delay(100);

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
  now = millis();
  read_lidars();
  if (now - timestamp > 40) {
    timestamp = now;
//    dist[0] = 4.5 + dist[0] * (0.98); //scaling equation
    Input_theta = calcAngle(dist[0], dist[1]);
    Input_dist = calcPerpDist(dist[0], dist[1], Input_theta);
    Input_theta = Input_theta * 180 / pi;
//    Setpoint_theta = constrain(0.5*(Input_dist - Setpoint_dist), -45, 45);

    // Run the PID loops
    Serial.print(dist[0]);
    Serial.print(", ");
    Serial.print(dist[1]);
    Serial.print(", ");
    Serial.print(Input_theta);
    Serial.print(", ");
    Serial.println(Input_dist);
    PID_theta.Compute();
    PID_dist.Compute();

    motorServo.write(90  - 20 );
    steeringServo.write(90 + (Output_theta) - Output_dist); // + Output_theta - Output_dist
  } // (50/abs(Input_dist - Setpoint_dist)) * 
  delay(2);
}


//---------------------------------------------------------//

double calcAngle(double dist0, double dist1) {
  return  atan((dist0 - dist1) / LIDARspacing);
}

double calcPerpDist(double dist0, double dist1, double angle) {
  return ((dist0 + dist1) * cos(angle)) / 2;
}


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
    Serial.println("RST");
  }
}

