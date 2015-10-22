/*
 * EC544-Group2
 * Challenge 4 Arduino Code
 * Objective: keep the vehicle going straight down a hallway.
 * Solution: Two LIDAR-Lite v1 rangefinders, PID loops
 */

//#include <I2C.h>
#include <Servo.h>
#include <Wire.h>
#include <PID_v1.h>
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

Servo wheels; // servo for turning the wheels
Servo esc; // not actually a servo, but controlled like one!
bool startup = true; // used to ensure startup only happens once
int startupDelay = 1000; // time to pause at each calibration step
double maxSpeedOffset = 45; // maximum speed magnitude, in servo 'degrees'
double maxWheelOffset = 75; // maximum wheel turn magnitude, in servo 'degrees'


int last_0;
int last_1;
int last_PID0;

int distance;

const int switch_0 = 12;
const int switch_1 = 13;

int active_sensor = 0; //0 is left, 1 is right

// define variables we will be connecting
double Setpoint, Input, Output;
int Kp = 1.5;     // initial 2
int Ki = 0.00;  // initial 0.05
int Kd = 0.9;   // initial 0.5

PID PID0(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {

  wheels.attach(8); // initialize wheel servo to Digital IO Pin #8
  esc.attach(9); // initialize ESC to Digital IO Pin #9
  /*  If you're re-uploading code via USB while leaving the ESC powered on,
   *  you don't need to re-calibrate each time, and you can comment this part out.
   */
  calibrateESC();

  //initialize LIDAR's left on, right off
  pinMode(switch_0, OUTPUT);
  pinMode(switch_1, OUTPUT);
  digitalWrite(switch_0, HIGH);
  digitalWrite(switch_1, LOW);

  Serial.begin(9600); //Opens serial connection at 9600bps.
  Wire.begin(); // Opens & joins the irc bus as master
  delay(100); // Waits to make sure everything is powered up before sending or receiving data

  //initialize linked variables
  Setpoint = 100; //100
  PID0.SetOutputLimits(-50, 50);

  //turn on PID loop
  PID0.SetMode(AUTOMATIC);

}


/* Convert degree value to radians */
double degToRad(double degrees) {
  return (degrees * 71) / 4068;
}

/* Convert radian value to degrees */
double radToDeg(double radians) {
  return (radians * 4068) / 71;
}

/* Calibrate the ESC by sending a high signal, then a low, then middle.*/
void calibrateESC() {
  esc.write(180); // full backwards
  delay(startupDelay);
  esc.write(0); // full forwards
  delay(startupDelay);
  esc.write(90); // neutral
  delay(startupDelay);
  esc.write(90); // reset the ESC to neutral (non-moving) value
}

void loop() {
  for (int i = 0; i < 1; i++) {
    //    active_sensor = i;
    switch (i) {
      case 0:
        digitalWrite(switch_0, LOW);
        digitalWrite(switch_1, LOW);
        delay(5);
        digitalWrite(switch_0, HIGH);
        delay(5);
        break;
      case 1:
        digitalWrite(switch_0, LOW);
        digitalWrite(switch_1, LOW);
        delay(5);
        digitalWrite(switch_1, HIGH);
        delay(5);
        break;
    }
    //distance array to average distances
    int distSum = 0;

    for (int j = 0; j < 5; j++) {

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
      //low pass filter
      switch (i) {
        case 0:
          if (distance < 0 || distance > 4000) {
            distance = last_0;
          }
          distance = 0.7 * distance + 0.3 * last_0;
          last_0 = distance;
          break;
        case 1:
          if (distance < 0 || distance > 4000) {
            distance = last_1;
          }
          distance = 0.7 * distance + 0.3 * last_1;
          last_1 = distance;
          break;
      }
      distSum = distSum + distance;
      Serial.print(j);
      Serial.print(" : ");
      Serial.println(distance);
      delay(5);
    }

    distance = distSum / 5;

    //input to PID
    Input = distance;

    // use PID loop
    PID0.Compute();
    delay(5);

    Output = 0.9 * Output + 0.1 * last_PID0;

    wheels.write(90-15);
    esc.write(90 + Output);

    // Print Distance
    switch (i) {
      case 0:
        Serial.print("Dist0: ");
        Serial.print(distance);
        Serial.print(", PID0: ");
        Serial.println(Output);
        delay(1);
        break;
      case 1:
        Serial.print("Dist1: ");
        Serial.print(distance);
        Serial.print(", PID1: ");
        Serial.println(Output);
    }
    delay(10);
  }
  delay(10);
}
