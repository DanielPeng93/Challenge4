/*
 * EC544-Group2
 * Challenge 4 Arduino Code
 * Objective: keep the vehicle going straight down a hallway.
 * Solution: Two LIDAR-Lite v1 rangefinders, PID loops
 */

//#include <I2C.h>
#include <Wire.h>
#include <PID_v1.h>
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

int last_0;
int last_1;

int distance;

const int switch_0 = 12;
const int switch_1 = 13;

int active_sensor = 0; //0 is left, 1 is right

// define variables we will be connecting
double Setpoint, Input, Output;
int Kp = 2;     // initial 2
int Ki = 0.05;  // initial 0.05
int Kd = 0.5;   // initial 0.5

PID PIDleft(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {

  //initialize LIDAR's left on, right off
  pinMode(switch_0, OUTPUT);
  pinMode(switch_1, OUTPUT);
  digitalWrite(switch_0, HIGH);
  digitalWrite(switch_1, LOW);

  Serial.begin(9600); //Opens serial connection at 9600bps.
  Wire.begin(); // Opens & joins the irc bus as master
  delay(100); // Waits to make sure everything is powered up before sending or receiving data

  //initialize linked variables
  Setpoint = 30; //30
  PIDleft.SetOutputLimits(-50, 50);

  //turn on PID loop
  PIDleft.SetMode(AUTOMATIC);

}

void loop() {
  for (int i = 0; i < 2; i++) {
    //    active_sensor = i;
    switch (i) {
      case 0:
        digitalWrite(switch_0, LOW);
        digitalWrite(switch_1, LOW);
        delay(10);
        digitalWrite(switch_0, HIGH);
        delay(10);
        break;
      case 1:
        digitalWrite(switch_0, LOW);
        digitalWrite(switch_1, LOW);
        delay(10);
        digitalWrite(switch_1, HIGH);
        delay(10);
        break;
    }
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
        distance = 0.7 * distance + 0.3 * last_0;
        last_0 = distance;
        break;
      case 1:
        distance = 0.7 * distance + 0.3 * last_1;
        last_1 = distance;
        break;
    }

    //input to PID
    Input = distance;

    // use PID loop
    PIDleft.Compute();
    delay(10);

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
    delay(500);
  }
}
