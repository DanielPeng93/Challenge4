/*
 * EC544-Group2
 * Challenge 4 Arduino Code
 * Objective: keep the vehicle going straight down a hallway.
 * Solution: Two LIDAR-Lite v1 rangefinders, PID loops
 */
 
#include <I2C.h>
#include <PID_v1.h>
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

int active_sensor = 0; //0 is left, 1 is right

// define variables we will be connecting
double Setpoint_left, Input_left, Output_left;
int Kp = 2;     // initial 2
int Ki = 0.05;  // initial 0.05
int Kd = 0.5;   // initial 0.5

PID PIDleft(&Input_left, &Output_left, &Setpoint_left, Kp, Ki, Kd, DIRECT); 

void setup() {
  Serial.begin(9600); //Opens serial connection at 9600bps.     
  I2c.begin(); // Opens & joins the irc bus as master
  delay(100); // Waits to make sure everything is powered up before sending or receiving data  
  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails

  //initialize linked variables
  Setpoint_left = 30; //30
  PIDleft.SetOutputLimits(-50,50);

  //turn on PID loop
  PIDleft.SetMode(AUTOMATIC);
}

void loop() {
// Write 0x04 to register 0x00
  uint8_t nackack_left = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack_left != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack_left = I2c.write(LIDARLite_ADDRESS,RegisterMeasure, MeasureValue); // Write 0x04 to 0x00
    delay(1); // Wait 1 ms to prevent overpolling
  }

  byte distanceArray[2]; // array to store distance bytes from read function
  
  // Read 2byte distance from register 0x8f
  nackack_left = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack_left != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack_left = I2c.read(LIDARLite_ADDRESS,RegisterHighLowB, 2, distanceArray); // Read 2 Bytes from LIDAR-Lite Address and store in array
    delay(1); // Wait 1 ms to prevent overpolling
  }
  int distance_left = (distanceArray[0] << 8) + distanceArray[1];  // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
  Input_left = distance_left;

  // use PID loop 
  PIDleft.Compute();
  delay(10);
  
  // Print Distance
  Serial.print("Distance: ");
  Serial.print(distance_left);
  Serial.print(", PIDleft: ");
  Serial.println(Output_left);

}
