#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <Servo.h>
#include <Wire.h>

#define LIDARLITE_ADDR 0x62
#define LIDARLITE_CMD_CTRL_ADDR 0x00
#define LIDARLITE_TRIG_VAL 0x04
#define LIDARLITE_RANGE_ADDR 0x8f

Servo wheel_speed_servo, wheel_direction_servo;
const int lidar_pwr_en[] = {
  12, 13};    // PWR_EN pins to put lidars to sleep
ros::NodeHandle nh;

sensor_msgs::Range range_msg;
ros::Publisher pub_range("range_data", &range_msg);
char frame_id[] = "/lidar_0";
int enabled_lidar = 0;
int lidar_state = 0;    // {0:Triggered, 1:Requested data, 2:Receiving data}

void vel_cmd_cb(const geometry_msgs::Twist &cmd_msg) {
  wheel_speed_servo.write(90 - 90 * cmd_msg.linear.x);
  wheel_direction_servo.write(90 + 90 * cmd_msg.angular.z);
}
ros::Subscriber<geometry_msgs::Twist> sub_vel("cmd_vel", vel_cmd_cb);

void setup() {
  Wire.begin();
  nh.initNode();
  nh.advertise(pub_range);
  nh.subscribe(sub_vel);

  range_msg.radiation_type = 2;
  range_msg.field_of_view = 0.001;
  range_msg.max_range = 40;
  range_msg.min_range = 0.1;

  wheel_speed_servo.attach(8);
  wheel_direction_servo.attach(9);
  calibrate_servo(wheel_direction_servo, 0, 180);

  enable_and_trigger(0);
}

void loop() {
  switch (lidar_state) {
  case 0:
    Wire.beginTransmission(LIDARLITE_ADDR);
    Wire.write(LIDARLITE_RANGE_ADDR);
    if (Wire.endTransmission() == 0) lidar_state = 1;
    break;
  case 1:
    if (Wire.requestFrom(LIDARLITE_ADDR, 2) > 0) lidar_state = 2;
    break;
  case 2:
    if (Wire.available() >= 2) {
      uint16_t range_data = Wire.read() << 8;
      range_data += Wire.read();
      if(range_data < 40000){
        range_msg.range = (float)range_data;
        range_msg.header.stamp = nh.now();
        frame_id[7] = enabled_lidar + '0';
        range_msg.header.frame_id = frame_id;
        pub_range.publish(&range_msg);
      }
      if (enabled_lidar == 0)  enable_and_trigger(1);
      else enable_and_trigger(0);
      lidar_state = 0;
    }
  }
  nh.spinOnce();
  delay(1);
}

void enable_and_trigger(int lidar_id) {
  // Put all lidars to sleep
  for (int i = 0; i < sizeof(lidar_pwr_en) / sizeof(*lidar_pwr_en); i++)
    digitalWrite(lidar_pwr_en[i], LOW);

  // Wake up the specified lidar and mark it's state as triggered
  digitalWrite(lidar_pwr_en[lidar_id], HIGH);
  enabled_lidar = lidar_id;

  // Trigger the signal acquisition
  while (1) {
    Wire.beginTransmission(LIDARLITE_ADDR);
    Wire.write(LIDARLITE_CMD_CTRL_ADDR);
    Wire.write(LIDARLITE_TRIG_VAL);
    if (Wire.endTransmission() == 0) break;
    delay(1);
  }
}

void calibrate_servo(Servo & servo, int minVal, int maxVal) {
  const int calibration_delay = 1000;
  const int calibration_angles[] = {
    maxVal, minVal, (minVal + maxVal) / 2, (minVal + maxVal) / 2      };
  for (int i = 0; i < sizeof(calibration_angles) / sizeof(*calibration_angles); i++) {
    servo.write(calibration_angles[i]);
    delay(calibration_delay);
  }
}



