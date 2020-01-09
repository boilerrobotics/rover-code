/*
   rosserial Publisher Example
   Prints "hello world!"
*/

#include <ros.h>
#include <std_msgs/String.h>
#include "Wire.h"

#define DEBUG
#define GYRO_FS 1000.0
#define ACCE_FS 2.0
#define RESOLUTION 32768 // 16 bits / 2

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher imu_pub("imu", &str_msg);

char hello[13] = "hello world!";
const int MPU_ADDR = 0x68;
int16_t accelerometer_x, accelerometer_y, accelerometer_z;
int16_t gyro_x, gyro_y, gyro_z;
int16_t temperature;
uint8_t resgister_temp;

void setup()
{
  nh.initNode();
  nh.advertise(imu_pub);
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  // set gyroscope full scale range to 1000 degree/sec (FS_SEL = 2)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);
  resgister_temp = Wire.read();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write((resgister_temp & (0x00) << 3) | (0x02 << 3));
  Wire.endTransmission(true);
  // set accelerometer full scale range to 2g (AFS_SEL = 0)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);
  resgister_temp = Wire.read();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write((resgister_temp & (0x00) << 3) | (0x00 << 3));
  Wire.endTransmission(true);
}

float convert_acce(int16_t i) {
//  Serial.println(ACCE_FS / RESOLUTION);
  return i * ACCE_FS / RESOLUTION;
}

float convert_gyro(int16_t i) {
  return i * (GYRO_FS / RESOLUTION) ;
}

void read_sensor() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 7 * 2, true);

  accelerometer_x = Wire.read() << 8 | Wire.read();
  accelerometer_y = Wire.read() << 8 | Wire.read();
  accelerometer_z = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() << 8 | Wire.read();
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();
#ifdef DEBUG
  Serial.print("aX = "); Serial.print(convert_acce(accelerometer_x));
  Serial.print(" | aY = "); Serial.print(convert_acce(accelerometer_y));
  Serial.print(" | aZ = "); Serial.print(convert_acce(accelerometer_z));
  Serial.print(" | tmp = "); Serial.print(temperature / 340.00 + 36.53);
  Serial.print(" | gX = "); Serial.print(convert_gyro(gyro_x));
  Serial.print(" | gY = "); Serial.print(convert_gyro(gyro_y));
  Serial.print(" | gZ = "); Serial.print(convert_gyro(gyro_z));
  Serial.println();
#endif
}

void loop()
{
  read_sensor();
  str_msg.data = hello;
  imu_pub.publish(&str_msg);
  nh.spinOnce();
  delay(1000);
}
