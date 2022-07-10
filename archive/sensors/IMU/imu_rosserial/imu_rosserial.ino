#include <ros.h>
#include <std_msgs/String.h>
#include "Wire.h"

//#define DEBUG
#define IMU_ON
#define GYRO_FS 1000.0
#define ACCE_FS 2.0
#define RESOLUTION 32768 // 16 bits / 2

// timer1 2Hz prescale 1024
#define TIMER1_CMR 7812

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher imu_pub("imu", &str_msg);

const int MPU_ADDR = 0x68;
char payload[50];
float accelerometer_x, accelerometer_y, accelerometer_z;
float gyro_x, gyro_y, gyro_z;
float temperature;

bool timer1_flag;

void setup()
{
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = TIMER1_CMR;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  sei();

  nh.initNode();
  nh.advertise(imu_pub);

#ifdef DEBUG
  Serial.begin(115200);
#endif

#ifdef IMU_ON
  uint8_t resgister_temp;
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
  Wire.write((resgister_temp & (~(0x03) << 3)) | (0x02 << 3));
  Wire.endTransmission(true);
  // set accelerometer full scale range to 2g (AFS_SEL = 0)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);
  resgister_temp = Wire.read();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write((resgister_temp & (~(0x03) << 3)) | (0x00 << 3));
  Wire.endTransmission(true);
#endif
}

ISR(TIMER1_COMPA_vect) {
  timer1_flag = true;
}

float convert_acce(int16_t i) {
  return i * ACCE_FS / RESOLUTION;
}

float convert_gyro(int16_t i) {
  return i * (GYRO_FS / RESOLUTION);
}

float convert_temp(int16_t i) {
  return i / 340.00 + 36.53;
}

void read_sensor() {

  char tmp_str[6];

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 7 * 2, true);

  accelerometer_x = convert_acce(Wire.read() << 8 | Wire.read());
  accelerometer_y = convert_acce(Wire.read() << 8 | Wire.read());
  accelerometer_z = convert_acce(Wire.read() << 8 | Wire.read());
  temperature = convert_temp(Wire.read() << 8 | Wire.read());
  gyro_x = convert_gyro(Wire.read() << 8 | Wire.read());
  gyro_y = convert_gyro(Wire.read() << 8 | Wire.read());
  gyro_z = convert_gyro(Wire.read() << 8 | Wire.read());

  sprintf(payload, "");
  dtostrf(accelerometer_x, 4, 3, tmp_str);
  strcat(payload, tmp_str);
  strcat(payload, ":");
  dtostrf(accelerometer_y, 4, 3, tmp_str);
  strcat(payload, tmp_str);
  strcat(payload, ":");
  dtostrf(accelerometer_z, 4, 3, tmp_str);
  strcat(payload, tmp_str);
  strcat(payload, ":");
  dtostrf(temperature, 4, 2, tmp_str);
  strcat(payload, tmp_str);
  strcat(payload, ":");
  dtostrf(gyro_x, 4, 1, tmp_str);
  strcat(payload, tmp_str);
  strcat(payload, ":");
  dtostrf(gyro_y, 4, 1, tmp_str);
  strcat(payload, tmp_str);
  strcat(payload, ":");
  dtostrf(gyro_z, 4, 1, tmp_str);
  strcat(payload, tmp_str);


#ifdef DEBUG
  Serial.print("aX = "); Serial.print(accelerometer_x);
  Serial.print(" | aY = "); Serial.print(accelerometer_y);
  Serial.print(" | aZ = "); Serial.print(accelerometer_z);
  Serial.print(" | tmp = "); Serial.print(temperature);
  Serial.print(" | gX = "); Serial.print(gyro_x);
  Serial.print(" | gY = "); Serial.print(gyro_y);
  Serial.print(" | gZ = "); Serial.print(gyro_z);
  Serial.println();
  Serial.println(payload);
#endif
}

void loop()
{
  if (timer1_flag) {
#ifdef IMU_ON
    timer1_flag = false;
    read_sensor();
    str_msg.data = payload;
    imu_pub.publish(&str_msg);
    nh.spinOnce();
#endif
  }

}
