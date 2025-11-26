#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
Adafruit_MPU6050 mpu;
Servo esc;
const int escPin = 9;
int throttleUs = 1000;
void writeESC(int us) {
  if (us < 1000) us = 1000;
  if (us > 2000) us = 2000;
  esc.writeMicroseconds(us);
}
void setup() {
  Serial.begin(115200);
  Wire.begin();
  // Initialize MPU6050
  if (!mpu.begin(0x68)) {
    if (!mpu.begin(0x69)) {
      Serial.println("MPU6050_FAIL");
      while (1);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  esc.attach(escPin);
  delay(2000);          // let ESC power up
  writeESC(1000);       // arm ESC at idle
  delay(3000);
  Serial.println("READY");
}
void loop() {
  // 1) Read IMU
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  // 2) Send IMU data to MATLAB
  Serial.print(a.acceleration.x);
  Serial.print(',');
  Serial.print(a.acceleration.z);
  Serial.print(',');
  Serial.println(g.gyro.y);
  // 3) Read new throttle command from MATLAB
  if (Serial.available()) {
    int val = Serial.parseInt();
    if (val >= 1000 && val <= 2000) {
      throttleUs = val;
      writeESC(throttleUs);
    }
  }
  delay(5);  // ~200 Hz
}