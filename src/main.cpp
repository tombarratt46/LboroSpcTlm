#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>


// 84 bytes
struct reading
{
  //12 bytes
  float magnetometer_x;
  float magnetometer_y;
  float magnetometer_z;

  //12 bytes
  float accelerometer_x;
  float accelerometer_y;
  float accelerometer_z;

  //12 bytes
  float gyroscope_x;
  float gyroscope_y;
  float gyroscope_z;

  //12 bytes
  float gravity_x;
  float gravity_y;
  float gravity_z;

  //32 bytes
  double orientation_w;
  double orientation_x;
  double orientation_y;
  double orientation_z;

  // 4 bytes
  unsigned long VOT;

};


// Quaternion
// Angular Velocity 100Hz
// Linear Acceleration 100Hz
// Magnetic Field strength 20Hz
// Temperature 1Hz
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

unsigned long BNO055_sample_rate = 100;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  if (!bno.begin())
  {
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    };
  }

  delay(500);
}

void loop() {

  delay(1.0 / BNO055_sample_rate * 1000.0);
}

// Raw data from magnetometer 
void getMagnetometerFromEvent(reading* current_reading){
  sensors_event_t event;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_MAGNETOMETER);


  current_reading->magnetometer_x = event.magnetic.x;
  current_reading->magnetometer_y = event.magnetic.y;
  current_reading->magnetometer_z = event.magnetic.z;
}

// Raw data from magnetometer 
void getAccelerometerFromEvent(reading* current_reading){
  sensors_event_t event;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  current_reading->accelerometer_x = event.acceleration.x;
  current_reading->accelerometer_y = event.acceleration.y;
  current_reading->accelerometer_z = event.acceleration.z;
}

// Raw data from magnetometer 
void getGyroscopeFromEvent(reading* current_reading){
  sensors_event_t event;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);

  current_reading->gyroscope_x = event.gyro.x;
  current_reading->gyroscope_y = event.gyro.y;
  current_reading->gyroscope_z = event.gyro.z;
}

void getGravity(reading* current_reading){
  sensors_event_t event;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_GRAVITY);

  current_reading->gravity_x = event.acceleration.x;
  current_reading->gravity_y = event.acceleration.y;
  current_reading->gravity_z = event.acceleration.z;
}

void getOrientation(reading* current_reading){
  imu::Quaternion quat;
  quat = bno.getQuat();

  current_reading->orientation_w = quat.w();
  current_reading->orientation_x = quat.x();
  current_reading->orientation_y = quat.y();
  current_reading->orientation_z = quat.z();
}