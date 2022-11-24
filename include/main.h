#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <SD.h>

// 84 bytes
struct reading
{
  // Magnetic Field Strength Vector (uT)
  //12 bytes
  float magnetometer_x;
  float magnetometer_y;
  float magnetometer_z;

  // Acceleration Vector (m/s^2)
  //12 bytes
  float accelerometer_x;
  float accelerometer_y;
  float accelerometer_z;

  // Angular Velocity Vector (rad/s)
  //12 bytes
  float gyroscope_x;
  float gyroscope_y;
  float gyroscope_z;

  // Gravity Vector (m/s^2)
  //12 bytes
  float gravity_x;
  float gravity_y;
  float gravity_z;

  // Absolute Orientation Quaterion
  //16 bytes
  double orientation_w;
  double orientation_x;
  double orientation_y;
  double orientation_z;

  // Vehicle on time in microseconds
  // 4 bytes
  unsigned long VOT;

  // 12 bytes
  float temperature; //Kelvin
  float pressure; //Pa
  float altitude; // Meters (from AGL)
};

void getMagnetometer(reading* currentReading);
void getAccelerometer(reading* currentReading);
void getGyroscope(reading* currentReading);
void getGravity(reading* currentReading);
void getOrientation(reading* currentReading);
void getTemperature(reading* currentReading);
void getPressure(reading* currentReading);
void getAltitude(reading* currentReading);


Adafruit_BNO055 bno;
Adafruit_BMP280 bmp;
File datafile;
char filename[8];
float ground_pressure;
int writeCount = 0;