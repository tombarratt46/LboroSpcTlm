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

char filename[8];

// Quaternion
// Angular Velocity 100Hz
// Linear Acceleration 100Hz
// Magnetic Field strength 20Hz
// Temperature 1Hz
Adafruit_BNO055 bno = Adafruit_BNO055();
Adafruit_BMP280 bmp = Adafruit_BMP280();
File datafile;
unsigned long BNO055_sample_rate = 100;
float ground_pressure;
int writeCount = 0;
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // blink once repeatedly if bno fails to start
  if (!bno.begin())
  {
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
    };
  }
  // blink twice repeatedly if bmp fails to start
  if (!bmp.begin())
    {
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
    };
  }

    // blink three times repeatedly if SD fails to start
  if (!SD.begin(10)) // CS pin 10
    {
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
    };
  }

  // Approx reading @ 26.3hz
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, // Operating Mode
                  Adafruit_BMP280::SAMPLING_X2, // Temperature sampling mode
                  Adafruit_BMP280::SAMPLING_X16, // Pressure sampling mode
                  Adafruit_BMP280::FILTER_X16, // Filtering mode
                  Adafruit_BMP280::STANDBY_MS_1 // Standby time
                  );
  ground_pressure = bmp.readPressure();

  int n = 0;
  do
  {
     n++;
     snprintf(filename, sizeof(filename), "%04d.dat", n);
  } while (SD.exists(filename));

  datafile = SD.open(filename, FILE_WRITE);

  delay(500);
}

void loop() {
  reading current_reading;

  getMagnetometer(&current_reading);
  getAccelerometer(&current_reading);
  getGyroscope(&current_reading);
  getGravity(&current_reading);
  getOrientation(&current_reading);
  getTemperature(&current_reading);
  getPressure(&current_reading);
  getAltitude(&current_reading);
  current_reading.VOT = micros();

  datafile.write((uint8_t *) &current_reading, sizeof(reading)/sizeof(uint8_t));
  writeCount++;


  //Every ~10 seconds (10,000 writes at 100hz) close and open the file to rebuild file structure
  // Unsure if this is actually needed, need to check
  if (writeCount >= 10000){
    datafile.close();
    datafile = SD.open(filename, FILE_WRITE);
    writeCount = 0;
  }

  delay(1.0 / BNO055_sample_rate * 1000.0);
}

// Raw data from magnetometer 
void getMagnetometer(reading* current_reading){
  sensors_event_t event;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_MAGNETOMETER);


  current_reading->magnetometer_x = event.magnetic.x;
  current_reading->magnetometer_y = event.magnetic.y;
  current_reading->magnetometer_z = event.magnetic.z;
}

// Raw data from magnetometer 
void getAccelerometer(reading* current_reading){
  sensors_event_t event;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  current_reading->accelerometer_x = event.acceleration.x;
  current_reading->accelerometer_y = event.acceleration.y;
  current_reading->accelerometer_z = event.acceleration.z;
}

// Raw data from magnetometer 
void getGyroscope(reading* current_reading){
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


void getTemperature(reading* current_reading){
  current_reading->temperature = bmp.readTemperature() + 273.15;
}

void getPressure(reading* current_reading){
  current_reading->pressure = bmp.readPressure();
  
}

void getAltitude(reading* current_reading){
  current_reading->altitude = bmp.readAltitude(ground_pressure / 100);
}