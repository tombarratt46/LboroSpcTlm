#include <main.h>

#define SAMPLE_RATE 100

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  
  bno = Adafruit_BNO055();
  bmp = Adafruit_BMP280();

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

  delay(1.0 / SAMPLE_RATE * 1000.0);
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