#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Wire.h> 

#define LIS3MDL_CX -35.9544001 // values used after calibration
#define LIS3MDL_CY 9.0032138 // values used after calibration
#define LIS3MDL_CZ 11.2905578 // values used after calibration

Adafruit_LSM6DSOX sox;
Adafruit_LIS3MDL lis3mdl;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); 
  } 

  if (!sox.begin_I2C()) {
    while (1) {
      delay(10);
    }
  }

  sox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
  sox.setAccelDataRate(LSM6DS_RATE_52_HZ);
  sox.setGyroDataRate(LSM6DS_RATE_52_HZ);
  sox.configInt1(false, false, true, false, false);
  sox.configInt2(false, true, false);

  if (!lis3mdl.begin_I2C()) {
    while (1) { 
      delay(10); 
    }
  }

  lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_40_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
}

void loop() {
   /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sox.getEvent(&accel, &gyro, &temp);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("accl:"); 
  Serial.print(micros()); Serial.print(",");
  Serial.print(accel.acceleration.x, 8); Serial.print(",");
  Serial.print(accel.acceleration.y, 8); Serial.print(",");
  Serial.print(accel.acceleration.z, 8); Serial.println();

  /* Display the results (rotation is measured in rad/s) */
  Serial.print("gyro:"); 
  Serial.print(micros()); Serial.print(",");
  Serial.print(gyro.gyro.x, 8); Serial.print(",");
  Serial.print(gyro.gyro.y, 8); Serial.print(",");
  Serial.print(gyro.gyro.z, 8); Serial.println();

  sensors_event_t event; 
  lis3mdl.getEvent(&event);

  /* Display the results (magnetic field is measured in uTesla) */
  Serial.print("mag:"); 
  Serial.print(micros()); Serial.print(",");
  Serial.print(event.magnetic.x - LIS3MDL_CX, 8); Serial.print(",");
  Serial.print(event.magnetic.y - LIS3MDL_CY, 8); Serial.print(","); 
  Serial.print(event.magnetic.z - LIS3MDL_CZ, 8); Serial.println(); 
  
  // /* delay sensor reads */
  delay(50);
}