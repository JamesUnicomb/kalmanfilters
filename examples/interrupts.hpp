#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Wire.h> 

#define LIS3M_MAG_INT 9
#define LSM6S_ACCL_INT 10
#define LSM6S_GYRO_INT 11

#define LIS3MDL_CX -31.058171275 
#define LIS3MDL_CY 4.47968483 
#define LIS3MDL_CZ 9.12744713

Adafruit_LSM6DSOX sox;
Adafruit_LIS3MDL lis3mdl;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;
sensors_event_t mag; 

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); 

  if (!sox.begin_I2C()) {
    while (1) {
      delay(10);
    }
  }

  sox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS );
  sox.setAccelDataRate(LSM6DS_RATE_104_HZ);
  sox.setGyroDataRate(LSM6DS_RATE_104_HZ);
  sox.configInt1(false, false, true, false, false);
  sox.configInt2(false, true, false);

  if (!lis3mdl.begin_I2C()) {
    while (1) { 
      delay(10); 
    }
  }

  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  pinMode(LSM6S_ACCL_INT, INPUT_PULLUP);
  pinMode(LSM6S_GYRO_INT, INPUT_PULLUP);
  pinMode(LSM6S_GYRO_INT, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LSM6S_ACCL_INT), acclCB, RISING);
  attachInterrupt(digitalPinToInterrupt(LSM6S_GYRO_INT), gyroCB, RISING);
  attachInterrupt(digitalPinToInterrupt(LIS3M_MAG_INT), magCB, RISING);
}

void acclCB() {
  Serial.print("accl:"); 
  Serial.print(micros()); Serial.print(",");
  Serial.print(accel.acceleration.x, 8); Serial.print(",");
  Serial.print(accel.acceleration.y, 8); Serial.print(",");
  Serial.print(accel.acceleration.z, 8); Serial.println();
}

void gyroCB() {
  Serial.print("gyro:");
  Serial.print(micros()); Serial.print(",");
  Serial.print(gyro.gyro.x, 8); Serial.print(",");
  Serial.print(gyro.gyro.y, 8); Serial.print(",");
  Serial.print(gyro.gyro.z, 8); Serial.println();
}

void magCB() {
  Serial.print("mag:");
  Serial.print(micros()); Serial.print(",");
  Serial.print(mag.magnetic.x - LIS3MDL_CX, 8); Serial.print(",");
  Serial.print(mag.magnetic.y - LIS3MDL_CY, 8); Serial.print(","); 
  Serial.print(mag.magnetic.z - LIS3MDL_CZ, 8); Serial.println(); 
}

void loop() {
  sox.getEvent(&accel, &gyro, &temp);
  lis3mdl.getEvent(&mag);
}