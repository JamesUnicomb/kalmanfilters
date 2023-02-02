#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define BUTTON_A 5
#define BUTTON_B 6
#define BUTTON_C 9

#define ACCLDRDY 10
#define GYRODRDY 11
#define MAGDRDY 13

#define LIS3MDL_CX -32.242035865 // values used after calibration
#define LIS3MDL_CY 55.524701594999996 // values used after calibration
#define LIS3MDL_CZ -125.12423706500002 // values used after calibration

#define GPSSerial Serial3

Adafruit_LSM6DSOX sox;
Adafruit_LIS3MDL lis3mdl;
Adafruit_GPS GPS(&GPSSerial);
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;
sensors_event_t mag;

double ax, ay, az;
double gx, gy, gz;
double mx, my, mz;
double lon, lat, alt;
int year, month, day, hour, minute, second;

void setup(void)
{
	GPS.begin(9600);
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
	GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
	GPS.sendCommand(PGCMD_ANTENNA);

	if(!sox.begin_I2C())
	{
		while(1)
		{
			delay(10);
		}
	}

	sox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
	sox.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
	sox.setAccelDataRate(LSM6DS_RATE_52_HZ);
	sox.setGyroDataRate(LSM6DS_RATE_52_HZ);
	sox.configInt1(false, false, true, false, false);
	sox.configInt2(false, true, false);

	if(!lis3mdl.begin_I2C())
	{
		while(1)
		{
			delay(10);
		}
	}

	lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
	lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
	lis3mdl.setDataRate(LIS3MDL_DATARATE_40_HZ);
	lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

	delay(200);
	display.begin(0x3C, true);
	display.clearDisplay();
	display.display();
	delay(200);

	display.setRotation(1);

	pinMode(BUTTON_A, INPUT_PULLUP);
	pinMode(BUTTON_B, INPUT_PULLUP);
	pinMode(BUTTON_C, INPUT_PULLUP);

	pinMode(ACCLDRDY, INPUT_PULLUP);
	pinMode(GYRODRDY, INPUT_PULLUP);
	pinMode(MAGDRDY, INPUT_PULLUP);

	display.setTextSize(1);
	display.setTextColor(SH110X_WHITE);
	display.setCursor(0, 0);
	display.clearDisplay();
	display.display();
}

uint32_t timer = millis();

void loop()
{
	/* Get GPS Event */
	GPS.read();
	if(GPS.newNMEAreceived())
	{
		Serial.print(GPS.lastNMEA());
		if(!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
			return; // we can fail to parse a sentence in which case we should just wait for another
	}

	/* get gyro/accl event */
	if(digitalRead(ACCLDRDY) && digitalRead(GYRODRDY))
	{
		sox.getEvent(&accel, &gyro, &temp);
		ax = accel.acceleration.x;
		ay = accel.acceleration.y;
		az = accel.acceleration.z;
		gx = gyro.gyro.x;
		gy = gyro.gyro.y;
		gz = gyro.gyro.z;
	}

	/* get mag event */
	if(digitalRead(MAGDRDY))
	{
		lis3mdl.getEvent(&mag);
		mx = mag.magnetic.x - LIS3MDL_CX;
		my = mag.magnetic.y - LIS3MDL_CY;
		mz = mag.magnetic.z - LIS3MDL_CZ;
		Serial.print(mx);
		Serial.print(",");
		Serial.print(my);
		Serial.print(",");
		Serial.print(mz);
		Serial.println();
	}

	if(GPS.fix)
	{
		lon = GPS.longitudeDegrees;
		lat = GPS.latitudeDegrees;
		alt = 1e-3 * GPS.altitude;

		year = GPS.year;
		month = GPS.month;
		day = GPS.day;
		hour = GPS.hour;
		minute = GPS.minute;
		second = GPS.seconds;
	}

	if(millis() - timer > 100)
	{
		timer = millis();

		display.clearDisplay();
		display.setCursor(0, 0);
		display.print("utc:   ");
		display.println();
		display.print("20");
		if(year < 10)
			display.print("0");
		display.print(year, DEC);
		display.print("-");
		if(month < 10)
			display.print("0");
		display.print(month);
		display.print("-");
		if(day < 10)
			display.print("0");
		display.print(day);
		display.print("T");
		if(hour < 10)
			display.print("0");
		display.print(hour);
		display.print(":");
		if(minute < 10)
			display.print("0");
		display.print(minute);
		display.print(":");
		if(second < 10)
			display.print("0");
		display.print(second);
		display.println();

		display.print("lon:   ");
		display.print(lon, 6);
		display.println();
		display.print("lat:   ");
		display.print(lat, 6);
		display.println();
		display.print("alt:   ");
		display.print(alt, 6);
		display.println();
		display.print("roll:  ");
		display.print(ax, 6);
		display.println();
		display.print("pitch: ");
		display.print(ay, 6);
		display.println();
		display.print("yaw:   ");
		display.print(az, 6);
		display.println();
		yield();
		display.display();
	}
}
