#include <bno055.h>
#include <stdlib.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.
   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground
   History
   =======
   2015/MAR/03  - First release (KTOWN)
   2018/MAY/18  - Second release Glucee
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
    Serial.print("\n");
}

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup_bno055(void)
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Calibration Data*/
  adafruit_bno055_offsets_t newCalib = {-273, 0, -273, 0, 0, -1, -129, 278, 46, 1000, 750};
  displaySensorOffsets(newCalib);
  Serial.println("\n\nRestoring Calibration data to the BNO055...");
  bno.setSensorOffsets(newCalib);

  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Calibrate Again*/
  while(!bno.isFullyCalibrated())
  {
            delay(BNO055_SAMPLERATE_DELAY_MS);
  }
  Serial.println("\nFully calibrated!");
  Serial.println("--------------------------------");
  Serial.println("New Calibration Results: ");
  bno.getSensorOffsets(newCalib);
  displaySensorOffsets(newCalib);
  bno.setSensorOffsets(newCalib);
  Serial.println("Please update these Calibration Results into the new calib array");
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
ORI_DATA getori_bno055(void)
{
  /* Board layout:
         +----------+
         |         *| RST   PITCH  ROLL  HEADING
     ADR |*        *| SCL
     INT |*        *| SDA     ^            /->
     PS1 |*        *| GND     |            |
     PS0 |*        *| 3VO     Y    Z-->    \-X
         |         *| VIN
         +----------+
  */

  /* The processing sketch expects data as roll, pitch, heading, this only works in a limited angle range */
  /*
  sensors_event_t event;
  bno.getEvent(&event);

  Serial.print(F("Orientation: "));
  Serial.print((float)event.orientation.x);
  Serial.print(F(" "));
  Serial.print((float)event.orientation.y);
  Serial.print(F(" "));
  Serial.print((float)event.orientation.z);
  Serial.println(F(""));
  */

  /* Get Orientation from Quaternion Data
  this works much better than getting it directly from chip*/
  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> ori = quat.toEuler();

  ORI_DATA ori_data;
  ori_data.roll = ori.x()*180.0f/PI;
  ori_data.pitch = ori.y()*180.0f/PI;
  ori_data.yaw = ori.z()*180.0f/PI;

  /* The processing sketch expects data as roll, pitch, heading */
  /*
  Serial.print(F("Orientation: "));
  Serial.print((float)ori.x()*180/3.14);
  Serial.print(F(" "));
  Serial.print((float)ori.y()*180/3.14);
  Serial.print(F(" "));
  Serial.print((float)ori.z()*180/3.14);
  Serial.println(F(""));
  */

  delay(BNO055_SAMPLERATE_DELAY_MS);
  return ori_data;
}

void getcal_bno055()
{
  /* Also send calibration data for each sensor. */
  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print(F("Calibration: "));
  Serial.print(sys, DEC);
  Serial.print(F(" "));
  Serial.print(gyro, DEC);
  Serial.print(F(" "));
  Serial.print(accel, DEC);
  Serial.print(F(" "));
  Serial.println(mag, DEC);
}

