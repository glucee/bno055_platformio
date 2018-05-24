Description

bno055 library used in platformio, the library is highly inspired by Adafruit_LSM9DS1 library

It has been tested on adafruit bno055 connecting to ESP8266 via I2C

you could easily config the platform to your chip in platformio.ini

USAGE

You can use:

make (all): to compile the firmware

make upload: to upload to your device

make clean: clean the project

make update: to update all the libraries

API function

//setup bno055, if the device is not setup successfully, it will enter forever loop

bool setup_lsm9ds1();

//option functions:

//get the yaw, roll and pitch (degrees) from LSM9DS1

ORI_DATA getori_lsm9ds1();

typedef struct {
   float yaw; 
   float roll; 
   float pitch; 
} ORI_DATA; //measured by degrees

LICENSE

MIT
