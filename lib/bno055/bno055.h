#ifndef BNO055_H
#define BNO055_H
typedef struct {
	float yaw; 
	float roll; 
	float pitch; 
} ORI_DATA; //measured by degrees

extern void setup_bno055();
extern void getcal_bno055();
extern void get_and_display_sensor_offsets();
ORI_DATA getori_bno055();
void reset_bno055();

#endif