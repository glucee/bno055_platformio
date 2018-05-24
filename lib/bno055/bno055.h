typedef struct {
	float yaw; 
	float roll; 
	float pitch; 
} ORI_DATA; //measured by degrees

void setup_bno055();

ORI_DATA getori_bno055();

