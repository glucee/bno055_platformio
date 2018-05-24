#include "../lib/bno055/bno055.h"

void setup()
{
    Serial.begin(115200);
    setup_bno055();
    /*calabriation*/
}

void loop()
{
    ORI_DATA data = getori_bno055();
    Serial.print("Orientation: ");
    Serial.print(data.yaw);
    Serial.print(" ");
    Serial.print(data.pitch); 
    Serial.print(" ");
    Serial.println(data.roll);
    delay(20);
}
