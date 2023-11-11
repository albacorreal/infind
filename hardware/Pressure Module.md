# PRESSURE MODULE
This document shows how to connect and program the BMP180 Module with an ESP32. 
## About BMP180
The BMP180 Module is a digital barometic pressure measurer. 
[See datasheet](https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf)

## Connecting the BMP180 to the ESP32 board
Bare in mind that its operating voltage is 5V. 
![Pressure Module Wiring](https://github.com/albacorreal/infind/blob/main/multimedia/BMP1820_wiring.jpg)
## Programming the BMP180 module with Adafruit_BMP085.h library
A sample code is provided below: 
~~~c++
#include <Wire.h>
#include <Adafruit_BMP085.h>
#define seaLevelPressure_hPa 1013.25

Adafruit_BMP085 bmp;
  
void setup() {
  Serial.begin(115200);
  if (!bmp.begin()) {
  Serial.println("BMP180 Not Found. CHECK CIRCUIT!");
  while (1) {}
  }
}
  
void loop() {
    Serial.print("Temperature = ");
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
    
    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print("Pressure at sealevel (calculated) = ");
    Serial.print(bmp.readSealevelPressure());
    Serial.println(" Pa");

    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude());
    Serial.println(" meters");

    Serial.print("Real altitude = ");
    Serial.print(bmp.readAltitude(seaLevelPressure_hPa * 100));
    Serial.println(" meters");
    
    Serial.println();
    delay(1000);
}
~~~
