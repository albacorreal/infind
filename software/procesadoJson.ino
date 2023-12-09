
#include <ArduinoJson.h>

//-----------------------------------------------------------
// deserializeJson
//-----------------------------------------------------------
void deserializeJSON (String respuesta, int* max_temp, int* min_temp)
{
  StaticJsonDocument<128> root;
  deserializeJson(root, respuesta);
  *max_temp = root["max_temp"];
  *min_temp = root["min_temp"];
}


//-----------------------------------------------------------
// serializeJson
//-----------------------------------------------------------
void serializeObject(String * mensaje) {
  
    StaticJsonDocument<1024>doc;
    doc["MAC"] = 
    doc["Time"] = 
    doc["GPS"] = 
    doc["Pressure"] =
    doc["Radiation"] = 
    doc["Altitude"]
    doc["Temperature"]
    doc["Humidity"]
    doc["accelerometer"]["acceleration"]["x"] = 
    doc["accelerometer"]["acceleration"]["y"] = 
    doc["accelerometer"]["acceleration"]["z"] = 
    doc["accelerometer"]["gyro"]["x"] = 
    doc["accelerometer"]["gyro"]["y"] = 
    doc["accelerometer"]["gyro"]["z"] = 

    serializeJson(doc,mensaje);

}



void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
