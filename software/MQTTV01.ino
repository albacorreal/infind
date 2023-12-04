//Libraries
#include <string>
#include <ArduinoJson.h> 

#include <PubSubClient.h>
/* insert your sensor libraries here
#include <DHTesp.h>
*/

#ifdef ESP32
#include <WiFi.h>
#endif

#ifdef ESP8266
#include <ESP8266WiFi.h>
#endif

#include <Wire.h>
#include <Adafruit_BMP085.h>
#define seaLevelPressure_hPa 1013.25
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;


//Mqtt Client
WiFiClient wClient;
PubSubClient mqtt_client(wClient);

// Update these with values suitable for your network.
const String ssid = "infind";
const String password = "1518wifi";
const String mqtt_server = "iot.ac.uma.es";
const String mqtt_user = "infind";
const String mqtt_pass = "zancudo";
String IP;
int RSSI;

// Strings for topics e ID
String ID_PLACA;
String conexion; // for LastWill & Testament
String datos_uv; // publish topic
String data; //subscribe topic

// Pin numbers
const int uv_pin = 23;

//-----------------------------------------------------
//Set Up WiFi Connection
void conecta_wifi() {
  Serial.println("Connecting to " + ssid);
 
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }
  // Get local IP
  IP = WiFi.localIP().toString();

  // Get RSSI
  RSSI = WiFi.RSSI();

  Serial.println();
  Serial.println("WiFi connected, ip address: " + IP);
}
//-----------------------------------------------------
//Set Up Client-Broker Connection
void conecta_mqtt() {
  // Loop until we're reconnected
  while (!mqtt_client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    //                                                                                WillTopic    , WillQoS, WillRetain, WillMessage
    if (mqtt_client.connect(ID_PLACA.c_str(), mqtt_user.c_str(), mqtt_pass.c_str(),conexion.c_str(),       0,    true   ,"{\"online\":false}")) {
      Serial.println(" conectado a broker: " + mqtt_server);
      mqtt_client.subscribe(data.c_str());
    } else {
      Serial.println("ERROR:"+ String(mqtt_client.state()) +" reintento en 5s" );
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
  
}

void setup() {
  // Start Up Serial Communication
   Serial.begin(115200);
  Serial.println();
  Serial.println("Empieza setup...");
  
  // ID de la placa
  #ifdef ESP32
    uint32_t id = 0;
    for(int i = 0; i < 17; i = i+8) {
      id |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
    }
    ID_PLACA = "ESP_" + String(id);
  #endif
  // chiID specific for ESP8266
  #ifdef ESP8266
    ID_PLACA = "ESP_" + String(ESP.getChipId());
  #endif

   // Topics
  conexion = "infind/GRUPO15/conexion";
  datos_uv = "infind/GRUPO15/uv";
  data = "infind/GRUPO15/led/cmd";

  conecta_wifi();   //establecemos conexión wifi

  //Configuraciones del cliente mqtt
  mqtt_client.setServer(mqtt_server.c_str(), 1883);
  mqtt_client.setBufferSize(512); // para poder enviar mensajes de hasta X bytes
  //mqtt_client.setCallback(callback);  //para procesar los mensajes cada vez que llegue uno al topic que estamos suscritos.

  conecta_mqtt(); //establecemos la conexión del cliente con el servidor

  //Mensaje formateado en JSON con el estado de conexión a true
  String mensaje="{\"online\": ""true"" }";
  
  //Imprimimos mensaje y topic en el monitor serie
  Serial.println();
  Serial.println("Topic   : "+ conexion);
  Serial.println("Payload : "+ mensaje);
 
 //Publicamos el mensaje
  mqtt_client.publish(conexion.c_str(), mensaje.c_str(),true);

  if (!bmp.begin()) {
  Serial.println("BMP180 Not Found. CHECK CIRCUIT!");
  while (1) {}
  }

  Serial.println("Adafruit MPU6050 test!");
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  //Mensajes de finalización de setup 
  Serial.println("Identificador placa: "+ ID_PLACA);
  Serial.println("Termina setup en " +  String(millis()) + " ms");
 
}
unsigned long ultimo_mensaje = 0;

void loop() {
 if (!mqtt_client.connected())  //si no está conectado el cliente al broker
  {
    conecta_mqtt();   //establecemos la conexión

    //Mensaje formateado en JSON con el estado de conexión a true
    String mensaje = "{\"online\": ""true"" }";

    //Imprimimos mensaje y topic en el monitor serie
    Serial.println();
    Serial.println("Topic   : "+ conexion);
    Serial.println("Payload : "+ mensaje);

    //Publicamos el mensaje
    mqtt_client.publish(conexion.c_str(), mensaje.c_str(),true);
  }
  mqtt_client.loop(); // esta llamada para que la librería recupere el control

  unsigned long ahora = millis();
    
  if (ahora - ultimo_mensaje >= 5000) {
    ultimo_mensaje = ahora;

    // Lectura del sensor  
    float dato_uv = analogRead(uv_pin);
    
    //Mensaje formateado en JSON con los datos de los sensores y de conexión Wifi 
    String mensaje = "{\"Radiacion\":"+ String(dato_uv) +"}";
   
    //Imprimimos el mensaje en el monitor serie
    Serial.println();
    Serial.println("Topic   : "+ datos_uv);
    Serial.println("Payload : "+ mensaje);
   
    //Publicamos el mensaje
    mqtt_client.publish(datos_uv.c_str(), mensaje.c_str());
  
  }

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
  delay(10000);


  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);
}

