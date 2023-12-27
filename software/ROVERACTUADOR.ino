//Libraries
#include <string>
#include <ArduinoJson.h> 

#include <PubSubClient.h>
/* insert your sensor libraries here
#include <DHTesp.h>
*/
// #ifdef ESP8266
// #include <ESP8266WiFi.h>
// #endif

#ifdef ESP32
  #include <esp_now.h>
  #include <WiFi.h>
#endif

#include <Wire.h>
//#include <Adafruit_BMP085.h>
//#define seaLevelPressure_hPa 1013.25
#include <Adafruit_MPU6050.h> // Acelerometro
#include <Adafruit_Sensor.h>

#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <queue>


Adafruit_MPU6050 mpu;
//Adafruit_BMP085 bmp;

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
String emergencia; // publish topic
String data; //subscribe topic

// Pin numbers
//const int uv_pin = 23;

//-----------------------------------------------------
// ESP-NOW

// MAC receptora ESP32 victor para ESP-NOW
uint8_t broadcastAddress[] = {0x24, 0xDC, 0xC3, 0xA7, 0x31, 0x48};// REPLACE WITH RECEIVER MAC ADDRESS

// typedef struct struct_message {
//  	char a[32];
//  	int b;
//  	float c;
//  	String aviso;   
//  	bool e;     // 
// } struct_message; 

String emergency_message; // DATOS QUE SE ENVIARAN A ROBER SENSOR
char dataRcv[15];
uint MAX_SIZE = ;

// callbacks for sending and receiving data
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
 	Serial.print("\r\nMaster packet sent:\t");
 	Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
 	memcpy(&dataRcv, incomingData, sizeof(dataRcv));
 	Serial.print("\r\nBytes received: ");
 	Serial.println(len);
 	Serial.print("From slave: ");
 	Serial.println(dataRcv);
 	Serial.println();
}

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
  data = "infind/GRUPO15/led/cmd";
  ubicacionGPS = "infind/GRUPO15/...";
  emergencia = "infind/GRUPO15/...";  // Emergencia caida, terremoto, carro lleno de muestras

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
/*
  if (!bmp.begin()) {
  Serial.println("BMP180 Not Found. CHECK CIRCUIT!");
  //while (1) {}
  }
*/
  Serial.println("Adafruit MPU6050 test!");
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    //while (1) {
    //  delay(10);
    //}
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
*/
  //Mensajes de finalización de setup 
  Serial.println("Identificador placa: "+ ID_PLACA);
  Serial.println("Termina setup en " +  String(millis()) + " ms");

//-----------------------------------------------------------
//SET-UP ESP-NOW
// Init Serial Monitor
 	//Serial.begin(115200);

 	// Set device as a Wi-Fi Station
 	WiFi.mode(WIFI_STA);

 	// Init ESP-NOW
 	if (esp_now_init() != ESP_OK) {
 			Serial.println(F("Error initializing ESP-NOW"));
 			return;
 	}
 	Serial.print(F("Reciever initialized : "));
 	Serial.println(WiFi.macAddress());
 	
 	// Define callback functions
 	esp_now_register_send_cb(OnDataSent);
 	esp_now_register_recv_cb(OnDataRecv);

 	// Register peer
 	esp_now_peer_info_t peerInfo;
 	memcpy(peerInfo.peer_addr, broadcastAddress, 6);
 	peerInfo.channel = 0;
 	peerInfo.encrypt = false;

 	// Add peer
 	if (esp_now_add_peer(&peerInfo) != ESP_OK) {
 			Serial.println(F("Failed to add peer"));
 			return;
 	}
 
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
    
    ultimo_mensaje_terremoto = ahora;
    ultimo_mensaje_caida = ahora;
    ultimo_mensaje_carro = ahora;

// ENVIAR POR ESP-NOW A ROBER SENSOR  

// Set values to send
  if(a.acceleration.z > 12 AND (ahora - ultimo_mensaje_terremoto >= 30000)){


      String msg;
      StaticJsonDocument<64> root;
      root["RoverId"] = "Actuator";
      root["ErrCode"] = "1";
      root["Latitude"] = ;
      root["Longitude"] = ;
      root["Status"] = "Error" ;
      root["Timestamp"] =  ;
      serializeJson(root,msg);
      Serial.println(msg);


      char emergency_message[MAX_SIZE]
     	strcpy(emergency_message, msg.c_str(), MAX_SIZE);
      // Send message via ESP-NOW
 	    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &emergency_message, sizeof(emergency_message));

  }

  if(a.acceleration.z > (18) AND (ahora - ultimo_mensaje_caida >= 30000)){

      String msg;
      StaticJsonDocument<64> root;
      root["RoverId"] = "Actuator";
      root["ErrCode"] = "2";
      root["Latitude"] = ;
      root["Longitude"] = ;
      root["Status"] = "Error" ;
      root["Timestamp"] =  ;
      serializeJson(root,msg);
      Serial.println(msg);


      char emergency_message[MAX_SIZE]
     	strcpy(emergency_message, msg.c_str(), MAX_SIZE);
      // Send message via ESP-NOW
 	    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &emergency_message, sizeof(emergency_message));

  }
                                             //Diferenciar entre ultimo mensaje de cada emergencia
  if (CONDICIONES PARA QUE CARRO DE MUESTRAS LLENO AND (ahora - ultimo_mensaje_carro >= 30000)){

      String msg;
      StaticJsonDocument<64> root;
      root["RoverId"] = "Actuator";
      root["ErrCode"] = "3";
      root["Latitude"] = ;
      root["Longitude"] = ;
      root["Status"] = "Error" ;
      root["Timestamp"] =  ;
      serializeJson(root,msg);
      Serial.println(msg);


      char emergency_message[MAX_SIZE]
     	strcpy(emergency_message, msg.c_str(), MAX_SIZE);
      // Send message via ESP-NOW
 	    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &emergency_message, sizeof(emergency_message));

  }

 	delay(1000);

  }

}