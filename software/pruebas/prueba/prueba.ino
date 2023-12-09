// (1) LIBRARIES
#include <string>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <iostream>
#include <sstream>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "DHTesp.h"

// (2) DECLARE CLIENTS
  // MQTT
WiFiClient wClient; // wifi client
PubSubClient mqtt_client(wClient); //mqtt client
  // HTTP
#define DEBUG_STRING "["+String(__FUNCTION__)+"():"+String(__LINE__)+"] "
HTTPClient https; //client declaration
#define __HTTPS__
#ifdef __HTTPS__
  #include <WiFiClientSecure.h>
  WiFiClientSecure ClienteWiFi;
  const String URL_BASE = "https://iot.ac.uma.es:1880/my_token?id=0598077"; 
  // huella digital SHA-1 del servidor iot.ac.uma.es (vencimiento enero 2024)
  const char* fingerprint = "DE:3C:76:79:45:D8:F0:13:9F:22:A5:42:97:0B:F6:56:4E:E6:B8:FD";
#else
  #include <WiFiClient.h>
  WiFiClient ClienteWiFi;
  const String URL_BASE = "https://iot.ac.uma.es:1880/my_token?id=0598077";
#endif

// (3) DEFINE NETWORK CREDENTIALS
// WiFi
const String ssid = "MOVISTAR_F730";
const String password = "HxeFuGxr9cUEJwbzDt5a";
// mqtt
const String mqtt_server = "iot.ac.uma.es";
const String mqtt_user = "infind";
const String mqtt_pass = "zancudo";

// (4) CLIENT ID & TOPIC DECLARATION 
String ID_BOARD;
String topic_PUB;
String topic_SUB;
unsigned long long timestamp = 0; 
unsigned long sincronizacion = 0;
// (5) SENSOR CLASSES
DHTesp dht; // sensor class

// (6) VARIABLES & GPIOS
float sensor_var;

// (7) CONNECTION FUNCTIONS
  // WiFi
void conecta_wifi(){
  Serial.println("Connecting to" + ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid,password);
  while (WiFi.status() != WL_CONNECTED){
    delay (200);
    Serial.println("."); 
  }
  Serial.println();
  Serial.println(DEBUG_STRING+"WiFi connected, IP address: "+ WiFi.localIP().toString());

}
  //MQTT
void conecta_mqtt() {
  while (!mqtt_client.connected()){
    Serial.print("Attempting MQTT Connection...");
    if (mqtt_client.connect(ID_BOARD.c_str(), mqtt_user.c_str(), mqtt_pass.c_str())){
      Serial.println("connected to broker " + mqtt_server);
      mqtt_client.subscribe(topic_SUB.c_str());
    } else {
      Serial.println("ERROR: " + String(mqtt_client.state()) + "reintento en 5s...");
      delay (500);
    }
  }
}
 //HTTP
int http_GET(String URL, String* respuesta)
{
  return peticion_HTTP("GET", URL, "", respuesta);
}
//-----------------------------------------------------------
int http_POST(String URL, String body, String* respuesta)
{
  return peticion_HTTP("POST", URL, body, respuesta);
}
//-----------------------------------------------------------
int peticion_HTTP (String metodo, String URL, String body, String* respuesta)
{
  int httpCode=-1;
  unsigned long start = millis();
  if (https.begin(ClienteWiFi, URL)) {  // HTTPS

      Serial.println(DEBUG_STRING + metodo +" petición... " + URL);
      // start connection and send HTTP header
      https.addHeader("Content-Type", "application/json");
      if(metodo=="GET" )  httpCode = https.GET();
      if(metodo=="POST"){ httpCode = https.POST(body);
        Serial.println(DEBUG_STRING +"cuerpo solicitud: \n     "+ body);
      }
      // httpCode will be negative on error
      if (httpCode > 0) {
        Serial.println(DEBUG_STRING + metodo +" respuesta... código Status: "+ String(httpCode));

        // queremos la respuesta del servidor
        if (respuesta!=NULL) {
          *respuesta = https.getString();
          Serial.println(DEBUG_STRING+"cuerpo respuesta: \n     "+ *respuesta);
        }
      } else {
        Serial.println(DEBUG_STRING+ metodo +"... falló, error: "+ String(https.errorToString(httpCode).c_str()) );
      }

      https.end();
    } else {
      Serial.println(DEBUG_STRING+"No se pudo conectar");
    }
    Serial.println(DEBUG_STRING+"tiempo de respuesta: "+ String(millis()-start) +" ms\n");
    return httpCode;
}

float presion;
void callback(char* topic, byte* payload, unsigned int length){
  String message = String(std::string((char*) payload, length).c_str()); //create the message recieved string
  Serial.println("Mensaje recibido ["+ String(topic)+"] "+ message);
  // Uncomment if subscribed to a topic
  /*if(String(topic)==topic_SUB) {                                         // check the topic analysed
    // Deserialize
    StaticJsonDocument<512> root;       
  deserializeJson(root, message.c_str());
  if(root.containsKey("BME280"))  // comprobar si existe el campo/clave que estamos buscando
    { 
     presion = float(root["BME280"]["Presion"]);
    }
    else
    {
      Serial.print("Error : ");
      Serial.println("campo \"BME280\"Presion\" no encontrado");
    }
  }*/
}

String valor_token;
// (8) SETUP
void setup(){
String respuesta="";
  int codigoStatus;
  Serial.begin(115200);
  Serial.println();
  Serial.println(DEBUG_STRING+"Programa: "+String(__FILE__));
  Serial.println(DEBUG_STRING+"Comienza SETUP...");
  dht.setup(5, DHTesp::DHT11);
  //pinMode (GPIO, INPUT);  //GPIO definition;
// Board & topic names
  ID_BOARD = "ESP" + String( ESP.getChipId() );
  topic_PUB = "infind/ESP_1088397/alba/temperatura";
  //topic_SUB = "iot/datos/json";
// initialize connections
  conecta_wifi();
  mqtt_client.setServer(mqtt_server.c_str(),1883);
  mqtt_client.setBufferSize(512);
  mqtt_client.setCallback(callback);
  conecta_mqtt();

  #ifdef __HTTPS__
  ClienteWiFi.setFingerprint(fingerprint); // se comprobará el certificado del servidor
  //sslClienteWiFi.setInsecure(); // si no se quiere comprobar el certificado del servidor
#endif

  sincronizacion = millis();
  codigoStatus = http_GET(URL_BASE+"/timestamp", &respuesta);
  if(codigoStatus==HTTP_CODE_OK) 
  {
    std::stringstream ss(respuesta.c_str());
    ss >> timestamp;   // leemos el timestamp que viene en el String de respuesta
  }
  Serial.println(DEBUG_STRING+"fin SETUP\n");
// initialize sensors
// Print info
  Serial.println("Board Identifier: "+ ID_BOARD);
  http_GET(URL_BASE, &respuesta);
  // DESERIALIZE JSON
   StaticJsonDocument<512> root;
  deserializeJson(root, respuesta.c_str());
  if(root.containsKey("token"))  // comprobar si existe el campo/clave que estamos buscando
    { 
     valor_token = String(root["token"]);
    }
    else
    {
      Serial.print("Error : ");
      Serial.println("campo \"token\" no encontrado");
    }
}
unsigned long last_msg = 0; 
// (9) LOOP
void loop(){
  String respuesta="";
  int codigoStatus;
  if (!mqtt_client.connected()) conecta_mqtt(); // verify connection
  mqtt_client.loop();                           // gives authority for callback.
  unsigned long now = millis();
  if (now - last_msg >= 18000){                 // defines the publishing rate
    last_msg = now;
    float temp = dht.getTemperature();
    float hum = dht.getHumidity();

    String message = "{\"temperatura\": "+ String(temp) +", \"humedad\": "+ String(hum) +"}";
    Serial.println(message);
    Serial.println(presion);
    // Serialize 
    String httpMessage;
    StaticJsonDocument<512> root;
    root["id"] = String("0598077");
    root["ip"] = WiFi.localIP().toString();
    root["tokenAPI"] = String(valor_token);
    serializeJson(root,httpMessage);
    mqtt_client.publish(topic_PUB.c_str(),message.c_str());
    http_POST("https://iot.ac.uma.es:1880/sensor_data",httpMessage,&respuesta);
  }
}