/*
 * IoT DAC UMA 2023
 * 
 * Ejemplo de programa Arduino que usa un par de operaciones de esta API para enviar datos de un supuesto sensor de temperatura al servidor.
 * El programa usa la librería ESP8266HTTPClient incluida en el soporte Arduino de nuestra placa, como siempre tiene ejemplos de uso:
 * https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266HTTPClient/examples
 * y se puede consultar el interfaz de la librería en github:
 * https://links2004.github.io/Arduino/dd/d8d/class_h_t_t_p_client.html 
 * 
 * El programa primero en el setup() se hace uso de una petición GET para obtener la fecha y hora actual en forma de timestamp.
 * Después en el loop() se envía periodicamente datos al servidor mediante una petición POST. 
 * Los datos se formatean en JSON y contienen la fecha y hora actuales (campo time, calculado a partir del timestamp obtenido previamente y el lapso de tiempo transcurrido desde su obtención) 
 * y un valor de temperatura (campo temp). En este ejemplo el campo temperatura tiene un valor constante y no está realmente leyendo ningún sensor.
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <iostream>
#include <sstream>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include "DHTesp.h"

// definimos macro para indicar función y línea de código en los mensajes
#define DEBUG_STRING "["+String(__FUNCTION__)+"():"+String(__LINE__)+"] "

HTTPClient https;

#define __HTTPS__
#ifdef __HTTPS__
 /* #include <WiFiClientSecure.h>
  WiFiClientSecure ClienteWiFi;
  const String URL_BASE = "https://iot.ac.uma.es:1880";
  // huella digital SHA-1 del servidor iot.ac.uma.es (vencimiento enero 2024)
  const char* fingerprint = "DE:3C:76:79:45:D8:F0:13:9F:22:A5:42:97:0B:F6:56:4E:E6:B8:FD";
#else*/
  #include <WiFiClient.h>
  WiFiClient ClienteWiFi;
  PubSubClient mqtt_client(ClienteWiFi);
  const String URL_BASE = "http://172.16.52.52:1880";
#endif

// wifi config
const String ssid     = "infind";
const String password = "infind1518wifi";
// mqtt config
const String mqtt_server = "iot.ac.uma.es";
const String mqtt_user = "infind";
const String mqtt_pass = "zancudo";

// cadenas para topics e ID
String ID_PLACA;
String topic_PUB;
String topic_SUB;

// GPIOs
int LED1 = 2;
int LED2 = 16;

// hora actual
unsigned long long timestamp = 0; 
unsigned long sincronizacion = 0;

// sensor
DHTesp dht;

//-----------------------------------------------------------
//-----------------------------------------------------------
void conecta_wifi() {
  Serial.println(DEBUG_STRING+"Conectando a SSID: " + ssid);
 
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED1,!digitalRead(LED1));
    delay(200);
    Serial.print(".");
  }
  digitalWrite(LED1,HIGH);
  Serial.println();
  Serial.println(DEBUG_STRING+"WiFi conectada, dirección IP: " + WiFi.localIP().toString());
}

void conecta_mqtt() {
  // Loop until we're reconnected
  while (!mqtt_client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt_client.connect(ID_PLACA.c_str(), mqtt_user.c_str(), mqtt_pass.c_str())) {
      Serial.println(" conectado a broker: " + mqtt_server);
      mqtt_client.subscribe(topic_SUB.c_str());
    } else {
      Serial.println("ERROR:"+ String(mqtt_client.state()) +" reintento en 5s" );
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//-----------------------------------------------------------
// peticiones HTTP
//-----------------------------------------------------------
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
 // función de deserialización de las temperaturas mínima y máxima
 void deserialize(String* respuesta){
  StaticJsonDocument<128> root;
  deserializeJson(root, respuesta);
  *tempmin = root["tempmin"]; 
  *tempmax = root["tempmax"];
 }
//-----------------------------------------------------------
//  SETUP
//-----------------------------------------------------------
void setup() {
  String respuesta="";
  int codigoStatus;
  Serial.begin(115200);
  Serial.println();
  Serial.println(DEBUG_STRING+"Programa: "+String(__FILE__));
  Serial.println(DEBUG_STRING+"Comienza SETUP...");
  pinMode(LED1, OUTPUT);    // inicializa GPIO como salida
  pinMode(LED2, OUTPUT);    
  digitalWrite(LED1, LOW); 
  digitalWrite(LED2, HIGH); // apaga el led
 
  conecta_wifi();
  mqtt_client.setServer(mqtt_server.c_str(), 1883);
  mqtt_client.setBufferSize(512); // para poder enviar mensajes de hasta X bytes
  conecta_mqtt();

/*#ifdef __HTTPS__
  ClienteWiFi.setFingerprint(fingerprint); // se comprobará el certificado del servidor
  //sslClienteWiFi.setInsecure(); // si no se quiere comprobar el certificado del servidor
#endif*/



//-----------------------------------------------------------
//  LOOP
//-----------------------------------------------------------
void loop() {
  static unsigned long ultimo_post=0;
  static unsigned long ultima_lectura=0;
  unsigned long ahora = millis();
  String respuesta="";

  if (ahora-ultima_lectura >=5000){
    ultima_lectura=ahora; 
    temperatura = analogRead(A0); 
    http_GET(URL_BASE+"/temp_parameters/values", &respuesta);
    deserialize(&respuesta,&tempmin,&tempmax);
  }

  if (!mqtt_client.connected()) conecta_mqtt();
  
  mqtt_client.loop(); // esta llamada para que la librería recupere el control
  

  if(ahora-ultimo_post >= 30000)
  {
    ultimo_post=ahora;
    mqtt_client.publish(topic_PUB.c_str(), mensaje.c_str());

  }
}

