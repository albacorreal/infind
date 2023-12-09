/*
MINIMOS ESP8266
*/
#include <string>
#include <iostream>
#include <sstream>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266HTTPClient.h>
#include <DHTesp.h>
#include <ArduinoJson.h>

WiFiClient wClient;
PubSubClient mqtt_client(wClient);


// definimos macro para indicar función y línea de código en los mensajes
#define DEBUG_STRING "["+String(__FUNCTION__)+"():"+String(__LINE__)+"] "

// API
HTTPClient https;

#define __HTTPS__
#ifdef __HTTPS__
  #include <WiFiClientSecure.h>
  WiFiClientSecure ClienteWiFi;
  const String URL_BASE = "https://iot.ac.uma.es:1880";
  // huella digital SHA-1 del servidor iot.ac.uma.es (vencimiento enero 2024)
  const char* fingerprint = "DE:3C:76:79:45:D8:F0:13:9F:22:A5:42:97:0B:F6:56:4E:E6:B8:FD";
#else
  #include <WiFiClient.h>
  WiFiClient ClienteWiFi;
  const String URL_BASE = "http://iot.ac.uma.es:1880";
#endif



// Update these with values suitable for your network.
const String ssid = "infind";
const String password = "zancudo1518wifi";

const String mqtt_server = "iot.ac.uma.es";
const String mqtt_user = "infind";
const String mqtt_pass = "zancudo";

// cadenas para topics e ID
String ID_PLACA;
String topic_PUB;
String topic_SUB;
String ;

// GPIOs
int LED1 = 2;  
int LED2 = 16; 

// Sensor
DHTesp dht;

//-----------------------------------------------------
void conecta_wifi() {
  Serial.println("Connecting to " + ssid);
 
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }
  IP =  WiFi.localIP().toString();
  Serial.println();
  Serial.println("WiFi connected, IP address: " + IP);
}

//-----------------------------------------------------
void conecta_mqtt() {
  // Loop until we're reconnected
  while (!mqtt_client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt_client.connect(ID_PLACA.c_str(), mqtt_user.c_str(), mqtt_pass.c_str())) {
      Serial.println(" conectado a broker: " + mqtt_server);
      //mqtt_client.subscribe(topic_SUB.c_str());
    } else {
      Serial.println("ERROR:"+ String(mqtt_client.state()) +" reintento en 5s" );
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//-----------------------------------------------------
void procesa_mensaje(char* topic, byte* payload, unsigned int length) { 
  String mensaje=String(std::string((char*) payload,length).c_str());
  Serial.println("Mensaje recibido ["+ String(topic) +"] "+ mensaje);
  // compruebo el topic
  if(String(topic)==topic_SUB) 
  {
    if (mensaje[0] == '1') {
        digitalWrite(LED1, LOW);   // Turn the LED on (Note that LOW is the voltage level 
      } else {
        digitalWrite(LED1, HIGH);  // Turn the LED off by making the voltage HIGH
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

// string para guardar el token
String tokenAPI;

//-----------------------------------------------------
//     SETUP
//-----------------------------------------------------
void setup() {
  String respuesta="";
  int codigoStatus;

  Serial.begin(115200);
  Serial.println();
  Serial.println(DEBUG_STRING+"Programa: "+String(__FILE__));
  Serial.println(DEBUG_STRING+"Comienza SETUP...");
  
  pinMode(LED1, OUTPUT);    // inicializa GPIO como salida
  pinMode(LED2, OUTPUT);    
  digitalWrite(LED1, HIGH); // apaga el led
  digitalWrite(LED2, HIGH); 

  // crea topics usando id de la placa
  ID_PLACA = String(ESP.getChipId());
  topic_PUB = "examen/ejercicio2/ESP" + ID_PLACA;
  //topic_SUB = "";

  conecta_wifi();

  mqtt_client.setServer(mqtt_server.c_str(), 1883);
  mqtt_client.setBufferSize(512); // para poder enviar mensajes de hasta X bytes
  mqtt_client.setCallback(procesa_mensaje);
  conecta_mqtt();

  #ifdef __HTTPS__
  ClienteWiFi.setFingerprint(fingerprint); // se comprobará el certificado del servidor
  //sslClienteWiFi.setInsecure(); // si no se quiere comprobar el certificado del servidor
  #endif

  codigoStatus = http_GET(URL_BASE + "/my_token?id=0598077", &respuesta);
  StaticJsonDocument<512>root;
  deserializeJson(root, respuesta);
  tokenAPI = String(root["token"]);
  Serial.println("Token obteido de la API: " + tokenAPI);


  Serial.println("Identificador placa: "+ ID_PLACA);
  Serial.println("Topic publicacion  : "+ topic_PUB);
  Serial.println("Topic subscripción  : "+ topic_SUB);
  Serial.println(DEBUG_STRING + "Termina setup en " +  String(millis()) + " ms");

  dht.setup(5, DHTesp::DHT11); // Connect DHT sensor to GPIO 5
}

//-----------------------------------------------------

unsigned long ultimo_mensaje = 0;
//-----------------------------------------------------
//     LOOP
//-----------------------------------------------------
void loop() {
  if (!mqtt_client.connected()) conecta_mqtt();
  
  mqtt_client.loop(); // esta llamada para que la librería recupere el control
  
  unsigned long ahora = millis();
  
  if (ahora - ultimo_mensaje >= 15000) {
    ultimo_mensaje = ahora;
    

    String mensaje;
    StaticJsonDocument<1024>doc;
    doc["id"] = "0598077";
    doc["ip"] = IP;
    doc["tokenAPI"] = tokenAPI;
    doc["sensor"]["temperatura"] = dht.getTemperature();
    doc["sensor"]["humedad"] = dht.getHumidity();

    serializeJson(doc,mensaje);

    mqtt_client.publish(topic_PUB.c_str(), mensaje.c_str());
    Serial.println("Mensaje enviado: " + mensaje);
  
  }
}
