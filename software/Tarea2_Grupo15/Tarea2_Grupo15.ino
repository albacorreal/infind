/*
  TAREA DE GRUPO ESP8266
  -------------------------------------------------
  GRUPO 15
  --------------------------------------------------
  Componentes del grupo:
    - Jorge Ávila Orero
    - Laura Castro Lara
    - Alba Correal Olmo
    - Victor Gabriel Mengual Pirpamer
  
  ---------------------------------------------------

  Funciones:
    - Lectura de un sensor de humedad y temperatura DHT11
    - Lectura del nivel de voltaje
    - Control de la intensidad de encendido de un led

*/
//Libraries
#include <string>
#include <ArduinoJson.h> 
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHTesp.h>

//#define __HTTPS__
#ifdef ESP32
 #include <WiFi.h>
 #include <HTTPUpdate.h>
 #include <HTTPClient.h>
#endif

#ifdef ESP8266
 #include <ESP8266WiFi.h>
 #include <ESP8266httpUpdate.h>
#endif

// URL para actualización   >>>> SUSTITUIR IP <<<<<
#ifdef __HTTPS__
 #define OTA_URL "https://iot.ac.uma.es:1880/esp8266-ota/update"// Address of OTA update server
#else
 #define OTA_URL  "http://172.16.53.###:1880/esp8266-ota/update"// Address of OTA update server
#endif

// Para versiones del IDE < 2.0 poner en FW_BOARD_NAME el identificador de la placa, por ejemplo ".nodemcu",
//      ver nombre del binario generado para conocer identificador utilizado
// Para versiones del IDE >= 2.0 dejar vacío
#define FW_BOARD_NAME ""

// Nombre del firmware
#define HTTP_OTA_VERSION      String(__FILE__).substring(String(__FILE__).lastIndexOf('\\')+1) + FW_BOARD_NAME 

// definimos macro para indicar función y línea de código en los mensajes
#define DEBUG_STRING "["+String(__FUNCTION__)+"():"+String(__LINE__)+"] "


//Add this to use ESP.getVcc
ADC_MODE(ADC_VCC);

//Mqtt Client
WiFiClient wClient;
PubSubClient mqtt_client(wClient);

//Dht sensor
DHTesp dht;

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
String conexion;
String datos;
String led_cmd;
String led_status;

// GPIOs
// LEDs tarea inicial
int LED1 = 2;
int LED2 = 16;

/* LEDs modificación OTA update
No se han unificado los nombres porque se aplican con distinta funcionalidad
*/
int LED_blink = 16;  
int LED_OTA   = 16; 
int LED_wifi  = 2;

// Variables
struct registro_datos 
{
  float temperatura;
  float humedad;
  unsigned long uptime;
  double Vcc;
};


//-----------------------------------------------------
// funciones para progreso de OTA
//-----------------------------------------------------
void inicio_OTA(){ Serial.println(DEBUG_STRING+"Nuevo Firmware encontrado. Actualizando..."); }
void final_OTA() { Serial.println(DEBUG_STRING+"Fin OTA. Reiniciando..."); }
void error_OTA(int e){ Serial.println(DEBUG_STRING+"ERROR OTA: "+String(e)+" "+ESPhttpUpdate.getLastErrorString()); }
void progreso_OTA(int x, int todo)
{
  int progreso=(int)((x*100)/todo);
  String espacio = (progreso<10)? "  ":(progreso<100)? " " : "";
  if(progreso%10==0) Serial.println(DEBUG_STRING+"Progreso: "+espacio+String(progreso)+"% - "+String(x/1024)+"K de "+String(todo/1024)+"K");
}
//-----------------------------------------------------
void intenta_OTA()
{ 
  Serial.println( "---------------------------------------------" );  
  Serial.println( DEBUG_STRING+"Comprobando actualización:" );
  Serial.println( DEBUG_STRING+"URL: "+OTA_URL );
  Serial.println( "---------------------------------------------" );  
  ESPhttpUpdate.setLedPin(LED_OTA, LOW);
  ESPhttpUpdate.onStart(inicio_OTA);
  ESPhttpUpdate.onError(error_OTA);
  ESPhttpUpdate.onProgress(progreso_OTA);
  ESPhttpUpdate.onEnd(final_OTA);

#ifdef __HTTPS__
  WiFiClientSecure wClient;
  // La lectura sobre SSL puede ser lenta, poner timeout suficiente
  wClient.setTimeout(12); // timeout en segundos
  wClient.setInsecure();  // no comprobar el certificado del servidor
#else
  WiFiClient wClient;
#endif

  switch(ESPhttpUpdate.update(wClient, OTA_URL, HTTP_OTA_VERSION)) {
    case HTTP_UPDATE_FAILED:
      Serial.println(DEBUG_STRING+"HTTP update failed: Error ("+String(ESPhttpUpdate.getLastError())+"): "+ESPhttpUpdate.getLastErrorString());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println(DEBUG_STRING+"El dispositivo ya está actualizado");
      break;
    case HTTP_UPDATE_OK:
      Serial.println(DEBUG_STRING+"OK");
      break;
    }
}


//-----------------------------------------------------
//Establece la conexión WIfi
void conecta_wifi() {
  Serial.println(DEBUG_STRING+"Connecting to " + ssid);
 
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_wifi,!digitalRead(LED_wifi));
    delay(200);
    Serial.print(".");
  }
  digitalWrite(LED_wifi,HIGH);

  // Get local IP
  IP = WiFi.localIP().toString();

  // Get RSSI
  RSSI = WiFi.RSSI();

  Serial.println();
  Serial.println(DEBUG_STRING+"WiFi connected, ip address: " + IP);
}



//-----------------------------------------------------
//Establece la conexión entre cliente y broker
void conecta_mqtt() {
  // Loop until we're reconnected
  while (!mqtt_client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    //                                                                                WillTopic    , WillQoS, WillRetain, WillMessage
    if (mqtt_client.connect(ID_PLACA.c_str(), mqtt_user.c_str(), mqtt_pass.c_str(),conexion.c_str(),       0,    true   ,"{\"online\":false}")) {
      Serial.println(" conectado a broker: " + mqtt_server);
      mqtt_client.subscribe(led_cmd.c_str());
    } else {
      Serial.println("ERROR:"+ String(mqtt_client.state()) +" reintento en 5s" );
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


//------------------------CALLBACK-----------------------------
//Comrpueba el topic y manda el valor de intensidad pedido al led
//Publica el valor inetnsidad del led (0 a 100) en el topic correspondiente
void callback(char* topic, byte* payload, unsigned int length) {

  char *mensaje=(char *)malloc(length+1); // reservo memoria para copia del mensaje
  strncpy(mensaje,(char*)payload,length); // copio el mensaje en cadena de caracteres
  mensaje[length]='\0';
  
  Serial.printf("Message arrived [%s]: %s\n",topic, mensaje);
  
  // compruebo que es el topic adecuado
  if(strcmp(topic,"infind/GRUPO15/led/cmd")==0)
  {
    StaticJsonDocument<512> root; // el tamaño tiene que ser adecuado para el mensaje

    // Deserialize the JSON document
    DeserializationError error = deserializeJson(root, mensaje,length);

    // Compruebo si no hubo error
    if (error) {
      Serial.print("Error deserializeJson() failed: ");
      Serial.println(error.c_str());
    }
    else
    if(root.containsKey("level"))  // comprobar si existe el campo/clave que estamos buscando
    { 
      int valor = root["level"];
      Serial.print("Mensaje OK, level = ");
      Serial.println(valor);

      analogWriteRange(100); // cambiamos el rango de 255 a 100
      analogWrite(LED1,100-valor); // mandamos el valor al LED

      //Mensaje formateado en JSON con valor de intensidad del led
      String mensaje="{\"led\": "+ String(valor) +" }";

      //Imprimimos el mensaje y topic por monitor serie
      Serial.println();
      Serial.println("Topic   : "+ led_status);
      Serial.println("Payload : "+ mensaje);

      //Publicamos el mensaje
      mqtt_client.publish(led_status.c_str(), mensaje.c_str());

    }
    else
    {
      Serial.print("Error : ");
      Serial.println("\"level\" key not found in JSON");
    }
  } // if topic
  else
  {
    Serial.println("Error: Topic desconocido");
  }

  free(mensaje); // libero memoria

}



//-----------------------------------------------------------------------------
//                SETUP
//------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Empieza setup...");

  // setup OTA
  Serial.println(DEBUG_STRING+"Fuente del programa: "+String(__FILE__));
  Serial.println(DEBUG_STRING+"Nombre Firmware: "+String(HTTP_OTA_VERSION)+".bin");
  Serial.println(DEBUG_STRING+"Placa: "+String(ARDUINO_BOARD));
  Serial.println(DEBUG_STRING+"Comienza SETUP...");

  pinMode(LED1, OUTPUT);      // inicializa GPIO como salida
  pinMode(LED2, OUTPUT);   

  digitalWrite(LED1, HIGH);   // apaga el led
  digitalWrite(LED2, HIGH); 

  // ID de la placa
  ID_PLACA = "ESP_" + String( ESP.getChipId() );

  // Topics
  conexion = "infind/GRUPO15/conexion";
  datos = "infind/GRUPO15/datos";
  led_cmd = "infind/GRUPO15/led/cmd";
  led_status = "infind/GRUPO15/led/status";

  conecta_wifi();   //establecemos conexión wifi
  intenta_OTA();    //intenta la actualización OTA

  //Configuraciones del cliente mqtt
  mqtt_client.setServer(mqtt_server.c_str(), 1883);
  mqtt_client.setBufferSize(512); // para poder enviar mensajes de hasta X bytes
  mqtt_client.setCallback(callback);  //para procesar los mensajes cada vez que llegue uno al topic que estamos suscritos.

  conecta_mqtt(); //establecemos la conexión del cliente con el servidor
 
 //Mensaje formateado en JSON con el estado de conexión a true
  String mensaje="{\"online\": ""true"" }";
  
  //Imprimimos mensaje y topic en el monitor serie
  Serial.println();
  Serial.println("Topic   : "+ conexion);
  Serial.println("Payload : "+ mensaje);
 
 //Publicamos el mensaje
  mqtt_client.publish(conexion.c_str(), mensaje.c_str(),true);

  //Mensajes de finalización de setup 
  Serial.println("Identificador placa: "+ ID_PLACA);
  Serial.println(DEBUG_STRING+"Termina setup en " +  String(millis()) + " ms");
  
  dht.setup(5, DHTesp::DHT11); // Connect DHT sensor to GPIO 5

}

//Inicializamos tiempo desde ultimo mensaje a 0
unsigned long ultimo_mensaje=0;

//-----------------------------------------------------
//     LOOP
//-----------------------------------------------------
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
  
  registro_datos dato;

  unsigned long ahora = millis();

  // LED toggle for OTA update every 0.5s
  if (ahora-ultimo >= 500)
	{
		ultimo = ahora;
		digitalWrite(LED_blink, !digitalRead(LED_blink));   // Toggle the LED 
	}

  // Data update every 30s
  if (ahora - ultimo_mensaje >= 30000) {
    ultimo_mensaje = ahora;

    // Lectura del sensor  
    dato.humedad = dht.getHumidity();
    dato.uptime = millis();
    dato.Vcc = ESP.getVcc()/1000;
    dato.temperatura = dht.getTemperature();
    
    //Mensaje formateado en JSON con los datos de los sensores y de conexión Wifi 
    String mensaje = "{\"Uptime\":"+ String(dato.uptime) +",\"Vcc\":"+ String(dato.Vcc) +",\"DHT11\":{\"temp\":"+ String(dato.temperatura) +",\"hum\":"+ String(dato.humedad) +"},\"Wifi\":{\"SSId\":\""+ ssid +"\",\"IP\": \"" + IP + "\",\"RSSI\":"+ String(RSSI) +"}}";
   
    //Imprimimos el mensaje en el monitor serie
    Serial.println();
    Serial.println("Topic   : "+ datos);
    Serial.println("Payload : "+ mensaje);
   
    //Publicamos el mensaje
    mqtt_client.publish(datos.c_str(), mensaje.c_str());
    
    digitalWrite(LED2, LOW); // enciende el led al enviar mensaje
  }
  if (digitalRead(LED2)==LOW && ahora-ultimo_mensaje>=100) 
    digitalWrite(LED2, HIGH); 

}

