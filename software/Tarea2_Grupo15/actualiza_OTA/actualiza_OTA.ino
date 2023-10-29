/*
 * 
 *  Ejemplo actualización OTA
 *  IoT DAC UMA
 *  
 *  Válido para ESP8266 y ESP32
 * 
 *  Para usar con servidor seguro iot.ac.uma.es definir macro: __HTTPS__
 * 
*/

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

const String ssid     = "infind";
const String password = "1518wifi";

// GPIOs
int LED_blink = 16;  
int LED_OTA   = 16; 
int LED_wifi  = 2;

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
//-----------------------------------------------------
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
  Serial.println();
  Serial.println(DEBUG_STRING+"WiFi connected, IP address: " + WiFi.localIP().toString());
}
//-----------------------------------------------------------
//  SETUP
//-----------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println(DEBUG_STRING+"Fuente del programa: "+String(__FILE__));
  Serial.println(DEBUG_STRING+"Nombre Firmware: "+String(HTTP_OTA_VERSION)+".bin");
  Serial.println(DEBUG_STRING+"Placa: "+String(ARDUINO_BOARD));
  Serial.println(DEBUG_STRING+"Comienza SETUP...");
  pinMode(LED_blink, OUTPUT);       
  pinMode(LED_wifi, OUTPUT);    
  digitalWrite(LED_blink, HIGH);  // LED off
  digitalWrite(LED_wifi , LOW);   // LED on
  conecta_wifi();
  intenta_OTA();
  Serial.println(DEBUG_STRING+"fin SETUP\n");
}
//-----------------------------------------------------------
//  LOOP
//-----------------------------------------------------------
void loop() 
{
  static unsigned long ultimo = 0;
  unsigned long ahora = millis();
	if (ahora-ultimo >= 500)
	{
		ultimo = ahora;
		digitalWrite(LED_blink, !digitalRead(LED_blink));   // Toggle the LED 
	}
}
