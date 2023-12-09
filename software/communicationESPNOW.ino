// Código para una ESP actuando de pasarela/cliente


#ifdef ESP8266
  #include <ESP8266WiFi.h>
  extern "C" {  
    #include <espnow.h> 
  } 
#endif
#ifdef ESP32
  #include <esp_now.h>
  #include <WiFi.h>
#endif
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <queue> 


// definimos macro para indicar función y línea de código en los mensajes
#define DEBUG_STRING "["+String(__FUNCTION__)+"():"+String(__LINE__)+"] "

// this is the MAC Address of the remote ESP server which receives these sensor readings
uint8_t pasarelaMAC[] = {0x3E, 0x33, 0x33, 0x33, 0x33, 0x33};


// cadenas para topics e ID
String ID_PLACA;


// clase para manejar un mensaje esp-now
class Tmensaje
{
  public:
  uint8_t mac[6];
  uint8_t data[250];
  uint8_t len;
  Tmensaje ( uint8_t *_mac, uint8_t *_data, uint8_t _len)
   { 
    memcpy(mac,_mac,6);
    memcpy(data,_data,_len);
    len=_len;
   }
};

// cola de mensajes esp-now recibidos
std::queue<Tmensaje> cola_mensajes;

/* Set a private Mac Address
 *  http://serverfault.com/questions/40712/what-range-of-mac-addresses-can-i-safely-use-for-my-virtual-machines
 * Note: the point of setting a specific MAC is so you can replace this Gateway ESP8266 device with a new one
 * and the new gateway will still pick up the remote sensors which are still sending to the old MAC 
 */
uint8_t mac[] = {0x3E, 0x33, 0x33, 0x33, 0x33, 0x33};


#define WIFI_CHANNEL 6
//#define SLEEP_SECS 15 * 60 // 15 minutes
#define SLEEP_SECS 30  // segundos
#define SEND_TIMEOUT 2000  // 2 seconds timeout 

// keep in sync with slave struct
#define MENSAJE_MAXSIZE 250
char mensaje[MENSAJE_MAXSIZE];

unsigned long bootMs=0, setupMs=0, sendMs=0, waitMs=0;

RF_PRE_INIT() {
  bootMs = millis();
}


// GPIOs
int LED1 = 2;  
int LED2 = 16;


//-----------------------------------------------------
// devuelve 2 caracteres HEX para un byte
inline String byte2HEX (byte data)
{
  return (String(data, HEX).length()==1)? String("0")+String(data, HEX) : String(data, HEX);
}


//-----------------------------------------------------
//
void procesa_mensaje_espnow(uint8_t *mac, uint8_t *data, uint8_t len) {
  Serial.println(DEBUG_STRING+"Mensaje ESPNOW recibido!");
  digitalWrite(LED2, LOW); // led indica que llegó un mensaje
  // encola el mensaje para su envío en loop()
  cola_mensajes.push(Tmensaje(mac,data,len));
}


//-----------------------------------------------------
// callback fin envío esp-now
void fin_envio_espnow (uint8_t* mac, uint8_t sendStatus) {
  Serial.printf("Funcion callback de fin de envio\n");
  String deviceMac = "";
  for (int i=0; i<6; i++) deviceMac += byte2HEX(mac[i]);
  for (auto & c: deviceMac) c = toupper(c);
  Serial.printf("Mensaje enviado a MAC: %s\n",deviceMac.c_str());
  Serial.printf("Recepcion: %s\n", (sendStatus)? "ERROR" : "OK");
  
  gotoSleep();
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();
  Serial.println(DEBUG_STRING+"Programa: "+String(__FILE__));
  Serial.println(DEBUG_STRING+"Comienza SETUP..."); pinMode(LED1, OUTPUT);    // inicializa GPIO como salida
  digitalWrite(LED1, HIGH); // apaga el led
  pinMode(LED2, OUTPUT);    // inicializa GPIO como salida
  digitalWrite(LED2, HIGH); // apaga el led

  // crea id de la placa para cliente MQTT
  ID_PLACA= "ESP_"+String( ESP.getChipId() );

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.print("ESP8266 Board MAC Address:  ");
  Serial.println(WiFi.macAddress());


  if (esp_now_init()!=0) {
    Serial.println("*** ESP_Now init failed");
    ESP.restart();
  }

  //codigo pasarela
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_recv_cb(procesa_mensaje_espnow); 

  // codigo cliente
  //esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_add_peer(pasarelaMAC, ESP_NOW_ROLE_COMBO, WIFI_CHANNEL, NULL, 0);
  esp_now_register_send_cb(fin_envio_espnow);




  esp_now_send(mac,data, size data)


  Serial.println("Identificador placa: "+ ID_PLACA );
  Serial.println("Publicando en topic infind/espnow/#");
  Serial.print(DEBUG_STRING+"Mi dirección MAC: "); Serial.println(WiFi.softAPmacAddress());
  Serial.println(DEBUG_STRING+"Esperando mensajes ESP-NOW en esta MAC...\n");
}

void loop() {
  // put your main code here, to run repeatedly:
  while (!cola_mensajes.empty())
  {
    Tmensaje mensaje = cola_mensajes.front();
    cola_mensajes.pop();
    Serial.println(DEBUG_STRING+"Procesando mensaje ESPNOW...");
    conecta_wifi_mqtt();
    send_message(mensaje);
    digitalWrite(LED2, HIGH);
    mqtt_client.disconnect();
    Serial.println(DEBUG_STRING+"Reiniciamos placa -> modo esp-now...");
    delay(200);
    ESP.restart(); // <----- Reboots to re-enable ESP-NOW
  }








}



//-----------------------------------------------------
void gotoSleep() {
  // add some randomness to avoid collisions with multiple devices
  int sleepSecs = SLEEP_SECS + ((uint8_t)RANDOM_REG32/8); 
  Serial.printf("Boot: %i ms, setup: %ims, wait4DHT: %ims, send: %ims, now %ims, going to sleep for %isecs...\n", bootMs, setupMs, waitMs, sendMs, millis(), sleepSecs); 
  ESP.deepSleepInstant(sleepSecs * 1000000, RF_NO_CAL);
}





