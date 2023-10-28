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
int LED1 = 2;  
int LED2 = 16;

// Variables
struct registro_datos 
{
  float temperatura;
  float humedad;
  unsigned long uptime;
  double Vcc;
};


//-----------------------------------------------------
//Establece la conexión WIfi
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

//--------------------------------------------------------------------------------------------------------
// void serializa_sprintf_datos(struct registro_datos datos, char *cadena, int size)
// {
//   snprintf(cadena,size,"{\"Uptime\":%d,\"Vcc\":%f,\"DHT11\":{\"temp\":%g,\"hum\":%g},\"Wifi\":{\"SSId\":%s,\"IP\":%s,\"RSSI\":%d,}}",
//                         datos.uptime, datos.Vcc, datos.temperatura, datos.humedad, ssid, IP, RSSI);
// }
//------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//                SETUP
//------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Empieza setup...");

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
  Serial.println("Termina setup en " +  String(millis()) + " ms");

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

