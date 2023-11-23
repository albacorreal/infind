//Libraries
#include <string>
#include <ArduinoJson.h> 
#include <WiFi.h>
#include <PubSubClient.h>
/* insert your sensor libraries here
#include <DHTesp.h>
*/
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
  ID_PLACA = "ESP_" + String( ESP.getChipId() );
   // Topics
  conexion = "infind/GRUPO15/conexion";
  datos_uv = "infind/GRUPO15/uv";
  data = "infind/GRUPO15/led/cmd";

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
 
}

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
    
  if (ahora - ultimo_mensaje >= 30000) {
    ultimo_mensaje = ahora;

    // Lectura del sensor  
    dato_uv = serialRead(A0);
    
    //Mensaje formateado en JSON con los datos de los sensores y de conexión Wifi 
    String mensaje = "{\"RadiacionDHT11\":"+ String(dato_uv) +"}";
   
    //Imprimimos el mensaje en el monitor serie
    Serial.println();
    Serial.println("Topic   : "+ datos_uv);
    Serial.println("Payload : "+ mensaje);
   
    //Publicamos el mensaje
    mqtt_client.publish(datos.c_str(), mensaje.c_str());
  
}
