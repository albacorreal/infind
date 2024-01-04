//================================================================================
// PROGRAM INFO
  /* CENTRAL-ROVER GATEWAY
  * It's purpose is to receive via serial communication Central commands and via
  * LoRa rover's messages. Then, the commands received from Central, send them via
  * LoRa to the rovers, and the messages received from the rovers, send them via
  * serial communication to the Central, therefore, acting as a gateway.
  */


//================================================================================

  #include <string>
  #include <iostream>
  #include <sstream>
  #include <ArduinoJson.h>
  #include <LoRa.h>


  #define NSS_PIN 15
  #define RST_PIN 32
  #define DIO_PIN 36

  #define FREQUENCY 433E6
  #define LOCAL_ADDR 0x12
  #define SENSOR_ADDR 0x13
  #define ACTUATOR_ADDR 0x14


  bool TASK_RECEIVED = false;

  byte MSG_COUNT = 0;



//================================================================================

  /* Function to be called when LoRa messages are received
   * \param packetSize */
  void lora_recv_callback(int packetSize);

  /* Function to connect LoRa
   * @brief initialize LoRa (frequency and pins) */
  void init_lora(void);

  /* Function to send message via LoRa
   * \param msg */
  void send_msg_lora(String msg);



//================================================================================

  void setup() {

      Serial.begin(115200);
      init_lora();

  }



  void loop() {

      // Read msg from serial port
      String msg = "";
      while (Serial.available()>0) {
        msg += (char)Serial.read();
      }

      send_msg_lora(msg);
      
      lora_recv_callback(LoRa.parsePacket());

  }



//================================================================================

  void lora_recv_callback(int packetSize) {

      if (packetSize == 0) return;
      //Serial.println("Message received. Packet info:");

      int recipient = LoRa.read();
      byte sender = LoRa.read();
      byte msg_id = LoRa.read();
      byte msg_length = LoRa.read();

      String msg = "";

      while (LoRa.available()) {
        msg += (char)LoRa.read();
      }
      
      if (msg_length != msg.length()) {
        //Serial.println("ERROR: message length does not match length");
        return;

      } if (recipient != LOCAL_ADDR && recipient != 0xFF) {
        //Serial.println("This message is not for me.");
        return;
      }

      // Send msg over serial port
      Serial.println(msg);

      StaticJsonDocument<256> root;
      DeserializationError ERROR = deserializeJson(root, msg);

      if (ERROR) {
          //Serial.println("ERROR: Deserialization failed");
          //Serial.println(ERROR.c_str());

      } else {
          if (root.containsKey("Status")) {
              if (root["Status"] == "Task ongoing. Not available"){
                TASK_RECEIVED = true;

              } else{
                TASK_RECEIVED = false;
              }
          }
      }
  }
  


  void init_lora(void) {

      LoRa.setPins(NSS_PIN, RST_PIN, DIO_PIN);
      if (!LoRa.begin(FREQUENCY)) {
        //Serial.println("ERROR: LoRa initialization failed");
        return;

      } //Serial.println("LoRa initialization succeded");
  }



  void send_msg_lora(String msg) {
      
      StaticJsonDocument<256> root;
      DeserializationError ERROR = deserializeJson(root, msg);

      if (ERROR) {
          //Serial.println("ERROR: Deserialization failed");
          //Serial.println(ERROR.c_str());

      } else {
          if (root.containsKey("RoverId")) {
              //Serial.println("OK");
              // Send msg via LoRa
              LoRa.beginPacket();

              if (root["RoverId"] == "SensorRover") {
                LoRa.write(SENSOR_ADDR);
                
              } else if (root["RoverId"] == "ActuatorRover") {
                LoRa.write(ACTUATOR_ADDR);
              }
              LoRa.write(LOCAL_ADDR);
              LoRa.write(MSG_COUNT);
              LoRa.write(msg.length());
              LoRa.print(msg);
              LoRa.endPacket();
              MSG_COUNT++;

          } else {
            //Serial.println("ERROR: Field not found");

          }
      }
  }
