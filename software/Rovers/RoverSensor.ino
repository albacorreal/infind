//================================================================================
// PROGRAM INFO
  /* ROVER SENSOR
  *
  *
  *
  */


//================================================================================

  #include <string>
  #include <ArduinoJson.h>
  #include <Button2.h>
  #include <WiFi.h>
  #include <PubSubClient.h>
  #include <esp_now.h>
  #include <esp_wifi.h>
  #include <LoRa.h>
  #include <Wire.h>

  #include <DHTesp.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BMP085.h>
  #include <Adafruit_MPU6050.h>
  #include <TinyGPSPlus.h>


  #define ROVER_ID "SensorRover"

  #define LED_PIN 2
  #define BTN_PIN 0
  #define DHT_PIN 25
  #define UVR_PIN 35
  #define RX2_PIN 16
  #define TX2_PIN 17
  #define NSS_PIN 15
  #define RST_PIN 32
  #define DIO_PIN 36
  #define MLF_PIN 26
  #define MRF_PIN 27
  #define MLB_PIN 14
  #define MRB_PIN 12

  #define TCA_ADDR 0x70
  #define BMP_CHANNEL 0
  #define MPU_CHANNEL 1

  #define seaLevelPressure_hPa 1013.25

  #define TEMP_THRES 40
  #define HUM_THRES 70
  #define PRES_THRES 103300
  #define UVR_THRES 600

  #define WIFI_CHANNEL 0

  #define FREQUENCY 433E6
  #define LOCAL_ADDR 0x13
  #define RECEIVER_ADDR 0x12

  #define BUFFER_SIZE 512
  #define MSG_MAX_SIZE 250

  #define CHECK_TIMEOUT 1000
  #define SEND_TIMEOUT 5000
  #define CONNECT_TIMEOUT 15000
  #define RETRY_TIMEOUT 30000

  #define FINISHED 0
  #define BUSY 1
  #define MOVING 1
  #define MEASURING 2
  #define EMERGENCY 3
  #define CHARGING 4

  #define _EARTHQUAKE 6
  #define _LOW_BATTERY 7

  #define NOT_PRESSED 0
  #define CLICK 1
  #define LONG_CLICK 4
  #define DOUBLE_CLICK 2
  #define TRIPLE_CLICK 3


  Button2 btn;
  DHTesp dht;
  Adafruit_BMP085 bmp;
  Adafruit_MPU6050 mpu;
  TinyGPSPlus gps;
  HardwareSerial Serial_gps(1);


  WiFiClient wifi_client;
  PubSubClient mqtt_client(wifi_client);


  const String SSID = "infind";
  const String PASSWORD = "zancudo1518wifi";
  //const String SSID = "MOVISTAR_D5E8";
  //const String PASSWORD = "33E26BEEC5BEA3F5611C";
  
  
  const String MQTT_SERVER = "iot.ac.uma.es";
  const String MQTT_USER = "II15";
  const String MQTT_PASS = "qX5nQDDd";

  const String STATUS_TOPIC = "II15/Rovers";
  const String DATA_TOPIC = "II15/Sensors";
  const String EMERGENCY_TOPIC = "II15/Emergency";

  const uint8_t RECEIVER[] = { 0x24, 0xDC, 0xC3, 0xA7, 0x31, 0x48 };
  

  bool EXECUTE = false;
  bool DONE = false;
  bool PROBLEM = false;
  bool SEND = false;

  bool EARTHQUAKE = false;
  bool LOW_BATTERY = false;
  bool FLAG = false;

  bool FROM_PARTNER = false;


  uint8_t STATE = FINISHED;
  uint8_t _STATUS = FINISHED;
  uint8_t BUTTON = NOT_PRESSED;
  uint8_t PWM_F, PWM_B = 0;


  unsigned long CHECK_TIMER = 0;
  unsigned long SEND_TIMER = 0;
  unsigned long CONNECT_TIMER = 0;
  unsigned long TIMESTAMP = 0;

  byte MSG_COUNT = 0;
  

  float latitude = 0;
  float longitude = 0;
  float altitude = 0;
  float temperature = 0;
  float humidity = 0;
  float pressure = 0;
  float radiation = 0;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;


  String DEVICE_ID;
  String msg;
  String _status_;



//================================================================================

  /* Function to convert MAC to string
   * @brief return mac address as a string
   * \param mac */
  String mac2str(const uint8_t *mac);

  /* Function to generate message
   * @brief return string with json fields according to type
   * \param type message according to state */
  String get_msg(uint8_t TYPE);


  /* Function to connect WiFi network 
   * \param ssid
   * \param passphrase */
  void init_wifi(void);

  /* Function to recieve and process MQTT messages
   * \param topic mqtt topic
   * \param payload message content
   * \param len payload length */
  void mqtt_callback(char *topic, byte *payload, unsigned int len);

  /* Function to connect MQTT network
   * \param mqtt_server broker address
   * \param mqtt_user
   * \param mqtt_pass
   * \param device_id */
  void init_mqtt(void);


  /* Function to be called when ESP-NOW messages are sent
   * \param mac receiver mac address
   * \param status data sent status */
  void esp_now_send_callback(const uint8_t *mac, esp_now_send_status_t status);

  /* Function to be called when ESP-NOW messages are recieved
   * \param mac sender mac address
   * \param data message content
   * \param len data size */
  void esp_now_recv_callback(const uint8_t *mac, const uint8_t *data, int len);

  /* Function to connect ESP-NOW
   * @brief initialize esp-now and add peers */
  void init_esp_now(void);

  /* Function to enable ESP-NOW while WiFi disconnected
   * @brief change to wifi mode station a set peer channel */
  void enable_esp_now(void);


  /* Function to be called when LoRa messages are recieved
   * \param packetSize */
  void lora_recv_callback(int packetSize);

  /* Function to connect LoRa
   * @brief initialize LoRa, frequency and keyword */
  void init_lora(void);

  /* Function to send message via LoRa
   * \param msg */
  void send_msg_lora(String msg);


  /* Function to handle network status
   * @brief check if wifi or mqtt are disconnected
   * and handle reconnection */
  void handle_network(void);

  /* Function to handle rover states
   * @brief change state according to task */
  void handle_state(void);


  /* Function to handle single click interrupt
   * @brief set done flag true
   * \param btn */
  void singleClick(Button2& btn);

  /* Function to handle long click interrupt
   * @brief set low battery emergency
   * \param btn */
  void longClick(Button2& btn);

  /* Function to handle double click interrupt
   * @brief 
   * \param btn */
  void doubleClick(Button2& btn);

  /* Function to handle triple click interrupt
   * @brief 
   * \param btn */
  void tripleClick(Button2& btn);

  /* Function to initialize button
   * @brief initialize button interrupt callbacks */
  void init_btn(void);

  /* Function to handle button interrupts
   * @brief check if button was pressed */
  void handle_btn(void);


  /* Function to select TCA9548A channel 
   * @brief change i2c channel
   * \param channel */
  void tca_select(uint8_t channel);

  /* Function to search for connected I2C devices 
   * @brief scan every i2c channel */
  void scan_i2c_bus(void);

  /* Function to read DHT11
   * @brief return temperature and humidity */
  void read_dht(void);

  /* Function to read GUVA-S12SD
   * @brief return radiation
   * \param pin */
  void read_uvr(void);

  /* Function to read BMP180
   * @brief return pressure and altitude
   * \param channel TCA9548A channel */
  void read_bmp(void);

  /* Function to read MPU6050
   * @brief return acceleration and gyro
   * \param channel TCA9548A channel */
  void read_mpu(void);

  /* Function to read NEO-6M
   * @brief return latitude, longitude, speed, date and time */
  void read_gps(void);

  /* Function to configure connected hardware
   * @brief search for i2c devices and initialize sensors
   * in their corresponding pins, channels or ports */
  void init_hw(void);



//================================================================================

  void setup() {

      // setup begin
      Serial.begin(115200);
      Serial.println();
      Serial.println("Begin setup...");

      // board id
      DEVICE_ID = "ESP_" + String(ESP.getEfuseMac());
      Serial.println("Device ID: " + DEVICE_ID);

      while(!Serial);

      // communication
      init_wifi();
      init_mqtt();
      init_esp_now();
      //init_lora();

      // hardware
      init_hw();
      init_btn();

      // setup end
      Serial.println("End setup at " + String(millis()) + "ms");
      Serial.println();

  }
 


  void loop() {
      
      handle_network();
      handle_state();
      handle_btn();


      if (millis() - CHECK_TIMER >= CHECK_TIMEOUT) {
        CHECK_TIMER = millis();

        read_mpu();

        if (abs(accel.acceleration.z) > 12 || abs(accel.acceleration.y) > 12) {
          PROBLEM = true;
          EARTHQUAKE = true;
          FROM_PARTNER = false;

        }
      }

      if (millis() - SEND_TIMER >= SEND_TIMEOUT) {
        SEND_TIMER = millis();
        SEND = true;
        read_gps();
        msg = get_msg(_STATUS);

        mqtt_client.publish(STATUS_TOPIC.c_str(), msg.c_str());
        Serial.println("Message published [" + STATUS_TOPIC + "] " + msg);

        //send_msg_lora(msg);
        Serial.println("Package sent");

      }

      if (STATE == MEASURING) {
          read_dht();
          read_uvr();
          read_bmp();
          msg = get_msg(MEASURING);

          mqtt_client.publish(DATA_TOPIC.c_str(), msg.c_str());
          Serial.println("Message published [" + DATA_TOPIC + "] " + msg);

          //send_msg_lora(msg);
          Serial.println("Package sent");

          if (!PROBLEM) {
              DONE = true;

          }

      } //lora_recv_callback(LoRa.parsePacket());
      analogWrite(MLF_PIN, PWM_F);
      analogWrite(MRF_PIN, PWM_F);
      analogWrite(MLB_PIN, PWM_B);
      analogWrite(MRB_PIN, PWM_B);
  }



//================================================================================

  String mac2str(const uint8_t *mac) {

    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return(macStr);

  }



  String get_msg(uint8_t TYPE) {

    String msg;
    StaticJsonDocument<512> root;

    root["Sender"] = ROVER_ID;
    root["Timestamp"] = millis() + TIMESTAMP;
    root["Latitude"] = latitude;
    root["Longitude"] = longitude;
    
    switch (TYPE) {
      case FINISHED:
        root["Status"] = "Task finished. Available";
        break;

      case BUSY:
        root["Status"] = "Task ongoing. Not available";
        break;

      case MEASURING:
        root["Altitude"] = altitude;
        root["Temperature"] = temperature;
        root["Humidity"] = humidity;
        root["Pressure"] = pressure;
        root["Radiation"] = radiation;
        
        FLAG = false;
        _status_ = "";
        if (temperature >= TEMP_THRES) {
          _status_ += " Temperature";
          FLAG = true;
        } if (humidity >= HUM_THRES) {
          _status_ += " Humidity";
          FLAG = true;
        } if (pressure >= PRES_THRES) {
          _status_ += " Pressure";
          FLAG = true;
        } if (radiation >= UVR_THRES) {
          _status_ += " Radiation";
          FLAG = true;
        } if (FLAG) {
          root["Status"] = "Interesting location." + _status_;
        } else {
          root["Status"] = "";
        } break;

      case EMERGENCY:
        root["Status"] = "Emergency state";
        break;

      case _EARTHQUAKE:
        root["Status"] = "Earthquake";
        break;

      case _LOW_BATTERY:
        root["Status"] = "Low battery";
        break;

    } serializeJson(root, msg);
    return msg;

  }



  void init_wifi(void) {

    WiFi.mode(WIFI_MODE_APSTA);
    WiFi.begin(SSID, PASSWORD);

    Serial.print("Connecting to " + SSID + "...");
    unsigned long CONNECT_TIMER = millis();
    while (WiFi.status() != WL_CONNECTED) {
      if (millis() - CONNECT_TIMER >= CONNECT_TIMEOUT) {
        Serial.println("TIMEOUT");
        return;

      } Serial.print(".");
      delay(200);

    } Serial.println("Connected");
    Serial.println("   IP  address: " + WiFi.localIP().toString());
    Serial.println("   MAC address: " + WiFi.macAddress());

  }



  void mqtt_callback(char *topic, byte *payload, unsigned int len) {

    String msg = String(std::string((char *) payload, len).c_str());
    Serial.println();
    Serial.print("Message recieved [ " + String(topic) + " ] ");
    Serial.println(msg);

    if (String(topic) == STATUS_TOPIC) {
      StaticJsonDocument<512> root;
      DeserializationError ERROR = deserializeJson(root, msg);

        if (ERROR) {
          Serial.println("ERROR: Deserialization failed");
          Serial.println(ERROR.c_str());

        } else {
          if (root.containsKey("")) {
            Serial.print("Message is");
              
          } else {
            Serial.println("ERROR: Field not found");

          }
        }

      } else {
        Serial.println("ERROR: Unknown topic");
      }
  }



  void init_mqtt(void) {

    mqtt_client.setServer(MQTT_SERVER.c_str(), 1883);
    mqtt_client.setBufferSize(BUFFER_SIZE);
    mqtt_client.setCallback(mqtt_callback);

    unsigned long CONNECT_TIMER = millis();
    while (!mqtt_client.connected()) {
      Serial.print("Attempting MQTT connection... ");
      if (mqtt_client.connect(DEVICE_ID.c_str(), MQTT_USER.c_str(), MQTT_PASS.c_str())) {
        Serial.println("Connected to broker: " + MQTT_SERVER);
        //mqtt_client.subscribe(.c_str());

      } else {
        if (millis() - CONNECT_TIMER >= CONNECT_TIMEOUT) {
          Serial.println("TIMEOUT");
          return;

        } Serial.println("ERROR: " + String(mqtt_client.state()));
        Serial.print("Retrying in 5s... ");
        delay(5000);

      }
    }
  }



  void esp_now_send_callback(const uint8_t *mac, esp_now_send_status_t status) {

      Serial.println();
      Serial.print("Message sent to MAC " + mac2str(mac));
      if (status == ESP_NOW_SEND_SUCCESS) {

          Serial.println(" successed");

      } else {

          Serial.println(" failed");
      }
  }



  void esp_now_recv_callback(const uint8_t *mac, const uint8_t *data, int len) {

      char data_recv[MSG_MAX_SIZE];
      memcpy(&data_recv, data, sizeof(data_recv));

      Serial.println();
      Serial.print("Message received [ " + mac2str(mac) + " / ");
      Serial.println(String(len) + " bytes ]");
      Serial.println(data_recv);

      StaticJsonDocument<512> root;
      DeserializationError ERROR = deserializeJson(root, String(data_recv));

      if (ERROR) {
          Serial.println("ERROR: Deserialization failed");
          Serial.println(ERROR.c_str());

      } else {
          if (root.containsKey("Status")) {
            if (root["Status"] == "Earthquake") {
              PROBLEM = true;
              EARTHQUAKE = true;
              FROM_PARTNER = true;

            } else {
              Serial.println("You are on your own");

            }
          
          } else {
            Serial.println("ERROR: Field not found");

          }
      }
  }



  void init_esp_now(void) {

      if (esp_now_init() != ESP_OK) {
          Serial.println("ERROR: ESP-NOW initialization failed");
          while (1);
      }
      esp_now_register_send_cb(esp_now_send_callback);
      esp_now_register_recv_cb(esp_now_recv_callback);
      Serial.println("ESP-NOW initialization succeded");

      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, RECEIVER, 6);
      peerInfo.channel = WIFI_CHANNEL;
      peerInfo.ifidx = WIFI_IF_STA;
      peerInfo.encrypt = false;

      if (esp_now_add_peer(&peerInfo) != ESP_OK) {
          Serial.println("ERROR: Failed to add peer");
          return;
      }
      Serial.print("Peer with MAC " + mac2str(peerInfo.peer_addr) + " added");
      Serial.println(" successfully on channel " + String(WiFi.channel()));

  }



  void enable_esp_now(void) {

      ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
      wifi_config_t sta_config = {
          .sta = { .channel = 0 }
      };
      ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
      ESP_ERROR_CHECK(esp_wifi_start());

  }



  void lora_recv_callback(int packetSize) {

    if (packetSize == 0) return;
    Serial.println("Message received. Packet info:");

    int recipient = LoRa.read();
    byte sender = LoRa.read();
    byte msg_id = LoRa.read();
    byte msg_length = LoRa.read();

    String msg = "";

    while (LoRa.available()) {
      msg += (char)LoRa.read();
    }

    if (msg_length != msg.length()) {
      Serial.println("ERROR: message length does not match length");
      return;

    } if (recipient != LOCAL_ADDR && recipient != 0xFF) {
      Serial.println("This message is not for me.");
      return;
    }

    Serial.println("Received from: 0x" + String(sender, HEX));
    Serial.println("Sent to: 0x" + String(recipient, HEX));
    Serial.println("Message ID: " + String(msg_id));
    Serial.println("Message length: " + String(msg_length));
    Serial.println("Message: " + msg);
    Serial.println("RSSI: " + String(LoRa.packetRssi()));
    Serial.println("Snr: " + String(LoRa.packetSnr()));
    Serial.println();

    StaticJsonDocument<256> root;
    DeserializationError ERROR = deserializeJson(root, msg);

    if (ERROR) {
        Serial.println("ERROR: Deserialization failed");
        Serial.println(ERROR.c_str());

    } else {
        if (root.containsKey("Priority")) {
          EXECUTE = true;
          if (root.containsKey("Timestamp")) {
            unsigned long time = root["Timestamp"];
            TIMESTAMP = time - millis();

          } else {
            Serial.println("ERROR: Field \Timestamp\ not found");

          }

        } else {
          Serial.println("ERROR: Field not found");

        }
    }
  }



  void init_lora(void) {

      LoRa.setPins(NSS_PIN, RST_PIN, DIO_PIN);
      if (!LoRa.begin(FREQUENCY)) {
        Serial.println("ERROR: LoRa initialization failed");
        return;

      } Serial.println("LoRa initialization succeded");
  }



  void send_msg_lora(String msg) {

      LoRa.beginPacket();
      LoRa.write(RECEIVER_ADDR);
      LoRa.write(LOCAL_ADDR);
      LoRa.write(MSG_COUNT);
      LoRa.write(msg.length());
      LoRa.print(msg);
      LoRa.endPacket();
      MSG_COUNT++;

  }



  void handle_network(void) {

      if (WiFi.status() != WL_CONNECTED) {
          if (millis() - CONNECT_TIMER >= RETRY_TIMEOUT) {
              CONNECT_TIMER = millis();
              init_wifi();

          } if (WiFi.status() != WL_CONNECTED) {
              enable_esp_now();

          }

      } if (!mqtt_client.connected()) {
          if (millis() - CONNECT_TIMER >= RETRY_TIMEOUT) {
              CONNECT_TIMER = millis();
              init_mqtt();

              if (!mqtt_client.connected()) {
                  enable_esp_now();
              
              } else {
                  mqtt_client.loop();

              }   
          }
      }
  }




  void handle_state(void) {

      switch (STATE) {
        case FINISHED:
          PWM_F = 0;
          PWM_B = 0;
          if (PROBLEM) {
            PROBLEM = false;
            STATE = EMERGENCY;
            _STATUS = EMERGENCY;
          
          } else if (EXECUTE) {
            EXECUTE = false;
            STATE = MOVING;
            _STATUS = BUSY;
            PWM_F = 255;

          } break;

        case MOVING:
          if (PROBLEM) {
            PROBLEM = false;
            STATE = EMERGENCY;
            _STATUS = EMERGENCY;
            PWM_F = 0;

          } else if (DONE) {
            DONE = false;
            STATE = MEASURING;
            _STATUS = BUSY;
            PWM_F = 0;

          } break;

        case MEASURING:
          if (PROBLEM) {
            PROBLEM = false;
            STATE = EMERGENCY;
            _STATUS = EMERGENCY;

          } else if (DONE) {
            DONE = false;
            STATE = FINISHED;
            _STATUS = FINISHED;

          } break;

        case EMERGENCY:
          if (EXECUTE) {
            Serial.println("Heading to base");
            PWM_B = 255;
            if (DONE) {
              EXECUTE = false;
              DONE = false;
              STATE = CHARGING;
              EARTHQUAKE = false;
              PWM_B = 0;
              Serial.println("Arrived to base");

            }

          } if (SEND) {
            if (EARTHQUAKE) {
              msg = get_msg(_EARTHQUAKE);

              if (!FROM_PARTNER) {
                char data[MSG_MAX_SIZE];
                strncpy(data, msg.c_str(), MSG_MAX_SIZE);
                esp_now_send(0, (uint8_t *) data, strlen(data));

              }

              mqtt_client.publish(EMERGENCY_TOPIC.c_str(), msg.c_str());
              Serial.println("Message published [" + EMERGENCY_TOPIC + "] " + msg);

            } if (LOW_BATTERY) {
              msg = get_msg(_LOW_BATTERY);

              char data[MSG_MAX_SIZE];
              strncpy(data, msg.c_str(), MSG_MAX_SIZE);
              esp_now_send(0, (uint8_t *) data, strlen(data));

              mqtt_client.publish(EMERGENCY_TOPIC.c_str(), msg.c_str());
              Serial.println("Message published [" + EMERGENCY_TOPIC + "] " + msg);

            } SEND = false;

          } break;

        case CHARGING:
          Serial.println("Charging");
          EARTHQUAKE = false;
          if (DONE) {
            DONE = false;
            PROBLEM = false;
            STATE = FINISHED;
            _STATUS = FINISHED;
            LOW_BATTERY = false;
            Serial.println("Charged");

          } break;
      }
  }



  void singleClick(Button2& btn) {

      Serial.println("click\n");
      BUTTON = CLICK;

  }



  void longClick(Button2& btn) {

      Serial.println("long click\n");
      BUTTON = LONG_CLICK;

  }



  void doubleClick(Button2& btn) {

      Serial.println("double click\n");
      BUTTON = DOUBLE_CLICK;

  }



  void tripleClick(Button2& btn) {

      Serial.println("triple click\n");
      BUTTON = TRIPLE_CLICK;

  }



  void init_btn() {

    btn.begin(BTN_PIN);
    btn.setClickHandler(singleClick);
    btn.setLongClickHandler(longClick);
    btn.setDoubleClickHandler(doubleClick);
    btn.setTripleClickHandler(tripleClick);

  }



  void handle_btn(void) {

      btn.loop();

      switch (BUTTON) {
        case CLICK:
          if (DONE) {
            DONE = false;
          } else {
            DONE = true;
          } break;

        case LONG_CLICK:
          PROBLEM = true;
          LOW_BATTERY = true;
          break;

        case DOUBLE_CLICK:
          EXECUTE = true;
          break;

        case TRIPLE_CLICK:
          break;

      } BUTTON = NOT_PRESSED;
  }



  void tca_select(uint8_t channel) {

      if (channel > 7) {
          return;

      } else {
          Wire.beginTransmission(TCA_ADDR);
          Wire.write(1 << channel);
          Wire.endTransmission();
      }
  }



  void scan_i2c_bus(void) {

      Wire.begin();
      Serial.println("I2C bus scan...");

      for (uint8_t i = 0; i < 8; i++) {
          tca_select(i);
          Serial.println("  Channel " + String(i)); 

          for (uint8_t addr = 0; addr <= 127; addr++) {
              if (addr == TCA_ADDR) continue;
              Wire.beginTransmission(addr);
              if (!Wire.endTransmission()) {
                  Serial.print("   - Found I2C 0x");
                  Serial.println(addr, HEX);

              }
          }
      } Serial.println();
  }



  void read_dht(void) {

      Serial.println("Reading DHT11 measurements...");

      temperature = dht.getTemperature();
      Serial.print("  Temperature (ºC): ");
      Serial.print(temperature);

      humidity = dht.getHumidity();
      Serial.print("  Humidity (%): ");
      Serial.println(humidity);
      Serial.println();

  }



  void read_uvr(void) {

      Serial.println("Reading GUVA-S12SD measurements...");

      radiation = analogRead(UVR_PIN);
      Serial.print("   UV Radiation (mV): ");
      Serial.println(radiation);
      Serial.println();

  }



  void read_bmp(void) {

      Serial.println("Reading BMP180 measurements...");
      tca_select(BMP_CHANNEL);

      Serial.print("   Temperature = ");
      Serial.print(bmp.readTemperature());
      Serial.println(" ºC");
            
      Serial.print("   Pressure = ");
      pressure = bmp.readPressure();
      Serial.print(pressure);
      Serial.println(" Pa");

      Serial.print("   Pressure at sealevel (calculated) = ");
      Serial.print(bmp.readSealevelPressure());
      Serial.println(" Pa");

      Serial.print("   Altitude = ");
      altitude = bmp.readAltitude(102600);
      Serial.print(altitude);
      Serial.println(" m");

      Serial.print("   Real altitude = ");
      Serial.print(bmp.readAltitude(seaLevelPressure_hPa * 100));
      Serial.println(" m");
      Serial.println();

  }



  void read_mpu(void) {

      Serial.println("Reading MPU6050 measurements...");
      tca_select(MPU_CHANNEL);

      mpu.getEvent(&accel, &gyro, &temp);

      Serial.print("   Acceleration X: ");
      Serial.print(accel.acceleration.x);
      Serial.print(", Y: ");
      Serial.print(accel.acceleration.y);
      Serial.print(", Z: ");
      Serial.print(accel.acceleration.z);
      Serial.println(" m/s^2");

      Serial.print("   Rotation X: ");
      Serial.print(gyro.gyro.x);
      Serial.print(", Y: ");
      Serial.print(gyro.gyro.y);
      Serial.print(", Z: ");
      Serial.print(gyro.gyro.z);
      Serial.println(" rad/s");

      Serial.print("   Temperature: ");
      Serial.print(temp.temperature);
      Serial.println(" ºC");
      Serial.println();

  }
    


  void read_gps() {

      Serial.println("Reading NEO-6M measurements...");

      if (Serial_gps.available() > 0) {
          Serial.println("Serial port available"); //
          if (gps.encode(Serial_gps.read())) {
              Serial.println("Character read"); //

              if (gps.location.isValid()) {
                  Serial.print(F("- latitude: "));
                  latitude = gps.location.lat();
                  Serial.println(latitude);

                  Serial.print(F("- longitude: "));
                  longitude = gps.location.lng();
                  Serial.println(longitude);

                  Serial.print(F("- altitude: "));
                  if (gps.altitude.isValid()) {
                      Serial.println(gps.altitude.meters());

                  } else {
                      Serial.println(F("INVALID"));

                  }
                    
              } else {
                Serial.println(F("- location: INVALID"));
              }

              Serial.print(F("- speed: "));
              if (gps.speed.isValid()) {
                  Serial.print(gps.speed.kmph());
                  Serial.println(F(" km/h"));

              } else {
                  Serial.println(F("INVALID"));

              }
              Serial.print(F("- GPS date & time: "));
              if (gps.date.isValid() && gps.time.isValid()) {
                  Serial.print(gps.date.year());
                  Serial.print(F("-"));
                  Serial.print(gps.date.month());
                  Serial.print(F("-"));
                  Serial.print(gps.date.day());
                  Serial.print(F(" "));
                  Serial.print(gps.time.hour());
                  Serial.print(F(":"));
                  Serial.print(gps.time.minute());
                  Serial.print(F(":"));
                  Serial.println(gps.time.second());
              
              } else {
                  Serial.println(F("INVALID"));

              } Serial.println();
          }
      }

      if (millis() > 5000 && gps.charsProcessed() < 10) {
          Serial.println(F("No GPS data received: check wiring"));

      } Serial.println();
  }



  void init_hw(void) {

      scan_i2c_bus();

      Serial.print("Begin DHT11      initialization... ");
      dht.setup(DHT_PIN, DHTesp::DHT11);
      Serial.println("Setup on pin " + String(DHT_PIN));


      Serial.print("Begin GUVA-S12SD initialization... ");
      Serial.println("Setup on pin " + String(UVR_PIN));


      Serial.print("Begin BMP180     initialization... ");
      tca_select(BMP_CHANNEL);

      if (!bmp.begin()) {
        Serial.println("Failed to initialize");
        return;
      }
      Serial.println("Setup on TCA9548A channel " + String(BMP_CHANNEL));


      Serial.print("Begin MPU6050    initialization... ");
      tca_select(MPU_CHANNEL);

      if (!mpu.begin()) {
        Serial.println("Failed to initialize");
        return;
      }
      Serial.print("Setup on TCA9548A channel " + String(MPU_CHANNEL));
      Serial.println(". Begin config...");

      mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
      Serial.print("   Accelerometer range set to: ");
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
      Serial.print("   Gyro range set to: ");
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
      Serial.print("   Filter bandwidth set to: ");
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
      Serial.print("Begin NEO-6M     initialization...");
      Serial_gps.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN);

      pinMode(MLF_PIN, OUTPUT);
      pinMode(MRF_PIN, OUTPUT);
      pinMode(MLB_PIN, OUTPUT);
      pinMode(MRB_PIN, OUTPUT);

      Serial.println(F(" Setup done"));

  }




