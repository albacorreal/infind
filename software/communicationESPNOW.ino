#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0x24, 0xDC, 0xC3, 0xA7, 0x31, 0x48};// REPLACE WITH RECEIVER MAC ADDRESS

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
 	char a[32];
 	int b;
 	float c;
 	String d;
 	bool e;
} struct_message;
struct_message myData;
char dataRcv[15];

// callbacks for sending and receiving data
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
 	Serial.print("\r\nMaster packet sent:\t");
 	Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
 	memcpy(&dataRcv, incomingData, sizeof(dataRcv));
 	Serial.print("\r\nBytes received: ");
 	Serial.println(len);
 	Serial.print("From slave: ");
 	Serial.println(dataRcv);
 	Serial.println();
}
void setup() {
 	// Init Serial Monitor
 	Serial.begin(115200);

 	// Set device as a Wi-Fi Station
 	WiFi.mode(WIFI_STA);

 	// Init ESP-NOW
 	if (esp_now_init() != ESP_OK) {
 			Serial.println(F("Error initializing ESP-NOW"));
 			return;
 	}
 	Serial.print(F("Reciever initialized : "));
 	Serial.println(WiFi.macAddress());
 	
 	// Define callback functions
 	esp_now_register_send_cb(OnDataSent);
 	esp_now_register_recv_cb(OnDataRecv);

 	// Register peer
 	esp_now_peer_info_t peerInfo;
 	memcpy(peerInfo.peer_addr, broadcastAddress, 6);
 	peerInfo.channel = 0;
 	peerInfo.encrypt = false;

 	// Add peer
 	if (esp_now_add_peer(&peerInfo) != ESP_OK) {
 			Serial.println(F("Failed to add peer"));
 			return;
 	}
}

void loop() {
 	// Set values to send
 	strcpy(myData.a, "from Master");
 	myData.b = random(1, 20);
 	myData.c = 1.2;
 	myData.d = "hello";
 	myData.e = false;

 	// Send message via ESP-NOW
 	esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

 	delay(1000);
}

