#include <Arduino.h>
/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-esp-now-wi-fi-web-server/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Arduino_JSON.h>

// Set your Board ID (ESP32 Sender #1 = BOARD_ID 1, ESP32 Sender #2 = BOARD_ID 2, etc)
#define BOARD_ID 1
#define CHANNEL 11

//MAC Address of the receiver 
uint8_t serverAddress[] = {0x78,0xE3,0x6D,0x09,0xEE,0x84};
//78:E3:6D:09:FC:88

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    int id;
    float temp;
    float hum;
    int readingId;
} struct_message;

//Create 2 struct_message 
struct_message myData;  // data to send
struct_message inData;  // data received

int32_t chan = 0;

float t = 0;
float h = 0;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;        // Interval at which to publish sensor readings

unsigned int readingId = 0;

// Insert your SSID
constexpr char WIFI_SSID[] = "COGECO-BE360";

int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
         if (strcmp(ssid, WiFi.SSID(i).c_str())) {
            return WiFi.channel(i);
         }         
      }
  }
  return 0;
}

float readDHTTemperature() {
  t += 1;
  Serial.println(t);
  return t;
}

float readDHTHumidity() {
  h += 10;
  Serial.println(h);
  return h;
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { 
  // Copies the sender mac address to a string
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  Serial.print("data size = ");
  Serial.println(sizeof(incomingData));
 //memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  memcpy(&inData, incomingData, sizeof(inData));
  t = inData.temp;
  h = inData.hum;
  Serial.print("ID  = ");
  Serial.println(inData.id);
  Serial.print("Setpoint temp = ");
  Serial.println(inData.temp);
  Serial.print("SetPoint humidity = ");
  Serial.println(inData.hum);
  Serial.print("reading Id  = ");
  Serial.println(inData.readingId);
}
 
void setup() {
  Serial.begin(74880);

  Serial.println();
  Serial.print("Client Board MAC Address:  ");
  Serial.println(WiFi.macAddress());

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  ESP_ERROR_CHECK(esp_wifi_set_channel(CHANNEL,  WIFI_SECOND_CHAN_NONE));
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
   //int32_t chan = getWiFiChannel(WIFI_SSID);
   //Serial.println(chan);

  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(esp_now_peer_info_t));
  peer.channel = 11;
  peer.encrypt = false;
  memcpy(peer.peer_addr, serverAddress, sizeof(uint8_t[6]));
  WiFi.printDiag(Serial);
  
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  if (esp_now_add_peer(&peer) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    //Set values to send
    myData.id = BOARD_ID;
    myData.temp = readDHTTemperature();
    myData.hum = readDHTHumidity();
    myData.readingId = readingId++;
     
    //Send message via ESP-NOW
    Serial.print("data size = ");
    Serial.println(sizeof(myData));
    esp_err_t result = esp_now_send(serverAddress, (uint8_t *) &myData, sizeof(myData));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
  }
}