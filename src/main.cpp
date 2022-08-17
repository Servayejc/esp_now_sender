#include <Arduino.h>
/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-esp-now-wi-fi-web-server/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/
/*
JC Servaye

Automatic pairing for ESP NOW

In the server: 
- The WiFi channel is determined by your router
- Don't know the sender MAC

In the senders: 
- No access to the router
- Don't know the routeur MAC

The server is supposed running.

The sender set esp now on channel 1
The server add en entry with the broadcast address to his peer list
The sender send a pairing request in broadcast

If the server received the message we are on the good channel
    The server add the received MAC to his peer list
    The server reply to the MAC address a message containing his channel number

    The sender replace the broadcast address by the server address in his peer list

else 
    The sender repeat the process on the next channel

Note: 
  In the code below, we use the size of the message to distinct regular message to pairing message

TODO 
  Repeat the process in case of the server is not running 
  

*/

#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>


// Set your Board and Server ID 
#define BOARD_ID 1
#define SERVER_ID 0

uint8_t serverAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

//Structure to send data
//Must match the receiver structure
typedef struct struct_message {
    int id;
    float temp;
    float hum;
    int readingId;
} struct_message;

// structure to send pairing request
// structure size must be different from message size
typedef struct struct_pairing {
    int id;
    int channel;
} struct_pairing;

//Create 2 struct_message 
struct_message myData;  // data to send
struct_message inData;  // data received
struct_pairing pairingData;

enum PairingStatus {NOT_PAIRED, PAIR_REQUEST, PAIR_REQUESTED, PAIR_PAIRED,};
PairingStatus pairingStatus = NOT_PAIRED;

int channel = 1;

// simulate temperature and humidity data
float t = 0;
float h = 0;


unsigned long currentMillis = millis();
unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;        // Interval at which to publish sensor readings

unsigned int readingId = 0;   

// simulate temperature reading
float readDHTTemperature() {
  t += 1;
  return t;
}

// simulate humidity reading
float readDHTHumidity() {
  h += 10;
  return h;
}

void addPeer(const uint8_t * mac_addr, int chan){
  esp_now_peer_info_t peer;
  Serial.println(chan);
  ESP_ERROR_CHECK(esp_wifi_set_channel(chan ,  WIFI_SECOND_CHAN_NONE));
  esp_now_del_peer(serverAddress);
  memset(&peer, 0, sizeof(esp_now_peer_info_t));
  peer.channel = chan;
  peer.encrypt = false;
  memcpy(peer.peer_addr, mac_addr, sizeof(uint8_t[6]));
  if (esp_now_add_peer(&peer) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  memcpy(serverAddress, mac_addr, sizeof(uint8_t[6]));
}

void printMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { 
  Serial.print("Packet received from: ");
  printMAC(mac_addr);
  Serial.println();
  Serial.print("data size = ");
  Serial.println(sizeof(incomingData));
  if (len == sizeof(myData)){     // we received data from server
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

  if (len == sizeof(pairingData)){  // we received pairing request from server
     memcpy(&pairingData, incomingData, sizeof(pairingData));
     Serial.print("Pairing ");
    if (pairingData.id == 0) {  // the message comes from server
      printMAC(mac_addr);

      Serial.print(" on channel " );
      Serial.println(pairingData.channel);    // channel used by the server
      addPeer(mac_addr, pairingData.channel); // add the server to the peer list 
      pairingStatus = PAIR_PAIRED;                 // set the pairing status
    }
  }  
}

PairingStatus autoPairing(){
  switch(pairingStatus) {
    case PAIR_REQUEST:
    Serial.print("Pairing request on channel "  );
    Serial.println(channel);
  
    // clean esp now
    //esp_now_deinit();

    // set WiFi channel   
    ESP_ERROR_CHECK(esp_wifi_set_channel(channel,  WIFI_SECOND_CHAN_NONE));
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
    }

    // set callback routines
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
  
    // set pairing data to send to the server
    pairingData.id = BOARD_ID;     
    pairingData.channel = channel;

    // add peer and send request
    addPeer(serverAddress, channel);
    esp_now_send(serverAddress, (uint8_t *) &pairingData, sizeof(pairingData));
    pairingStatus = PAIR_REQUESTED;
    break;

    case PAIR_REQUESTED:
    // time out to allow receiving response from server
    currentMillis = millis();
    if(currentMillis - previousMillis > 100) {
      previousMillis = currentMillis;
      // time out expired,  try next channel
      channel ++;
      pairingStatus = PAIR_REQUEST;
    }
    break;

    case PAIR_PAIRED:
    //Serial.println("Paired!");
    break;
  }
  return pairingStatus;
}  

void setup() {
  Serial.begin(74880);
  //ARDUINOTRACE_INIT(74880);
  Serial.println();
  Serial.print("Client Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  Serial.println(sizeof(myData));
  pairingStatus = PAIR_REQUEST;
}  

void loop() {
  if (autoPairing() == PAIR_PAIRED) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      // Save the last time a new reading was published
      previousMillis = currentMillis;
      //Set values to send
      myData.id = BOARD_ID;
      myData.temp = readDHTTemperature();
      myData.hum = readDHTHumidity();
      myData.readingId = readingId++;
      esp_err_t result = esp_now_send(serverAddress, (uint8_t *) &myData, sizeof(myData));
    }
  }
}