#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Arduino.h>
#include <EEPROM.h>



// Set your Board and Server ID
#define RxID 0
#define TxID 1
#define MAX_CHANNEL 13
//#define SAVE_CHANNEL

unsigned long currentMillis = millis();
unsigned long previousMillis = 0;


uint8_t RXAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; // Brocast addr

struct TxData {
  uint8_t msgType = DATA;
  int16_t lxAxis;
  int16_t lyAxis;
  int16_t rxAxis;
  int16_t ryAxis;
 
  int16_t CH1;
  int16_t CH2;
  int16_t CH3;
  int16_t CH4;  
  int16_t CH5;
  int16_t CH6;
  int16_t CH7;
  int16_t CH8;  
} data;

struct RxData {
  uint8_t id;
  uint8_t voltage;
  uint8_t RSSI;
} inData;

struct struct_pairing {
  uint8_t msgType;
  uint8_t id;
  uint8_t macAddr[6];
  uint8_t channel;
} pairingData;

enum PairingStatus {NOT_PAIRED, PAIR_REQUEST, PAIR_REQUESTED, PAIR_PAIRED,};
PairingStatus pairingStatus = NOT_PAIRED;

enum MessageType {PAIRING, DATA,};
MessageType messageType;

#ifdef SAVE_CHANNEL
  int lastChannel;
#endif  
int channel = 1;

void addPeer(const uint8_t * mac_addr, uint8_t chan){
  esp_now_peer_info_t peer;
  ESP_ERROR_CHECK(esp_wifi_set_channel(chan ,WIFI_SECOND_CHAN_NONE));
  esp_now_del_peer(mac_addr);
  memset(&peer, 0, sizeof(esp_now_peer_info_t));
  peer.channel = chan;
  peer.encrypt = false;
  memcpy(peer.peer_addr, mac_addr, sizeof(uint8_t[6]));
  if (esp_now_add_peer(&peer) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  memcpy(RXAddress, mac_addr, sizeof(uint8_t[6]));
}

void printMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { 
  Serial.print("Packet received from: ");
  printMAC(mac_addr);
  Serial.println();

  uint8_t type = incomingData[0];
  switch (type) {
  case DATA :      // we received data from server
    memcpy(&inData, incomingData, sizeof(inData));
    Serial.print("ID  = ");
    Serial.println(inData.id);
    Serial.print("Drone Voltage = ");
    Serial.println(inData.voltage);
    Serial.print("Drone RSSI = ");
    Serial.println(inData.RSSI);
    break;

  case PAIRING:    // we received pairing data from server
    memcpy(&pairingData, incomingData, sizeof(pairingData));
    if (pairingData.id == RxID) {              // the message comes from server
      Serial.print("Pairing done for ");
      printMAC(pairingData.macAddr);
      Serial.print(" on channel " );
      Serial.println(pairingData.channel);    // channel used by the server
      addPeer(pairingData.macAddr, pairingData.channel); // add the server  to the peer list 

      #ifdef SAVE_CHANNEL
        lastChannel = pairingData.channel;
        EEPROM.write(0, pairingData.channel);
        EEPROM.commit();
      #endif 

      pairingStatus = PAIR_PAIRED;             // set the pairing status
    }
    break;
  }  
}

PairingStatus autoPairing(){
  switch(pairingStatus) {
    case PAIR_REQUEST:
    Serial.print("Pairing request on channel ");
    Serial.println(channel);

    // set WiFi channel   
    ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
    }

    // set callback routines
    esp_now_register_recv_cb(OnDataRecv);
  
    // set pairing data to send to the server
    pairingData.msgType = PAIRING;
    pairingData.id = TxID;
    pairingData.channel = channel;

    // add peer and send request
    addPeer(RXAddress, channel);
    esp_now_send(RXAddress, (uint8_t *) &pairingData, sizeof(pairingData));
    previousMillis = millis();
    pairingStatus = PAIR_REQUESTED;
    break;

    case PAIR_REQUESTED:
    // time out to allow receiving response from server
    currentMillis = millis();
    if(currentMillis - previousMillis > 250) {
      previousMillis = currentMillis;
      // time out expired,  try next channel
      channel ++;
      if (channel > MAX_CHANNEL){
         channel = 1;
      }   
      pairingStatus = PAIR_REQUEST;
    }
    break;

    case PAIR_PAIRED:
      // nothing to do here 
    break;

    case NOT_PAIRED:
      // nothing to do here 
    break;
  }
  return pairingStatus;
}  

int mapAndAdjustJoystickDeadBandValues(int value, bool reverse) {
  if (value >= 2200) {
    value = map(value, 2200, 4095, 127, 254);
  } else if (value <= 1800) {
    value = (value == 0 ? 0 : map(value, 1800, 0, 127, 0));  
  } else {
    value = 127;
  }

  if (reverse) {
    value = 254 - value;
  }
  //Serial.println(value);  
  return value;
}

class ESPNOWTx {
public:
  Pair(uint8_t RxAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}){
    _RxAddr = RxAddress;
    pairingStatus = PAIR_REQUEST; // Use key 0 to active it 
  };

  PairingStatus autoPairing(){
    switch(pairingStatus) {
      case PAIR_REQUEST:
      Serial.print("Pairing request on channel ");
      Serial.println(channel);

      // set WiFi channel   
      ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));
      if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
      }

      // set callback routines
      esp_now_register_recv_cb(OnDataRecv);
    
      // set pairing data to send to the server
      pairingData.msgType = PAIRING;
      pairingData.id = TxID;
      pairingData.channel = channel;

      // add peer and send request
      addPeer(RXAddress, channel);
      esp_now_send(RXAddress, (uint8_t *) &pairingData, sizeof(pairingData));
      previousMillis = millis();
      pairingStatus = PAIR_REQUESTED;
      break;

      case PAIR_REQUESTED:
      // time out to allow receiving response from server
      currentMillis = millis();
      if(currentMillis - previousMillis > 250) {
        previousMillis = currentMillis;
        // time out expired,  try next channel
        channel ++;
        if (channel > MAX_CHANNEL){
          channel = 1;
        }   
        pairingStatus = PAIR_REQUEST;
      }
      break;

      case PAIR_PAIRED:
        // nothing to do here 
      break;

      case NOT_PAIRED:
        // nothing to do here 
      break;
    }
    return pairingStatus;
  }  

private:
  uint8_t _RxAddr[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; // Brocast addr

  struct struct_pairing {
    uint8_t msgType;
    uint8_t id;
    uint8_t macAddr[6];
    uint8_t channel;
  } pairingData;

  void addPeer(const uint8_t * mac_addr, uint8_t chan){
    esp_now_peer_info_t peer;
    ESP_ERROR_CHECK(esp_wifi_set_channel(chan ,WIFI_SECOND_CHAN_NONE));
    esp_now_del_peer(mac_addr);
    memset(&peer, 0, sizeof(esp_now_peer_info_t));
    peer.channel = chan;
    peer.encrypt = false;
    memcpy(peer.peer_addr, mac_addr, sizeof(uint8_t[6]));
    if (esp_now_add_peer(&peer) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
    memcpy(RXAddress, mac_addr, sizeof(uint8_t[6]));
  }

  void printMAC(const uint8_t * mac_addr){
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.print(macStr);
  }

  void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { 
    Serial.print("Packet received from: ");
    printMAC(mac_addr);
    Serial.println();

    uint8_t type = incomingData[0];
    switch (type) {
    case DATA :      // we received data from server
      memcpy(&inData, incomingData, sizeof(inData));
      Serial.print("ID  = ");
      Serial.println(inData.id);
      Serial.print("Drone Voltage = ");
      Serial.println(inData.voltage);
      Serial.print("Drone RSSI = ");
      Serial.println(inData.RSSI);
      break;

    case PAIRING:    // we received pairing data from server
      memcpy(&pairingData, incomingData, sizeof(pairingData));
      if (pairingData.id == RxID) {              // the message comes from server
        Serial.print("Pairing done for ");
        printMAC(pairingData.macAddr);
        Serial.print(" on channel " );
        Serial.println(pairingData.channel);    // channel used by the server
        addPeer(pairingData.macAddr, pairingData.channel); // add the server  to the peer list 
        pairingStatus = PAIR_PAIRED;             // set the pairing status
      }
      break;
    }  
  }
};