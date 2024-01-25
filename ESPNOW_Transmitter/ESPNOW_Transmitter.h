#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Arduino.h>
#include <EEPROM.h>

// Set your Board ID
#define BoardID 0
#define MAX_CHANNEL 13



enum MessageType {PAIRING, DATA,};

enum PairingStatus {
  NOT_PAIRED,
  PAIR_REQUEST,
  PAIR_PAIRED,
};

struct SystemStatus {
  PairingStatus pairingstatus;
} sys_status;

struct TxData {
  MessageType msgType = DATA;
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
  MessageType msgType;
  uint8_t id;
  uint8_t voltage;
  uint8_t RSSI;
} inData;

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) { 
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

void printMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}

class ESPNOWTx {
public:
  ESPNOWTx(int a){
    uint8_t TxMACAddr[6];

    _pairingStatus = NOT_PAIRED;
    esp_read_mac(TxMACAddr, ESP_MAC_WIFI_STA);
    _pairingData.macAddr = TxMACAddr;
  };

  void pair(uint64_t RxAddress = 0x0) {
    //RxAddress = ESP.getEfuseMac();
    if (RxAddress) {
      
    }

    _pairingStatus = PAIR_REQUEST; // Use key 0 to active it 
  };

  uint8_t autoPairing(void){
    switch(_pairingStatus) {
      case PAIR_REQUEST:
      if(millis() - previousMillis > 250) { // time out to allow receiving response from Rx
        _pairingData.channel ++;
        if (_pairingData.channel > MAX_CHANNEL){
          _pairingData.channel = 1;
        }

        // set WiFi channel
        ESP_ERROR_CHECK(esp_wifi_set_channel(_pairingData.channel, WIFI_SECOND_CHAN_NONE));
        if (esp_now_init() != ESP_OK) {
          Serial.println("Error initializing ESP-NOW");
        }

        // set callback routines
        esp_now_register_recv_cb(OnDataRecv);

        // brocast request
        addPeer(_RxAddr, _pairingData.channel);
        esp_now_send(_RxAddr, (uint8_t *) &_pairingData, sizeof(_pairingData));

        previousMillis = millis();
      }
      break;

      case PAIR_PAIRED:
        // nothing to do here 
      break;

      case NOT_PAIRED:
        // nothing to do here 
      break;
    }
    return _pairingStatus;
  };

  SendData(&data){
    reyurn esp_now_send(_RxAddr, (uint8_t *) &data, sizeof(data))
  }

private:
  uint8_t _BroMACAddr[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; // Brocast addr
  uint8_t _RxAddr[6];
  unsigned long previousMillis = 0;

  PairingStatus _pairingStatus = NOT_PAIRED;
  MessageType _messageType;

  struct struct_pairing {
    MessageType msgType = PAIRING;
    uint8_t id = BoardID;
    uint8_t macAddr[6];
    uint8_t channel = 0;
  } _pairingData;

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
    memcpy(_RxAddr, mac_addr, sizeof(uint8_t[6]));
  }
};