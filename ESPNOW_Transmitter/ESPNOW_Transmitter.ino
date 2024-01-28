#include "ESPNOW_Transmitter.h"



void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_MINUS_1dBm);
  WiFi.channel(channel_);

  //esp_wifi_start();
  //ESP_ERROR_CHECK(esp_wifi_set_channel(channel_, WIFI_SECOND_CHAN_NONE));

  if (esp_now_init() != ESP_OK) {   // Init ESP-NOW
    Serial.println("Error initializing ESP-NOW");
  } else {
    Serial.println("Succes: Initialized ESP-NOW");
  }

  //esp_wifi_config_espnow_rate();
  
  // Register peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, RxMacAddr, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  } else {
    Serial.println("Succes: Added peer");
  } 

  //esp_now_get_peer();

  pinMode(15,INPUT_PULLUP);
  pinMode(16,INPUT_PULLUP);
  pinMode(17,INPUT_PULLUP);
  pinMode(18,INPUT_PULLUP); 
  pinMode(19,INPUT_PULLUP);
  pinMode(21,INPUT_PULLUP);
  pinMode(22,INPUT_PULLUP);
  pinMode(23,INPUT_PULLUP); 
     
}
 
void loop() {
  data.lxAxis    = ReMap(analogRead(32), false);
  data.lyAxis    = ReMap(analogRead(33), false);
  data.rxAxis    = ReMap(analogRead(34), false);
  data.ryAxis    = ReMap(analogRead(35), false);

  // Other channels
  // data.ch1  = !digitalRead(15);
  // data.ch2   = !digitalRead(16);
  // data.ch3   = !digitalRead(17);
  // data.ch4   = !digitalRead(18);
  // data.ch5   = !digitalRead(19);
  // data.ch6   = !digitalRead(21);
  // data.ch7   = !digitalRead(22);
  // data.ch8   = !digitalRead(23);  
  
  esp_now_send(RxMacAddr, (uint8_t *) &data, sizeof(data));
  delay(5); //chip temperature issue
}
