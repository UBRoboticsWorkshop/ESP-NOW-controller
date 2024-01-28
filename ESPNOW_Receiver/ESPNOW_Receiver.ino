#include "ESPNOW_Receiver.h"



// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
#ifdef CheckMac
  char MACAd[6];
  memcpy(&MACAd, mac, sizeof(MACAd));
  for (int i = 0; i < 6; i++) {
    if (MACAd[i] != TxMacAddr[i]) return;
  }
#endif

  snprintf(sys_status.StrMac, sizeof(sys_status.StrMac), "%02x:%02x:%02x:%02x:%02x:%02x",mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  // get RSSI
  //wifi_pkt_rx_ctrl_t wifi_rx_status;

  // check the identity of Tx

  memcpy(&data, incomingData, sizeof(data));
  lastRecvTime = millis(); 
}

void setup(){
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.channel(channel_);

  //ESP_ERROR_CHECK(esp_wifi_set_channel(channel_, WIFI_SECOND_CHAN_NONE));

  ESP_ERROR_CHECK(esp_now_init());


  esp_now_register_recv_cb(OnDataRecv);
}

void loop(){
  // check the data speed and update status
  sys_status.DataUpdateSpeed = millis() - lastRecvTime;
  if (sys_status.DataUpdateSpeed > SIGNAL_TIMEOUT ) {
    sys_status.timeout = true;
  } else {
    sys_status.timeout = false;
  }

  // print out the data
  Serial.print("ReMsg from: ");
  Serial.print(sys_status.StrMac);
  Serial.print("  Time cost:");
  Serial.print(sys_status.DataUpdateSpeed);
  Serial.print("  lx:");
  Serial.print(data.lxAxis);
  Serial.print("  ly:");
  Serial.print(data.lyAxis);
  Serial.print("  rx:");
  Serial.print(data.rxAxis);
  Serial.print("  ry:");
  Serial.print(data.ryAxis);
  // Other chennels
  Serial.println();

  // Drone control & PID loop
}
