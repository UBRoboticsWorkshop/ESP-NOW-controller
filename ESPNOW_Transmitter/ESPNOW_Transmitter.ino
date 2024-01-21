#include "ESPNOW_Transmitter.h"



void setup() {
  Serial.begin(115200);

  Serial.print("TX MAC Address: ");
  Serial.println(WiFi.macAddress());

  #ifdef SAVE_CHANNEL 
    EEPROM.begin(10);
    lastChannel = EEPROM.read(0);
    Serial.println(lastChannel);
    if (lastChannel >= 1 && lastChannel <= MAX_CHANNEL) {
      channel = lastChannel; 
    }
    Serial.println(channel);
  #endif

  pairingStatus = PAIR_REQUEST; // Use key 0 to active it 
}  

void loop() {
  data.lxAxis = mapAndAdjustJoystickDeadBandValues(analogRead(32), false);
  data.lyAxis= mapAndAdjustJoystickDeadBandValues(analogRead(33), false);
  data.rxAxis= mapAndAdjustJoystickDeadBandValues(analogRead(34), false);
  data.ryAxis= mapAndAdjustJoystickDeadBandValues(analogRead(35), false);

  if (autoPairing() == PAIR_PAIRED) {
    esp_err_t result = esp_now_send(RXAddress, (uint8_t *) &data, sizeof(data));
  }
}