#include "ESPNOW_Transmitter.h"

ESPNOWTx espnowtx(1);

void setup() {
  Serial.begin(115200);

  Serial.print("TX MAC Address: ");
  Serial.println(WiFi.macAddress());

  espnowtx.pair(); // Use key 0 to active it 
}  

void loop() {
  data.lxAxis = mapAndAdjustJoystickDeadBandValues(analogRead(32), false);
  data.lyAxis= mapAndAdjustJoystickDeadBandValues(analogRead(33), false);
  data.rxAxis= mapAndAdjustJoystickDeadBandValues(analogRead(34), false);
  data.ryAxis= mapAndAdjustJoystickDeadBandValues(analogRead(35), false);

  if (espnowtx.autoPairing() == 2) {
    espnowtx.SendData(data);
  }
}