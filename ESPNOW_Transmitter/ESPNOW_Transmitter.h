#include <esp_now.h>
//#include <esp_wifi.h>
#include <WiFi.h>
#include <Arduino.h>

// Set the wifi channel (1-13)
#define channel_ 6

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t RxMacAddr[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

struct PacketData {
  uint16_t lxAxis;
  uint16_t lyAxis;
  uint16_t rxAxis;
  uint16_t ryAxis;
 
  uint16_t ch1;
  uint16_t ch2;
  uint16_t ch3;
  uint16_t ch4;  
  uint16_t ch5;
  uint16_t ch6;
  uint16_t ch7;
  uint16_t ch8;  
} data;

uint16_t ReMap(uint16_t value, bool reverse) {
  if (value >= 2200) {
    value = map(value, 2200, 4095, 2048, 4095);
  } else if (value <= 1800) {
    value = (value == 0 ? 0 : map(value, 1800, 0, 2048, 0));
  } else {
    value = 2048;
  }

  if (reverse) {
    value = 4095 - value;
  }
  //Serial.println(value);  
  return value;
}
