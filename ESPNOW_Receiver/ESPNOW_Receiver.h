#include <esp_now.h>
//#include <esp_wifi.h>
#include <WiFi.h>
#include <Arduino.h>

// Check Tx identity
//#define CheckMac

// Set the wifi channel (1-13)
#define channel_ 6

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t TxMacAddr[] = {0xA8,0x42,0xE3,0xCD,0x5F,0x04};

// Timeout
#define SIGNAL_TIMEOUT 100 //ms
unsigned long lastRecvTime = 0;

struct SystemStatus{
  unsigned int DataUpdateSpeed;
  bool timeout;
  char StrMac[18];
} sys_status;

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

void printMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}