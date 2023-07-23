#include "Arduino.h"
#include <ArduinoModbus.h>

#include <ModbusRTU.h>
#if defined(ESP8266)
 #include <SoftwareSerial.h>
 // SoftwareSerial S(D1, D2, false, 256);

 // receivePin, transmitPin, inverse_logic, bufSize, isrBufSize
 // connect RX to D2 (GPIO4, Arduino pin 4), TX to D1 (GPIO5, Arduino pin 4)
 SoftwareSerial S(4, 5);
#endif

ModbusRTU mb;


bool cbWrite(Modbus::ResultCode event, uint16_t transactionId, void* data) {
#ifdef ESP8266
  Serial.printf_P("Request result: 0x%02X, Mem: %d\n", event, ESP.getFreeHeap());
#elif ESP32
  Serial.printf_P("Request result: 0x%02X, Mem: %d\n", event, ESP.getFreeHeap());
#else
  Serial.print("Request result: 0x");
  Serial.print(event, HEX);
#endif
  return true;
}
bool cbRead(Modbus::ResultCode event, uint16_t transactionId, void* data) {
#ifdef ESP8266
  Serial.printf_P("Request result: 0x%02X, Mem: %d\n", event, ESP.getFreeHeap());
#elif ESP32
  Serial.printf_P("Request result: 0x%02X, Mem: %d\n", event, ESP.getFreeHeap());
#else
  Serial.print("Request result: 0x");
  Serial.print(event, HEX);
#endif
  return true;
}


#defined pin_485_rx 16
#defined pin_485_tx 17
#define pin_485_inverse false

void setup() {
  Serial.begin(115200);
 #if defined(ESP8266)
  S.begin(9600, SWSERIAL_8N1);
  mb.begin(&S);
 #elif defined(ESP32)
  Serial1.begin(9600,SERIAL_8N1,pin_485_rx,pin_485_tx,pin_485_inverse);
  mb.begin(&Serial1);
 #else
  Serial1.begin(9600, SERIAL_8N1);
  mb.begin(&Serial1);
  mb.setBaudrate(9600);
 #endif
  mb.master();
}

bool coils[20];
int Holdregister[10];

void loop() {
  if (!mb.slave()) {
    // mb.readCoil(1, 1, coils, 20, cbWrite);
    mb.readHreg(1,1,Holdregister,10,cbRead)
  }
  mb.task();
  yield();
}