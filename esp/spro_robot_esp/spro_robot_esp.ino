#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

HardwareSerial STMSerial(1);

void setup() {
  SerialBT.begin("Spro_BT");
  Serial.begin(115200);
  STMSerial.begin(115200,SERIAL_8N1,22,23);
}

void loop() {
  uint8_t sw_data;
  if(SerialBT.available()){
    sw_data = SerialBT.read();
  }
  STMSerial.write(sw_data);
  Serial.println(sw_data,BIN);
}
