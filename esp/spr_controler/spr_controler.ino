#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
String MACadd = "08:B6:1F:ED:40:CE";
uint8_t address[6] = {0x08,0xB6,0x1F,0xED,0x40,0xCE};
bool connected;

int sw_pin[8] = { 26, 32, 33, 27, 18, 19, 5, 4 };
uint8_t sw_data;
#define data_num 100
uint8_t SwData[data_num];
uint8_t equivalent_num[data_num];
uint8_t count,max_equivalent,tx_index;

uint8_t get_sw_data(){
  sw_data = 0;
  for(int i=0; i<8; i++){
    sw_data |= (digitalRead(sw_pin[i])==LOW)? 1<<i : 0;
  }
  SwData[count]=sw_data;
  count++;
  if(count==data_num) count=0;
  
  for(int i=0; i<data_num; i++){
    for(int j=i; j<data_num; j++){
      if(SwData[i]==SwData[j]) equivalent_num[i]++;
    }
  }

  max_equivalent = equivalent_num[0];
  tx_index = 0;
  for(int i=1; i<data_num; i++){
    if(max_equivalent < equivalent_num[i]){
      max_equivalent = equivalent_num[i];
      tx_index = i;
    }
  }
  return SwData[tx_index];
}

void setup() {
  SerialBT.begin("Spro_BT",true);
  Serial.begin(115200);
  
  connected = SerialBT.connect(address);
  if(connected) {
    Serial.println("Connect OK");
  } else {
    while(!SerialBT.connected(10000)) {
      Serial.println("No connect"); 
    }
  }
  
  for(int i=0; i<8; i++){
    pinMode(sw_pin[i],INPUT_PULLUP);
  }
}

void loop() {
  uint8_t tx_data;
  //tx_data = get_sw_data();
  
  sw_data = 0;
  for(int i=0; i<8; i++){
    sw_data |= (digitalRead(sw_pin[i])==LOW)? 1<<i : 0;
  }
  tx_data = sw_data;

  SerialBT.write(tx_data);
  Serial.println(tx_data,BIN);
}
