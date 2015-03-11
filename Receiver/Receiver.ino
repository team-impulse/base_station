#include <SPI.h>

#include <RFM98W_library.h>
RFMLib radio =RFMLib(20,7,16,21);
#define nss 20
void setup(){
  SPI.begin();
  Serial.begin(38400);
  byte my_config[5] = {0x44,0x84,0x88,0xAC,0xCD};
  radio.configure(my_config);
}
void handlePacket(RFMLib::Packet &p);
void loop(){
  if(radio.rfm_status == 0){
    radio.beginRX(); 
    attachInterrupt(7,RFMISR,RISING);
  }

  if(radio.rfm_done && radio.rfm_status==2){
    RFMLib::Packet rx;
    radio.endRX(rx);
   handlePacket(rx);
   rx.len = 0;
   while(Serial.available()){
     rx.data[rx.len] = Serial.read();
     rx.len++;
   }
   if(rx.len != 0){
     radio.beginTX(rx);
     attachInterrupt(7,RFMISR,RISING);
   }
  }
  if(radio.rfm_done && radio.rfm_status==1)
    radio.endTX();  
}

void RFMISR(){
 radio.rfm_done = true; 
}


void handlePacket(RFMLib::Packet &p){
 Serial.print((p.crc)?"PASS," : "FAIL,");//CRC
 Serial.print(p.rssi);Serial.print(",");
 Serial.print(p.snr);Serial.print(",");
 for(int i = 0;i<p.len-1;i++){
  Serial.print(p.data[i]);
  Serial.print(",");
 }
 Serial.println(p.data[p.len-1]);
}
