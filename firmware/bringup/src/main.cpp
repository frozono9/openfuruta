// Sensor-only bring-up. No step pulses and no enable command exist in this sketch.
#include <Arduino.h>
#include <Wire.h>
constexpr int SDA_PIN=27, SCL_PIN=14, STEP_PIN=32, DIR_PIN=26, ENABLE_PIN=25;
constexpr uint8_t AS5600=0x36;
uint32_t previous=0;
bool readRegister(uint8_t reg,uint8_t* bytes,size_t count){
 Wire.beginTransmission(AS5600);Wire.write(reg);
 if(Wire.endTransmission(false)!=0)return false;
 if(Wire.requestFrom(AS5600,(uint8_t)count)!=(int)count)return false;
 for(size_t i=0;i<count;i++)bytes[i]=Wire.read();
 return true;
}
void setup(){
 digitalWrite(ENABLE_PIN,HIGH);pinMode(ENABLE_PIN,OUTPUT);
 digitalWrite(STEP_PIN,LOW);pinMode(STEP_PIN,OUTPUT);
 digitalWrite(DIR_PIN,LOW);pinMode(DIR_PIN,OUTPUT);
 Serial.begin(115200);delay(500);
 Wire.begin(SDA_PIN,SCL_PIN,100000);Wire.setTimeOut(20);
 Serial.println("FURUTA SENSOR-ONLY: motor disabled; no STEP pulses");
 Serial.printf("SDA=%d SCL=%d STEP=%d DIR=%d EN=%d\n",SDA_PIN,SCL_PIN,STEP_PIN,DIR_PIN,ENABLE_PIN);
 for(uint8_t address=1;address<127;address++){
  Wire.beginTransmission(address);
  if(Wire.endTransmission()==0)Serial.printf("I2C device: 0x%02X\n",address);
 }
 Serial.println("time_ms,raw_angle,angle_deg,magnet_detected,magnet_too_weak,magnet_too_strong");
}
void loop(){
 if(millis()-previous<50)return;previous=millis();
 uint8_t angle[2],status;
 if(!readRegister(0x0C,angle,2)||!readRegister(0x0B,&status,1)){
  Serial.printf("%lu,ERROR_AS5600_NOT_RESPONDING\n",(unsigned long)millis());return;
 }
 uint16_t raw=((uint16_t)(angle[0]&0x0F)<<8)|angle[1];
 Serial.printf("%lu,%u,%.3f,%u,%u,%u\n",(unsigned long)millis(),raw,raw*360.0/4096.0,(status>>5)&1,(status>>4)&1,(status>>3)&1);
}
