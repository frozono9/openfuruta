#pragma once
#include "Arduino.h"
struct WireMock{
 bool shaft;int reg=0,index=0;explicit WireMock(bool shaft=false):shaft(shaft){}
 void end(){}void begin(int,int,int){}void setTimeOut(int){}void beginTransmission(int){}
 void write(int r){reg=r;index=0;}int endTransmission(bool){return 0;}
 int requestFrom(uint8_t,int n){return n;}
 uint8_t read(){
  int address=reg+index++;int raw=(int)round((shaft?shaftPlantAngle:sensorPlantAngle)*(4096/(2*PI)));raw=((raw%4096)+4096)%4096;
  if(address==0x0B)return 32;if(address==0x0C)return raw>>8;if(address==0x0D)return raw&255;return 0;
 }
};using TwoWire=WireMock;
inline WireMock Wire(false),Wire1(true);
