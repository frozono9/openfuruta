#pragma once
#include <cmath>
#include <cstdint>
struct BalanceExcitation {
 bool armed=false,running=false,done=false,quiet=false;uint32_t quietAt=0,start=0;int event=0;
 void arm(bool enabled){armed=enabled;running=done=quiet=false;quietAt=start=0;event=0;}
 void cancel(){if(running)event=3;armed=running=quiet=false;done=true;}
 float update(uint32_t now,bool active,float e,float w,float q,float v){
  if(!armed||done)return 0;
  if(!active||!std::isfinite(e)||!std::isfinite(w)||!std::isfinite(q)||!std::isfinite(v)){if(running)cancel();else quiet=false;return 0;}
  if(!running){
   if(std::fabs(e)>.04f||std::fabs(w)>.5f||std::fabs(q)>.35f||std::fabs(v)>.7f){quiet=false;return 0;}
   if(!quiet){quiet=true;quietAt=now;}
   if(uint32_t(now-quietAt)<2000)return 0;
   running=true;start=now;event=1;
  }
  if(std::fabs(e)>.12f||std::fabs(w)>2.f||std::fabs(q)>.65f||std::fabs(v)>2.f){cancel();return 0;}
  uint32_t elapsed=now-start;
  if(elapsed>=2250){running=armed=false;done=true;event=2;return 0;}
  if(elapsed<250)return 0;
  float t=(elapsed-250)*.001f,envelope=std::sin(3.14159265359f*t/2.f);
  return .12f*envelope*envelope*(std::sin(6.28318530718f*3.f*t)+std::sin(6.28318530718f*7.f*t));
 }
 int takeEvent(){int e=event;event=0;return e;}
};
