#pragma once
#include <cmath>
#include <cstdint>
struct CaptureTransient {
 static constexpr int N=40; uint32_t t[N]{};float w[N]{};int n=0,index=0;
 void reset(){n=index=0;}
 void push(uint32_t us,float rate){
  if(!std::isfinite(rate)){reset();return;}
  if(n&&uint32_t(us-t[(index+N-1)%N])>20000)reset();
  if(n&&us==t[(index+N-1)%N])return;
  t[index]=us;w[index]=rate;index=(index+1)%N;if(n<N)++n;
 }
 bool slope(float& a)const{
  if(n<4)return false;int last=(index+N-1)%N,used=0;float sx=0,sy=0,sxx=0,sxy=0;uint32_t span=0;
  for(int k=0;k<n;k++){int j=(last+N-k)%N;uint32_t age=t[last]-t[j];if(age>60000)break;
   float x=-float(age)*1e-6f,y=w[j];sx+=x;sy+=y;sxx+=x*x;sxy+=x*y;span=age;++used;}
  float den=used*sxx-sx*sx;if(used<4||span<40000||den<1e-9f)return false;
  a=(used*sxy-sx*sy)/den;return std::isfinite(a);
 }
 bool allowed()const{float a;return slope(a)&&std::fabs(a)<60.f;}
};
