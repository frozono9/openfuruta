#pragma once
#include <cmath>
#include <cstdint>
// Experimental causal angle-rate estimate. No future samples, no motor model.
struct CausalRate {
 static constexpr int Capacity=16;
 uint32_t time[Capacity]{};double angle[Capacity]{};int n=0,index=0;
 double last=0,unwrapped=0;
 void reset(){n=index=0;last=unwrapped=0;}
 void push(uint32_t us,double wrapped){
  if(!std::isfinite(wrapped)){reset();return;}
  if(n&&uint32_t(us-time[(index+Capacity-1)%Capacity])>20000)reset();
  if(n&&us==time[(index+Capacity-1)%Capacity])return;
  if(!n)unwrapped=wrapped;
  else {double delta=wrapped-last;constexpr double pi=3.14159265358979323846;
   if(delta>pi)delta-=2*pi;else if(delta<-pi)delta+=2*pi;unwrapped+=delta;}
  last=wrapped;time[index]=us;angle[index]=unwrapped;index=(index+1)%Capacity;if(n<Capacity)n++;
 }
 bool rate(double& result)const{
  if(n<4)return false;
  int newest=(index+Capacity-1)%Capacity;double m[3][4]={};int used=0;uint32_t span=0;
  for(int k=0;k<n;k++){
   int j=(newest+Capacity-k)%Capacity;uint32_t age=time[newest]-time[j];if(age>12000)break;
   double x=-double(age)/10000.,y=angle[j]-angle[newest],row[3]={1,x,x*x};
   for(int r=0;r<3;r++){for(int c=0;c<3;c++)m[r][c]+=row[r]*row[c];m[r][3]+=row[r]*y;}
   used++;span=age;
  }
  if(used<4||span<6000)return false;
  for(int c=0;c<3;c++){
   int p=c;for(int r=c+1;r<3;r++)if(std::fabs(m[r][c])>std::fabs(m[p][c]))p=r;
   if(std::fabs(m[p][c])<1e-9)return false;
   for(int j=c;j<4;j++){double tmp=m[p][j];m[p][j]=m[c][j];m[c][j]=tmp;}
   double divisor=m[c][c];for(int j=c;j<4;j++)m[c][j]/=divisor;
   for(int r=0;r<3;r++)if(r!=c){double f=m[r][c];for(int j=c;j<4;j++)m[r][j]-=f*m[c][j];}
  }
  result=m[1][3]*100.;return std::isfinite(result);
 }
};
