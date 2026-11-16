#pragma once
#include <math.h>
// Experimental controller. Disabled by default; currently simulation only.
// Predict the pendulum through the queued motor input before state feedback.
struct RobustBalanceParameters {
 float kp=296.62167354f, kd=21.57145258f, armP=4.47213595f, armD=4.44273215f;
 float g=133.75985282f,b=1.29481920f,d=4.36833114f;
 int horizon=16;
 int swingHorizon=17;bool blendSwingModel=false;bool separateSwingModel=false;float swingG=133.76f,swingB=1.2948f,swingD=4.3683f;
 float rateFilterLag=.008f;float trimRate=.0005f;
 float captureAngle=.4f,captureSpeed=4.f,captureArm=.85f,captureMotor=5.f;
};
// horizon is a duration in nominal 2 ms units, not a count of variable ticks.
inline float predictionDuration(const float* durations,int i){
 const float dt=durations?durations[i]:.002f;
 return isfinite(dt)&&dt>0?dt:.002f;
}
inline void timedPendulumPrediction(float& a,float& w,const float* history,
    const float* durations,int index,int size,int horizon,float g,float b,float d,bool swing,float rateFilterLag=.008f){
 int n=horizon;if(n<1)n=1;if(n>size)n=size;
 float remaining=.002f*n,firstDuration=0;
 int start=index,count=0;
 while(remaining>1.e-6f&&count<size){
  start=(start+size-1)%size;
  firstDuration=fminf(remaining,predictionDuration(durations,start));
  remaining-=firstDuration;
  ++count;
 }
 auto acceleration=[&](float u){return swing?
     -g*sinf(a)-d*w-b*cosf(a)*u:g*a-d*w+b*u;};
 w+=rateFilterLag*acceleration(history[start]);
 for(int k=0;k<count;k++){
  const int i=(start+k)%size;
  float segment=k==0?firstDuration:predictionDuration(durations,i);
  while(segment>1.e-6f){
   const float dt=fminf(.002f,segment);
   w+=dt*acceleration(history[i]);
   a+=dt*w;
   segment-=dt;
  }
 }
}
inline void robustPrediction(float& e,float& w,const float* history,int index,int size,
    const RobustBalanceParameters& p,const float* durations=nullptr){
 timedPendulumPrediction(e,w,history,durations,index,size,p.horizon,p.g,p.b,p.d,false,p.rateFilterLag);
}
inline void robustSwingPrediction(float& a,float& w,const float* history,int index,int size,
    const RobustBalanceParameters& p,const float* durations=nullptr){
 int n=p.blendSwingModel?int(p.swingHorizon+(p.horizon-p.swingHorizon)*.5f*(1-cosf(a))+.5f):p.horizon;
 float g=p.separateSwingModel?p.swingG:p.g,b=p.separateSwingModel?p.swingB:p.b,d=p.separateSwingModel?p.swingD:p.d;
 if(p.blendSwingModel){float blend=.5f*(1-cosf(a));g+=(p.g-g)*blend;b+=(p.b-b)*blend;d+=(p.d-d)*blend;}
 timedPendulumPrediction(a,w,history,durations,index,size,n,g,b,d,true);
}
