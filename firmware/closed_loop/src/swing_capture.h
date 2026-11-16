#pragma once

#include <cmath>

// CAD-derived rotation term: see sim/hardware_balance/cad_centrifugal_validation.json.
// e is measured from upright; base velocity changes the pendulum acceleration.
inline float capturePendulumAcceleration(float e,float w,float v,float u,
    float gravity,float coupling,float damping){
 const float se=sinf(e),ce=cosf(e);
 return gravity*se+coupling*ce*u+.97152414f*v*v*se*ce-damping*w;
}

struct CaptureState {
 float e, w, q, v, first, cost;
};

inline bool canCoastToCapture(float e,float w,float q,float v,
    float gravity,float coupling,float damping,float acceleration){
 const float stoppedAt=q+v*fabsf(v)/(2.f*acceleration);
 if(fabsf(stoppedAt)>.22f||fabsf(v)>3.4f||e*w>=0||fabsf(e)>.95f) return false;
 constexpr float dt=.006f;
 for(int i=0;i<65;i++){
  const float u=-copysignf(fminf(acceleration,fabsf(v)/dt),v);
  v+=dt*u;
  w+=dt*capturePendulumAcceleration(e,w,v-.5f*dt*u,u,gravity,coupling,damping);
  e+=dt*w;
  if(fabsf(e)<.14f&&fabsf(w)<2.5f&&fabsf(v)<1.f) return true;
  if(fabsf(e)>1.05f) return false;
 }
 return false;
}

inline float captureApproachAcceleration(float e,float w,float q,float v,
    float gravity,float coupling,float damping,float travel,float speed,float acceleration,float velocityPenalty=0.f,int stages=5,float positionPenalty=0.f) {
 constexpr int beamSize=7, substeps=4;
 if(stages<1)stages=1;if(stages>8)stages=8;
 constexpr float dt=.0125f;
 constexpr float controls[]={-1.f,-.5f,-.18f,0.f,.18f,.5f,1.f};
 CaptureState beam[beamSize]={{e,w,q,v,0,0}}, next[beamSize];
 int size=1;
 for(int stage=0;stage<stages;stage++){
  int nextSize=0;
  for(int i=0;i<size;i++){
   for(float fraction:controls){
    CaptureState s=beam[i];
    const float requested=fraction*acceleration;
    if(stage==0) s.first=requested;
    for(int j=0;j<substeps;j++){
     const float remaining=fmaxf(0.f,travel-fabsf(s.q));
     const float outwardLimit=fminf(speed,sqrtf(2.f*acceleration*remaining));
     const float lower=s.q<0?-outwardLimit:-speed;
     const float upper=s.q>0?outwardLimit:speed;
     const float target=fminf(upper,fmaxf(lower,s.v+dt*requested));
     const float actual=fminf(acceleration,fmaxf(-acceleration,(target-s.v)/dt));
     s.q+=dt*(s.v+.5f*dt*actual);
     s.v+=dt*actual;
     s.w+=dt*capturePendulumAcceleration(s.e,s.w,s.v-.5f*dt*actual,actual,gravity,coupling,damping);
     s.e+=dt*s.w;
    }
    const float travelError=fmaxf(0.f,fabsf(s.q)-travel);
    // LQR terminal value in [e,w,q,v], including coupling cross terms.
    const float x[]={s.e,s.w,s.q,s.v};
    constexpr float P[4][4]={{4.629868,.320535,-.155822,-.125015},{.320535,.024548,-.012505,-.010108},{-.155822,-.012505,.026980,.009632},{-.125015,-.010108,.009632,.007434}};
    float terminal=0;
    for(int j=0;j<4;j++)for(int k=0;k<4;k++)terminal+=x[j]*P[j][k]*x[k];
    s.cost=.05f*beam[i].cost+terminal+velocityPenalty*s.v*s.v+positionPenalty*s.q*s.q+2000.f*travelError*travelError;
    int pos=nextSize;
    while(pos>0&&s.cost<next[pos-1].cost) --pos;
    if(pos==beamSize) continue;
    if(nextSize<beamSize) ++nextSize;
    for(int k=nextSize-1;k>pos;k--) next[k]=next[k-1];
    next[pos]=s;
   }
  }
  size=nextSize;
  for(int i=0;i<size;i++) beam[i]=next[i];
 }
 return beam[0].first;
}
