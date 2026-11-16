#pragma once
#include <cmath>

// Independent implementation of energy pumping, Astrom & Furuta (2000), eq. 8.
// Logarithmic arm/speed penalties follow the published FurutaShield method.
// Coordinates here: pendulum a=0 hanging; positive base acceleration initially
// drives negative pendulum acceleration. No upstream source code is incorporated.
struct ReferenceSwingParameters {
  float acceleration, positionGain, velocityGain, energy, lead, energyBand;
  ReferenceSwingParameters(float a=12,float q=9,float v=2,float e=2.10f,
      float t=0,float band=.25f):acceleration(a),positionGain(q),velocityGain(v),
      energy(e),lead(t),energyBand(band){}
};
// Optional target scheduling: extra energy at the bottom, target 2 at upright.
// A zero exponent explicitly preserves the original constant-energy controller.
inline float referenceScheduledEnergy(float base,float measuredAngle,float exponent){
 if(exponent<=0.f)return base;
 return 2.f+(base-2.f)*powf(fmaxf(0.f,(1.f+cosf(measuredAngle))*.5f),exponent);
}
inline float referenceSwingAcceleration(float a,float w,float q,float v,
    float gravity,float positionLimit,float velocityLimit,
    const ReferenceSwingParameters& p,int& phaseSign) {
  const float phase=a+p.lead*w;
  const float phaseRate=w-p.lead*gravity*sinf(a);
  const float transfer=phaseRate*cosf(phase);
  if(transfer>.15f)phaseSign=1;
  else if(transfer<-.15f)phaseSign=-1;
  const float energy=.5f*w*w/gravity+1-cosf(a);
  const float error=fmaxf(-1.f,fminf(1.f,(energy-p.energy)/p.energyBand));
  const float pumping=p.acceleration*error*phaseSign;
  // Clamp log arguments: the independent braking envelope is still enforced by
  // velocity(), and the ISR retains the hard position stop.
  const float qr=fminf(.995f,fabsf(q)/positionLimit);
  const float vr=fminf(.995f,fabsf(v)/velocityLimit);
  const float positionBrake=(q>0?1.f:q<0?-1.f:0.f)*p.positionGain*log10f(1-qr);
  const float velocityBrake=(v>0?1.f:v<0?-1.f:0.f)*p.velocityGain*log10f(1-vr);
  return pumping+positionBrake+velocityBrake;
}

// Costanzo, Mazza & Natale (Machines 2025), eqs. 41/43, converted to
// downward-zero coordinates. Friction and centrifugal coupling are cancelled
// before the acceleration is integrated by the existing stepper ramp.
inline float compensatedSwingAcceleration(float a,float w,float q,float v,
    float gain,float center,float damping,float targetEnergy,
    float pendulumDamping,float centrifugal,float epsilon,
    float gravity=130.84f,float coupling=.85f) {
  const float ca=cosf(a);
  const float denominator=copysignf(fmaxf(fabsf(ca),epsilon),ca);
  const float energy=.5f*w*w/gravity+1-ca;
  return gain*(energy-targetEnergy)*w*ca-center*q-damping*v
      -pendulumDamping*w/(coupling*denominator)
      +centrifugal*v*v*sinf(a)/coupling;
}
