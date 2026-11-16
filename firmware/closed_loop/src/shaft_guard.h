#pragma once
#include <math.h>
// Called at encoder telemetry cadence (~20 ms). Large errors cannot wait
// four samples while a 6 rad/s command keeps accumulating unexecuted steps.
inline bool shaftTrackingFault(bool valid,float error,int& consecutive){
 constexpr float moderate=5.f*3.14159265358979323846f/180.f;
 constexpr float severe=12.f*3.14159265358979323846f/180.f;
 if(!valid){consecutive=2;return true;}
 consecutive=fabsf(error)>moderate?consecutive+1:0;
 return fabsf(error)>severe||consecutive>=2;
}
