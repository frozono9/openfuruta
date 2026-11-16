#pragma once
#include <cmath>
inline bool captureDirectionAllowed(float error,float rate,float arm,float velocity){
 return !(std::fabs(error)>.08f&&std::fabs(rate)>.5f&&error*rate>0.f&&std::fabs(arm)>.35f&&arm*velocity>0.f);
}
