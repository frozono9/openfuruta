#pragma once
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <cstring>
using std::abs;using std::size_t;
#define PI 3.14159265358979323846
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define IRAM_ATTR
#define constrain(x,l,h) ((x)<(l)?(l):((x)>(h)?(h):(x)))
inline uint64_t simulatedMicros=0;inline double sensorPlantAngle=0,shaftPlantAngle=0;
inline uint32_t micros(){return (uint32_t)simulatedMicros;}inline uint32_t millis(){return micros()/1000;}
inline void digitalWrite(int,int){}inline void pinMode(int,int){}
inline int digitalRead(int){return HIGH;}
using portMUX_TYPE=int;
#define portMUX_INITIALIZER_UNLOCKED 0
inline void portENTER_CRITICAL(int*){}inline void portEXIT_CRITICAL(int*){}inline void portENTER_CRITICAL_ISR(int*){}inline void portEXIT_CRITICAL_ISR(int*){}
struct hw_timer_t{};inline hw_timer_t* timerBegin(int,int,bool){static hw_timer_t t;return &t;}inline void timerAttachInterrupt(hw_timer_t*,void(*)(),bool){}inline void timerAlarmWrite(hw_timer_t*,int,bool){}inline void timerAlarmEnable(hw_timer_t*){}
struct SerialMock{size_t setTxBufferSize(size_t n){return n;}void begin(int){}int available(){return 0;}int read(){return 0;}void println(const char*s){fprintf(stderr,"%s\n",s);}void printf(const char* fmt,...){if(strncmp(fmt,"EVENT",5))return;va_list ap;va_start(ap,fmt);vfprintf(stderr,fmt,ap);va_end(ap);}};inline SerialMock Serial;
