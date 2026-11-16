#include "balance_excitation.h"
#include "capture_transient.h"
#include <new>
CaptureTransient* captureTransient=nullptr;
BalanceExcitation* balanceExcitation=nullptr;
#include "capture_direction.h"
#include <stdint.h>
struct FaultBurstSample {uint32_t us;uint16_t raw;uint8_t status,agc;uint16_t magnitude;bool angleOK:1;bool statusOK:1;bool magOK:1;};
FaultBurstSample faultBurst[64];bool faultBurstActive=false;int faultBurstN=0;unsigned long faultBurstAt=0,faultBurstLast=0;
#include <Arduino.h>
#include <stdlib.h>
#include <Wire.h>
#include <Preferences.h>
#include "tmc_uart.h"
#include "state.h"
#include "controller.h"
#include "policy.h"
#include "delayed_lqr.h"
#include "replay.h"
#include "replay_capture.h"
bool captureReplay=false;float captureReplayRate=1;
#include "reference_swingup.h"
#include "robust_balance.h"
#include "causal_rate.h"
CausalRate causalRate;bool causalRateEnabled=false;
#include "trajectory_swing.h"
#include "swing_plan.h"
#include "swing_capture.h"
#include "shaft_guard.h"
// Compared against the same firmware in native simulation. Keep the
// measured-rate feedback and direct balance handoff for this candidate.
constexpr bool UPRIGHT_OBSERVER_ENABLED=false;
constexpr bool CAPTURE_BLEND_ENABLED=false;
constexpr bool CAPTURE_COAST_ENABLED=false;
bool captureAssistEnabled=true;float captureVelocityPenalty=0;int capturePlanStages=5;float captureMinimumEnergy=1.82f;float captureApproachWindow=1.15f;float capturePositionPenalty=0;bool uprightDecayProbe=false;bool brakedCatchArmed=false;int autoProbeDirection=0;bool autoProbeActive=false;
bool plannedSwing=false;
float plannedSwingRate=1, plannedSwingFeedback=.65f;
bool swingShaftGuard=false;
int swingShaftHomeRaw=0,swingShaftFaultSamples=0;
int trajectoryHorizon=23;bool trajectoryPump=false;float trajectoryScale=1,trajectoryFeedback=1,trajectoryEarliest=0;
float powerPhaseLead=0,powerPhaseMix=0;
bool positionEnergyPump=false;float positionEnergyLead=0;float velocityEnergyShape=1;bool velocityEnergyPump=false;bool robustBalanceEnabled=false,robustVelocityPI=false;RobustBalanceParameters robustParameters;float robustTrim=0, robustBias=0;
constexpr int STEP=32,DIR=26,EN=25;constexpr float Q=2*PI/3200.f,SOFT=97.9f*PI/180.f;constexpr int HARD_STEPS=888;
constexpr int SHAFT_SDA=13,SHAFT_SCL=33;
volatile int armHardSteps=HARD_STEPS;
volatile uint16_t commandLeaseTicks=0;
volatile bool commandLeaseFault=false;
float armSoftLimit=SOFT;
float swingArmLimit=1000.f; // Optional pump-only travel reserve; independent hard guards unchanged.
constexpr int SHAFT_TEST_MODE=12, UPRIGHT_PROBE_MODE=13, ENCODER_HOME_MODE=14, AUTO_PROBE_BRAKE_MODE=15;
bool uprightProbePending=false;
float uprightProbeDirection=0;
constexpr int shaftTargets[]={200,-200,200,-200,0};
int shaftLeg=0;
int shaftTestOriginRaw=0;
bool shaftSlow=false;
uint32_t shaftStart=0,shaftPause=0;
int encoderHomeTarget=0;bool encoderHomeRecovery=false;
uint32_t encoderHomeSettled=0;

bool stopNNFault=false,neuralFull=false,neuralCatch=false;float nnHistory[48],nnPrevious=0,nnFiltered=0,nnTarget=0,nnPrevTarget=0;float nnRawHistory[5];int nnRawIndex=0,nnDelayFrames=2;uint32_t nnAt=0;
Preferences prefs;portMUX_TYPE mx=portMUX_INITIALIZER_UNLOCKED;hw_timer_t* timer=nullptr;
volatile bool on=false,highPulse=false,limitFault=false;volatile uint32_t phase=0,inc=0;volatile int requestedDir=1,actualDir=1,count=0;
volatile uint32_t forwardPulses=0,reversePulses=0;volatile int testTarget=0;
int homeStartCount=0;uint32_t homeForwardStart=0,homeReverseStart=0;const char* finishReason="none";
bool velocityPI=false,leadPosition=false,notifiedUpright=false;float vpiP=18,vpiKi=230,vpiBaseP=.015f,vpiBaseD=.012f,vpiAW=5,vpiIntegral=0,vpiBaseVelocity=0,vpiTrim=0,phaseLead=.06f,posEnergyBand=.3f;uint32_t pumpBudgetMs=20000;
bool predictivePump=false,delayedControl=false,replayPump=false;constexpr int ACTUATION_HISTORY=48;float pastAcceleration[ACTUATION_HISTORY]={};float pastDuration[ACTUATION_HISTORY]={};int pastIndex=0,replayIndex=0;
bool positionPump=false;float posA=.544584354f,posW=28.900139f,posZ=.50636f,posE=2.018964f,posSigma=2.222342f;
bool compensatedPump=false;float compDamping=1.35f,compCentrifugal=.26f,compEpsilon=.25f;
float referenceCenterGain=0,referenceCenterCorrection=0;
bool approachModelFixed=false;RobustBalanceParameters approachModel;
bool referencePump=false;ReferenceSwingParameters referenceParameters;int referencePhaseSign=0;float referenceEnergyTaper=0;bool referencePredictorFixed=false;RobustBalanceParameters referencePredictor;
float driveMax=6,driveAcc=80,initialKick=15;
float swingBalanceMax=2.4f,swingBalanceAcc=45.f;
float approachAcceleration=0;
uint32_t approachAt=0;
bool coastingIntoCapture=false;
float uprightEstimate=0,uprightRateEstimate=0;
uint32_t uprightEstimateSince=0;
uint32_t coastUntil=0;bool coastProbeArmed=false;
float pumpGain=12,pumpCenter=3,pumpDamping=1,targetEnergy=2.04;
uint32_t runAt=0,heldAt=0;
uint32_t quietWindowAt=0;float quietOrigin=0,quietMin=0,quietMax=0;
struct FieldSample{uint16_t ms,raw,mag;uint8_t enabled,agc,status;};FieldSample* fieldSamples=nullptr;int fieldN=0;bool fieldEnabled=false;
struct VectorSample{uint16_t ms,raw,mag;int16_t step;uint8_t agc,status;};
VectorSample* vectorSamples=nullptr;bool vectorCapture=false,vectorUprightOnly=false;int vectorN=0;uint32_t vectorStart=0,vectorAt=0;
// Capture the same observation used by the controller without extra bus traffic.
struct MotionSample {
 uint32_t us;
 int16_t step, speedMilliRad;
 uint16_t raw;
 uint8_t mode, status;
 int16_t shaftRaw;
 uint8_t shaftStatus;
};
constexpr size_t MOTION_TRACE_CAPACITY=2200;
MotionSample* motionTrace=nullptr;
size_t motionTraceSize=0;
bool motionTraceArmed=false, motionTraceActive=false, motionTraceCaptureOnly=false;
uint32_t motionTraceStart=0;
float motionTraceZero=0;
float releaseAngle=0;float calOrigin=0,calLast=0;
bool calSine=false;float calAmplitude=.1f,calFrequency=1.f,calDuration=10.f;
constexpr float MANUAL_RELEASE_DISPLACEMENT=PI/240.f; // 0.75 degrees
volatile int mode=0;int raw=0,status=0; // idle, waiting=2, balance=3, return=4, hold=5
float zero=INITIAL_ZERO,angle=0,omega=0,lastAngle=0,speed=0,previousRaw=0;
float kp=TUNED_KP,kd=TUNED_KD,armP=TUNED_ARM_P,armD=TUNED_ARM_D,predict=TUNED_PREDICT,integral=0;
bool centerReady=false;
bool valid=false,previousValid=false,restReady=false,rawReady=false;
uint32_t controlSamples=0,controlMaxUs=0,controlSlowSamples=0;uint64_t controlTotalUs=0;
uint32_t tickAt=0,teleAt=0,modeAt=0,restAt=0,motionAt=0,motionCheckAt=0,stableAt=0;
int zeroSamples=0;float zeroCaptureOrigin=0;double sumSin=0,sumCos=0;char input[150];size_t used=0;
float wrap(float x){return atan2f(sinf(x),cosf(x));}
void beginZeroCapture(){
 zeroCaptureOrigin=raw*(2*PI/4096.f);
 zeroSamples=500;
 sumSin=sumCos=0;
 restReady=false;
 Serial.println("EVENT,GRAVITY_ZERO_BEGIN");
}
float uprightTarget(){return robustBalanceEnabled?zero+PI+robustBias+robustTrim:velocityPI?zero+PI+vpiTrim:(delayedControl?DLQR_UP_RAW:zero+PI);}
float balanceError(){return wrap(raw*(2*PI/4096.f)-uprightTarget());}
void resetNN(){float frame[6]={sinf(angle),cosf(angle),0,0,0,0};for(int i=0;i<48;i++)nnHistory[i]=frame[i%6];nnPrevious=angle;for(int i=0;i<5;i++)nnRawHistory[i]=angle;nnRawIndex=0;nnFiltered=nnTarget=nnPrevTarget=0;nnAt=micros();}
void tickNN(float theta){if(micros()-nnAt<10000)return;if(micros()-nnAt>30000){stopNNFault=true;return;}nnAt+=10000;nnRawHistory[nnRawIndex]=angle;float delayed=nnRawHistory[(nnRawIndex-nnDelayFrames+5)%5];nnRawIndex=(nnRawIndex+1)%5;nnFiltered=.65f*nnFiltered+.35f*wrap(delayed-nnPrevious)/.01f;nnPrevious=delayed;for(int i=0;i<42;i++)nnHistory[i]=nnHistory[i+6];float frame[6]={sinf(delayed),cosf(delayed),nnFiltered/30,theta/(float(PI)/2),speed/8,nnPrevTarget/8};for(int i=0;i<6;i++)nnHistory[42+i]=frame[i];if(neuralFull||(mode==3&&neuralCatch)){float action=actionNN(nnHistory);nnTarget=constrain(speed+.8f*action,-fminf(8.f,driveMax),fminf(8.f,driveMax));nnPrevTarget=nnTarget;}else nnTarget=nnPrevTarget=speed;}
void resetActuationHistory(){for(int i=0;i<ACTUATION_HISTORY;i++){pastAcceleration[i]=0;pastDuration[i]=.002f;}pastIndex=replayIndex=0;}
void startMotionTrace(){
 motionTraceActive=motionTraceArmed;motionTraceArmed=false;
 if(motionTraceActive){motionTraceStart=micros();motionTraceZero=zero;motionTraceSize=0;}
}
float delayedLQR(float error,float w,float q,float v){const int start=(pastIndex+ACTUATION_HISTORY-DLQR_L)%ACTUATION_HISTORY;float corrected=w+.008f*(DLQR_G*error-DLQR_D*w+DLQR_B*pastAcceleration[start]);float u=-DLQR_K[0]*q-DLQR_K[1]*v-DLQR_K[2]*error-DLQR_K[3]*corrected;for(int i=0;i<DLQR_L;i++)u-=DLQR_K[4+i]*pastAcceleration[(start+i)%ACTUATION_HISTORY];return constrain(u,-80.f,80.f);}
void predictPendulum(float& a,float& w,float v){int n=20,start=(pastIndex+ACTUATION_HISTORY-n)%ACTUATION_HISTORY;float input=pastAcceleration[start];w+=.008f*(-144.29f*sinf(a)-1.297f*cosf(a)*input-3.08f*w+.26f*v*v*sinf(a)*cosf(a));for(int i=0;i<n;i++){input=pastAcceleration[(start+i)%ACTUATION_HISTORY];w+=(-144.29f*sinf(a)-1.297f*cosf(a)*input-3.08f*w+.26f*v*v*sinf(a)*cosf(a))*.002f;a+=w*.002f;}a=wrap(a);}
bool robustCapture(float e,float w,float q,float v){
 if(robustVelocityPI&&(!captureTransient||!captureTransient->allowed()))return false;
 if(robustVelocityPI&&!captureDirectionAllowed(e,w,q,v))return false;
 { // Measured-state gate applies to every swing-up policy, including REFERENCE.
  if(fabsf(e)>robustParameters.captureAngle||fabsf(w)>1.25f*robustParameters.captureSpeed) return false;
  const float stoppingPosition=q+v*fabsf(v)/(2.f*fminf(driveAcc,swingBalanceAcc));
  if(fabsf(stoppingPosition)>armSoftLimit-.065f) return false;
 }
 robustPrediction(e,w,pastAcceleration,pastIndex,ACTUATION_HISTORY,robustParameters,pastDuration);
 return fabsf(e)<robustParameters.captureAngle&&fabsf(w)<robustParameters.captureSpeed&&
     fabsf(q)<robustParameters.captureArm&&fabsf(v)<robustParameters.captureMotor;
}
bool predictableCapture(float error,float w,float v){if(abs(error)>.45f||abs(w)>6)return false;float a=error+PI;predictPendulum(a,w,v);return abs(wrap(a-PI))<.30f&&abs(w)<4;}

void resetVPI(float error,float theta){vpiBaseVelocity=speed;float ref=constrain(vpiBaseP*theta+vpiBaseD*vpiBaseVelocity,-.08f,.08f);vpiIntegral=(-speed-vpiP*(error-ref)-0.1f*omega)/vpiKi;}
int stepCount(){portENTER_CRITICAL(&mx);int n=count;portEXIT_CRITICAL(&mx);return n;}
void IRAM_ATTR pulseISR(){portENTER_CRITICAL_ISR(&mx);if(highPulse){digitalWrite(STEP,LOW);highPulse=false;}if(on&&inc){if(commandLeaseTicks==0||--commandLeaseTicks==0){on=false;inc=0;phase=0;commandLeaseFault=true;digitalWrite(EN,HIGH);portEXIT_CRITICAL_ISR(&mx);return;}if((mode==4&&count==0)||((mode==9||mode==SHAFT_TEST_MODE)&&count==testTarget)){inc=0;phase=0;portEXIT_CRITICAL_ISR(&mx);return;}if(requestedDir!=actualDir){actualDir=requestedDir;digitalWrite(DIR,actualDir>0?LOW:HIGH);phase=0;portEXIT_CRITICAL_ISR(&mx);return;}uint32_t old=phase;phase+=inc;if(phase<old){int next=count+actualDir;if(abs(next)>armHardSteps){on=false;inc=0;limitFault=true;digitalWrite(EN,HIGH);}else{digitalWrite(STEP,HIGH);highPulse=true;count=next;if(actualDir>0)forwardPulses++;else reversePulses++;}}}portEXIT_CRITICAL_ISR(&mx);}
void stop(const char* why,bool hold=false){if(balanceExcitation)balanceExcitation->cancel();brakedCatchArmed=false;autoProbeDirection=0;autoProbeActive=false;uprightDecayProbe=false;coastProbeArmed=false;uprightProbePending=false;swingShaftGuard=false;portENTER_CRITICAL(&mx);on=false;inc=0;commandLeaseTicks=0;phase=0;highPulse=false;digitalWrite(STEP,LOW);portEXIT_CRITICAL(&mx);digitalWrite(EN,hold?LOW:HIGH);on=hold;speed=0;if(!hold)centerReady=false;mode=hold?5:0;restReady=false;runAt=heldAt=0;restAt=motionAt=millis();quietWindowAt=0;Serial.printf("EVENT,STOP,%s\n",why);}
void enable(){digitalWrite(EN,LOW);on=true;speed=0;integral=0;modeAt=millis();}
float motionSoftLimit(){return mode==1?fminf(armSoftLimit,swingArmLimit):armSoftLimit;}
void velocity(float target,float dt,float accel=driveAcc,float vmax=driveMax){float q=stepCount()*Q;if(mode==3&&!neuralFull)vmax=fminf(vmax,6.f);target=constrain(target,-vmax,vmax);float brake=sqrtf(2*accel*fmaxf(0,motionSoftLimit()-abs(q)-.015f));if(target*q>0)target=copysignf(fminf(abs(target),brake),target);speed+=constrain(target-speed,-accel*dt,accel*dt);portENTER_CRITICAL(&mx);if(commandLeaseFault){portEXIT_CRITICAL(&mx);return;}commandLeaseTicks=mode==ENCODER_HOME_MODE?500:200;requestedDir=speed>=0?1:-1;inc=(uint32_t)(abs(speed)/Q*(4294967296.0/10000.0));portEXIT_CRITICAL(&mx);}
// Normal completion retains motor torque and the original step reference.
// STOP and sensor/limit faults still disable immediately and invalidate home.
void finishTrial(const char* reason){if(balanceExcitation)balanceExcitation->cancel();
 if(controlSamples)Serial.printf("EVENT,CONTROL_TIMING,n=%lu,mean_us=%lu,max_us=%lu,over4ms=%lu\n",(unsigned long)controlSamples,(unsigned long)(controlTotalUs/controlSamples),(unsigned long)controlMaxUs,(unsigned long)controlSlowSamples);
coastProbeArmed=false;uprightDecayProbe=false;finishReason=reason;runAt=heldAt=0;mode=7;modeAt=millis();Serial.printf("EVENT,TRIAL_FINISH,%s,step=%d,speed=%.5f\n",reason,stepCount(),speed);}
void beginReturn(){
 portENTER_CRITICAL(&mx);homeStartCount=count;homeForwardStart=forwardPulses;homeReverseStart=reversePulses;portEXIT_CRITICAL(&mx);
 mode=4;modeAt=millis();digitalWrite(EN,LOW);on=true;
 Serial.printf("EVENT,HOME_BEGIN,from_step=%d\n",homeStartCount);
}
void completeReturn(){
 int n;uint32_t f,b;portENTER_CRITICAL(&mx);n=count;f=forwardPulses-homeForwardStart;b=reversePulses-homeReverseStart;portEXIT_CRITICAL(&mx);
 if(n!=0||(int32_t)(f-b)!=-homeStartCount){stop("home_step_mismatch");return;}
 stop("centered",true);
 Serial.printf("EVENT,HOME_COMPLETE,step=%d,forward=%lu,reverse=%lu,reason=%s\n",n,(unsigned long)f,(unsigned long)b,finishReason);
}
void velocityBalance(float error,float theta,float dt){if(robustBalanceEnabled)robustTrim=constrain(robustTrim+robustParameters.trimRate*theta*dt,-.04f,.04f);else vpiTrim=constrain(vpiTrim+.0015f*theta*dt,-.03f,.03f);vpiBaseVelocity+=dt/(.2f+dt)*(speed-vpiBaseVelocity);float ref=constrain(vpiBaseP*theta+vpiBaseD*vpiBaseVelocity,-.08f,.08f);float inner=error-ref;vpiIntegral+=inner*dt;float target=-vpiP*inner-vpiKi*vpiIntegral-0.1f*omega;
 if(balanceExcitation){
  target+=balanceExcitation->update(millis(),mode==3,error,omega,theta,speed);
  const int event=balanceExcitation->takeEvent();
  if(event==1){
   if(motionTrace){motionTraceSize=0;motionTraceStart=micros();motionTraceZero=zero;motionTraceActive=true;motionTraceCaptureOnly=false;}
   Serial.printf("EVENT,BALANCE_EXCITE_BEGIN,us=%lu,delay_ms=250,duration_ms=2000,amplitude_each=0.12,hz=3+7\n",(unsigned long)micros());
  }else if(event==2)Serial.println("EVENT,BALANCE_EXCITE_END");
  else if(event==3)Serial.println("EVENT,BALANCE_EXCITE_ABORT");
 }
 velocity(target,dt,robustBalanceEnabled?fminf(driveAcc,swingBalanceAcc):80.f,robustBalanceEnabled?fminf(driveMax,swingBalanceMax):6.f);vpiIntegral+=(target-speed)*vpiAW/vpiKi*dt;vpiIntegral=constrain(vpiIntegral,-.15f,.15f);}
struct I2cReadFailure {
 uint8_t addressRegister, txStatus, expected, received;
};
bool reg(uint8_t a,uint8_t* p,uint8_t n,TwoWire& bus=Wire,I2cReadFailure* failure=nullptr){
 bus.beginTransmission(0x36);
 bus.write(a);
 const auto txStatus=bus.endTransmission(false);
 if(txStatus){
  if(failure)*failure={a,uint8_t(txStatus),n,0};
  return false;
 }
 const auto received=bus.requestFrom((uint8_t)0x36,n);
 if(received!=n){
  if(failure)*failure={a,0,n,uint8_t(received)};
  return false;
 }
 for(int i=0;i<n;i++)p[i]=bus.read();
 return true;
}
bool readShaft(int& shaftRaw,int& shaftStatus){
 uint8_t shaft[3];
 if(!reg(0x0B,shaft,3,Wire1))return false;
 shaftStatus=shaft[0];
 shaftRaw=((shaft[1]&15)<<8)|shaft[2];
 return true;
}
void process(const char* s){
 if(!strcmp(s,"BALANCEEXCITE ON")||!strcmp(s,"BALANCEEXCITE OFF")){
  if(mode!=0||!balanceExcitation||(!strcmp(s,"BALANCEEXCITE ON")&&!robustVelocityPI)){Serial.println("EVENT,REJECT,balance_excite_requires_idle_vpi");return;}
  balanceExcitation->arm(!strcmp(s,"BALANCEEXCITE ON"));Serial.printf("EVENT,BALANCE_EXCITE,%s\n",balanceExcitation->armed?"ON":"OFF");return;
 }

 if(!strcmp(s,"STOP")){coastUntil=0;stop("host");return;}
 if(!strcmp(s,"COAST")){
  if(mode!=1||!swingShaftGuard||fabsf(speed)>1.35f||fabsf(stepCount()*Q)>.24f||
     fabsf(balanceError())>.57f||fabsf(omega)>12.f){
   Serial.println("EVENT,COAST_SKIPPED");
   return;
  }
  coastUntil=millis()+850;
  stop("coast_test",true);
  Serial.println("EVENT,COAST_BEGIN");
  return;
 }
 if(mode!=0&&mode!=5){Serial.println("EVENT,REJECT,busy");return;}
 if(!strcmp(s,"RATEOBSERVER ON")||!strcmp(s,"RATEOBSERVER OFF")){
  if(mode!=0||on){Serial.println("EVENT,REJECT,rate_observer_idle_only");return;}
  causalRateEnabled=!strcmp(s,"RATEOBSERVER ON");causalRate.reset();
  Serial.printf("EVENT,RATE_OBSERVER,%d\n",causalRateEnabled);return;
 }
 if(!strcmp(s,"RATEBENCH")){
  if(mode!=0||on){Serial.println("EVENT,REJECT,rate_bench_idle_only");return;}
  CausalRate probe;double value=0;for(int i=0;i<16;i++)probe.push(i*2000,.002*i+.00001*i*i);
  uint32_t total=0,maximum=0;int validCount=0;
  for(int i=0;i<64;i++){uint32_t begin=micros();double k=i+16;probe.push((i+16)*2000,.002*k+.00001*k*k);validCount+=probe.rate(value);uint32_t elapsed=micros()-begin;total+=elapsed;maximum=elapsed>maximum?elapsed:maximum;}
  Serial.printf("EVENT,RATE_BENCH,max_us=%lu,mean_us=%lu,valid=%d,rate=%.6f\n",(unsigned long)maximum,(unsigned long)(total/64),validCount,float(value));return;
 }
 float centerGain;
 if(sscanf(s,"REFCENTERI %f",&centerGain)==1){
  if(mode!=0||on||!isfinite(centerGain)||centerGain<0||centerGain>15){Serial.println("EVENT,REJECT,center_integral_requires_idle_valid");return;}
  referenceCenterGain=centerGain;referenceCenterCorrection=0;Serial.printf("EVENT,REFERENCE_CENTER_I,%.4f\n",centerGain);return;
 }
 if(!strcmp(s,"BRAKEDCATCH ON")||!strcmp(s,"BRAKEDCATCH OFF")){
  if(mode!=0||on){Serial.println("EVENT,REJECT,braked_catch_idle_only");return;}
  brakedCatchArmed=!strcmp(s,"BRAKEDCATCH ON");autoProbeDirection=brakedCatchArmed?1:0;
  Serial.printf("EVENT,BRAKED_CATCH_ARMED,%d\n",brakedCatchArmed);return;
 }
 if(!strcmp(s,"DECAYPROBE ON")||!strcmp(s,"DECAYPROBE OFF")){
  if(mode!=0||on){Serial.println("EVENT,REJECT,decay_probe_requires_idle");return;}
  uprightDecayProbe=!strcmp(s,"DECAYPROBE ON");Serial.printf("EVENT,DECAY_PROBE,%d\n",uprightDecayProbe);return;
 }
 int automaticDirection;
 if(sscanf(s,"AUTOPROBE %d",&automaticDirection)==1){
  if(mode!=0||on||automaticDirection< -1||automaticDirection>1){Serial.println("EVENT,REJECT,automatic_probe_requires_idle_valid");return;}
  brakedCatchArmed=false;autoProbeDirection=automaticDirection;autoProbeActive=false;Serial.printf("EVENT,AUTOMATIC_PROBE_ARMED,%d\n",automaticDirection);return;
 }
 float swingTravel;
 if(sscanf(s,"SWINGTRAVEL %f",&swingTravel)==1){
  if(mode!=0||on||!isfinite(swingTravel)||swingTravel<20||swingTravel>100){Serial.println("EVENT,REJECT,swing_travel_requires_idle_valid");return;}
  swingArmLimit=swingTravel*PI/180.f;Serial.printf("EVENT,SWING_TRAVEL,%.3f\n",swingTravel);return;
 }
 float capturePosition;
 if(sscanf(s,"CAPTUREQPENALTY %f",&capturePosition)==1){
  if(mode!=0||on||!isfinite(capturePosition)||capturePosition<0||capturePosition>3){Serial.println("EVENT,REJECT,capture_position_requires_idle_valid");return;}
  capturePositionPenalty=capturePosition;Serial.printf("EVENT,CAPTURE_Q_PENALTY,%.5f\n",capturePositionPenalty);return;
 }
 float captureWindow;
 if(sscanf(s,"CAPTUREWINDOW %f",&captureWindow)==1){
  if(mode!=0||on||!isfinite(captureWindow)||captureWindow<.8f||captureWindow>1.9f){Serial.println("EVENT,REJECT,capture_window_requires_idle_valid");return;}
  captureApproachWindow=captureWindow;Serial.printf("EVENT,CAPTURE_WINDOW,%.5f\n",captureApproachWindow);return;
 }
 float captureEnergy;
 if(sscanf(s,"CAPTUREENERGY %f",&captureEnergy)==1){
  if(mode!=0||on||!isfinite(captureEnergy)||captureEnergy<1.8f||captureEnergy>2.3f){Serial.println("EVENT,REJECT,capture_energy_requires_idle_valid");return;}
  captureMinimumEnergy=captureEnergy;Serial.printf("EVENT,CAPTURE_MIN_ENERGY,%.5f\n",captureMinimumEnergy);return;
 }
 int captureStages;
 if(sscanf(s,"CAPTURESTAGES %d",&captureStages)==1){
  if(mode!=0||on||captureStages<5||captureStages>8){Serial.println("EVENT,REJECT,capture_stages_requires_idle_valid");return;}
  capturePlanStages=captureStages;Serial.printf("EVENT,CAPTURE_STAGES,%d\n",capturePlanStages);return;
 }
 float capturePenalty;
 if(sscanf(s,"CAPTUREVPENALTY %f",&capturePenalty)==1){
  if(mode!=0||on||!isfinite(capturePenalty)||capturePenalty<0||capturePenalty>.3f){Serial.println("EVENT,REJECT,capture_penalty_requires_idle_valid");return;}
  captureVelocityPenalty=capturePenalty;Serial.printf("EVENT,CAPTURE_V_PENALTY,%.5f\n",captureVelocityPenalty);return;
 }
 if(!strcmp(s,"CAPTUREASSIST ON")||!strcmp(s,"CAPTUREASSIST OFF")){captureAssistEnabled=!strcmp(s,"CAPTUREASSIST ON");Serial.printf("EVENT,CAPTURE_ASSIST,%d\n",captureAssistEnabled);return;}
 const bool shaftTest = !strcmp(s,"SHAFTTEST") || !strcmp(s,"SHAFTTEST SLOW");
 const bool verifyDriver = !strcmp(s,"TMCVERIFY");
 int encoderTarget;
 const bool recoveryHome=sscanf(s,"RECOVERHOME %d",&encoderTarget)==1;
 const bool encoderHome = recoveryHome||sscanf(s,"ENCODERHOME %d",&encoderTarget)==1;
 const bool startsMotor = shaftTest || !strcmp(s,"FIELDTEST") || !strcmp(s,"CAL") || !strncmp(s,"SINEID ",7) ||
     !strcmp(s,"RUN") || !strcmp(s,"RETURN") || !strncmp(s,"HOMETEST ",9) ||
     !strcmp(s,"ARM") || !strcmp(s,"UPRIGHTPROBE") || encoderHome;
 if(verifyDriver && (mode!=0 || on)){
  Serial.println("EVENT,REJECT,tmc_verify_requires_idle");
  return;
 }
 if(startsMotor&&faultBurstActive){Serial.println("EVENT,REJECT,fault_burst_active");return;}
 if(mode==0 && (startsMotor || verifyDriver)){
  const bool configured=tmc_uart::configure();
  tickAt=micros();
  if(!configured) Serial.println("EVENT,REJECT,tmc_current_unverified");
  if(!configured || verifyDriver) return;
 }
 if(encoderHome){
  int rawPosition, status;
  if(mode!=0||on||(recoveryHome?millis()-restAt<15000:(!restReady||!valid||fabsf(angle)>.12f||fabsf(omega)>.3f))||
     encoderTarget<0||encoderTarget>=4096||!readShaft(rawPosition,status)||(status&0x38)!=0x20){
   Serial.println("EVENT,REJECT,encoder_home_requires_idle_hanging");
   return;
  }
  const float error=wrap((rawPosition-encoderTarget)*(2*PI/4096.f));
  if(fabsf(error)>100*PI/180){Serial.println("EVENT,REJECT,encoder_home_outside_travel");return;}
  portENTER_CRITICAL(&mx);
  count=lroundf(error/Q);
  portEXIT_CRITICAL(&mx);
  encoderHomeTarget=encoderTarget;encoderHomeRecovery=recoveryHome;
  encoderHomeSettled=0;
  mode=ENCODER_HOME_MODE;
  enable();
  Serial.println("EVENT,ENCODER_HOME_BEGIN");
  return;
 }
 if(!strcmp(s,"SHAFTSCAN")){
  if(mode!=0||on){Serial.println("EVENT,REJECT,shaft_scan_requires_idle");return;}
  int devices=0;
  for(uint8_t address=0x08;address<=0x77;address++){
   Wire1.beginTransmission(address);
   if(Wire1.endTransmission(true)==0){
    devices++;
    Serial.printf("EVENT,SHAFT_SCAN_DEVICE,0x%02X\n",static_cast<unsigned>(address));
   }
  }
  Serial.printf("EVENT,SHAFT_SCAN_COMPLETE,%d\n",devices);
  return;
 }
 if(!strcmp(s,"CAPTUREBENCH")){
  if(mode!=0||on){Serial.println("EVENT,REJECT,capture_bench_requires_disabled");return;}
  uint32_t maximum=0,total=0;volatile float result=0;
  for(int i=0;i<24;i++){
   const uint32_t began=micros();
   result=captureApproachAcceleration((i%3-1)*.8f,(i%5-2)*3.f,(i%7-3)*.4f,(i%9-4)*1.1f,
       139.2744f,1.0565f,1.8863f,armSoftLimit-.015f,6.473883f,65.f,captureVelocityPenalty,capturePlanStages,capturePositionPenalty);
   const uint32_t duration=micros()-began;total+=duration;if(duration>maximum)maximum=duration;
  }
  Serial.printf("EVENT,CAPTURE_BENCH,max_us=%lu,mean_us=%lu,result=%.3f\n",(unsigned long)maximum,(unsigned long)(total/24),float(result));return;
 }
 if(!strcmp(s,"MAGNETREAD")){
  if(mode!=0||on){Serial.println("EVENT,REJECT,magnet_read_requires_disabled");return;}
  uint8_t m[3]={},st=0,a[2]={};const bool ok=reg(0x0B,&st,1)&&reg(0x0C,a,2)&&reg(0x1A,m,3);
  Serial.printf("EVENT,MAGNET_READ,ok=%d,status=%u,raw=%u,agc=%u,magnitude=%u\n",ok,st,((a[0]&15)<<8)|a[1],m[0],((m[1]&15)<<8)|m[2]);return;
 }
 if(!strcmp(s,"SENSORRESET")){
  if(mode!=0||on){Serial.println("EVENT,REJECT,sensor_reset_requires_disabled");return;}
  Wire.end();Wire.begin(27,14,100000);Wire.setTimeOut(3);
  valid=previousValid=restReady=rawReady=false;zeroSamples=0;
  restAt=motionAt=millis();quietWindowAt=0;
  Serial.println("EVENT,SENSOR_BUS_RESET");return;
 }
 if(!strcmp(s,"SHAFTREAD")){
  if(mode!=0||on){Serial.println("EVENT,REJECT,shaft_read_requires_idle");return;}
  Wire1.beginTransmission(0x36);
  const int ack=Wire1.endTransmission(true);
  if(ack){
   Serial.printf("EVENT,SHAFT_SENSOR,I2C_ERROR,code=%d,sda=%d,scl=%d\n",
     ack,digitalRead(SHAFT_SDA),digitalRead(SHAFT_SCL));
   return;
  }
  uint8_t angleBytes[2],shaftStatus;
  if(!reg(0x0C,angleBytes,2,Wire1)||!reg(0x0B,&shaftStatus,1,Wire1)){
   Serial.println("EVENT,SHAFT_SENSOR,I2C_ERROR");
   return;
  }
  const int shaftRaw=((angleBytes[0]&15)<<8)|angleBytes[1];
  Serial.printf("EVENT,SHAFT_SENSOR,%d,%.3f,%u\n",shaftRaw,shaftRaw*360.f/4096,static_cast<unsigned>(shaftStatus));
  return;
 }
 if(shaftTest){
  if(mode!=0||on||armHardSteps<200){Serial.println("EVENT,REJECT,shaft_test_requires_idle");return;}
  int shaftStatus=0;
  if(!readShaft(shaftTestOriginRaw,shaftStatus)||(shaftStatus&0x38)!=0x20){
   Serial.println("EVENT,REJECT,shaft_test_requires_encoder");
   return;
  }
  portENTER_CRITICAL(&mx);
  count=0;
  testTarget=shaftTargets[0];
  portEXIT_CRITICAL(&mx);
  centerReady=false;
  shaftLeg=0;
  shaftSlow=!strcmp(s,"SHAFTTEST SLOW");
  shaftStart=shaftPause=millis();
  mode=SHAFT_TEST_MODE;
  enable();
  Serial.println("EVENT,SHAFT_TEST_BEGIN");
  return;
 }
 if(!strcmp(s,"CENTER")){if(mode==0&&valid&&restReady&&abs(angle)<.2f&&abs(omega)<.3f){portENTER_CRITICAL(&mx);count=0;portEXIT_CRITICAL(&mx);centerReady=true;Serial.println("EVENT,CENTER_CONFIRMED");}else Serial.println("EVENT,REJECT,center_hanging_disabled_only");return;}
 if(!strcmp(s,"TRACE ON")||!strcmp(s,"TRACE CAPTURE")){
  if(!motionTrace)motionTrace=static_cast<MotionSample*>(malloc(sizeof(MotionSample)*MOTION_TRACE_CAPACITY));
  motionTraceArmed=motionTrace!=nullptr;
  motionTraceCaptureOnly=!strcmp(s,"TRACE CAPTURE");
  if(motionTraceArmed)vectorCapture=false;
  Serial.println(motionTraceArmed?"EVENT,TRACE_READY":"EVENT,REJECT,trace_allocation");
  return;
 }
 if(!strcmp(s,"TRACEDUMP")){
  if(mode!=0||on){Serial.println("EVENT,REJECT,trace_requires_disabled");return;}
  motionTraceActive=motionTraceArmed=false;
  Serial.printf("EVENT,TRACE_BEGIN,%u,%.7f\n",(unsigned)motionTraceSize,motionTraceZero);
  for(size_t i=0;i<motionTraceSize;i++){
   const MotionSample& row=motionTrace[i];
   Serial.printf("EVENT,TRACE_SAMPLE,%lu,%d,%d,%u,%u,%u,%d,%u\n",
       (unsigned long)row.us,row.step,row.speedMilliRad,row.raw,row.mode,row.status,
       row.shaftRaw,row.shaftStatus);
  }
  Serial.println("EVENT,TRACE_END");
  return;
 }
 int restored;
 if(!strcmp(s,"VECTOR ON")||!strcmp(s,"VECTOR UPRIGHT")){if(mode!=0){Serial.println("EVENT,REJECT,vector_requires_idle");return;}vectorUprightOnly=!strcmp(s,"VECTOR UPRIGHT");if(!vectorSamples)vectorSamples=(VectorSample*)malloc(sizeof(VectorSample)*800);vectorCapture=vectorSamples!=nullptr;if(vectorCapture)motionTraceArmed=false;Serial.println(vectorCapture?"EVENT,VECTOR_READY":"EVENT,REJECT,vector_allocation");return;}
 if(!strcmp(s,"VECTORDUMP")){vectorCapture=false;Serial.printf("EVENT,VECTOR_COUNT,%d\n",vectorN);for(int i=0;i<vectorN;i++){auto &v=vectorSamples[i];Serial.printf("EVENT,VECTOR,%u,%d,%u,%u,%u,%u\n",v.ms,v.step,v.raw,v.agc,v.mag,v.status);}Serial.println("EVENT,VECTOR_END");return;}
 if(!strcmp(s,"FIELDTEST")){if(mode==0&&valid&&restReady&&abs(angle)<.2f){if(!fieldSamples)fieldSamples=(FieldSample*)malloc(sizeof(FieldSample)*400);if(!fieldSamples){Serial.println("EVENT,REJECT,field_buffer_unavailable");return;}fieldN=0;fieldEnabled=false;mode=11;modeAt=millis();on=false;inc=0;speed=0;digitalWrite(EN,HIGH);Serial.println("EVENT,FIELD_BEGIN,no_step_pulses");}else Serial.println("EVENT,REJECT,field_requires_disabled_hanging");return;}
 if(sscanf(s,"NNDELAY %d",&restored)==1){if(restored>=0&&restored<=4){nnDelayFrames=restored;Serial.printf("EVENT,NN_DELAY,%d_ms\n",10*nnDelayFrames);}return;}
 if(sscanf(s,"RESTORECOUNT %d",&restored)==1){if(mode==0&&valid&&abs(angle)<.25f&&abs(omega)<.5f&&abs(restored)<=armHardSteps){portENTER_CRITICAL(&mx);count=restored;portEXIT_CRITICAL(&mx);centerReady=true;Serial.println("EVENT,COUNT_RESTORED");}return;}
 float idA,idF,idT;
 if(sscanf(s,"SINEID %f %f %f",&idA,&idF,&idT)==3){
  const float ww=2*PI*idF;
  const float peakAcc=idA*(ww*ww+PI*ww+PI*PI/8);
  if(!isfinite(idA)||!isfinite(idF)||!isfinite(idT)||idA<.03f||idA>.45f||idF<.5f||idF>2.5f||idT<6||idT>15||peakAcc>60||idA*ww>4||idA>armSoftLimit-.08f||!centerReady||!valid||!restReady||fabsf(angle)>.15f||fabsf(omega)>.4f||stepCount()!=0){Serial.println("EVENT,REJECT,sine_identification_limits");return;}
  int ss=0;if(!readShaft(swingShaftHomeRaw,ss)||(ss&0x38)!=0x20){Serial.println("EVENT,REJECT,sine_requires_shaft");return;}
  calSine=true;calAmplitude=idA;calFrequency=idF;calDuration=idT;calOrigin=calLast=0;swingShaftGuard=true;swingShaftFaultSamples=0;mode=6;enable();startMotionTrace();
  Serial.printf("EVENT,SINE_IDENTIFICATION,%.6f,%.6f,%.3f\n",idA,idF,idT);return;
 }
 if(!strcmp(s,"CAL")){if(centerReady&&valid&&restReady&&abs(angle)<.15f&&abs(omega)<.4f){calSine=false;calOrigin=calLast=stepCount()*Q;mode=6;enable();Serial.println("EVENT,CHIRP_BEGIN,12s");}else Serial.println("EVENT,REJECT,cal_wait_hanging");return;}
 if(!strcmp(s,"RUN")){if(centerReady&&valid&&restReady&&abs(angle)<.18f&&abs(omega)<.5f&&stepCount()==0&&abs(speed)<.0001f){
 int shaftStatus=0;
 if(!readShaft(swingShaftHomeRaw,shaftStatus)||(shaftStatus&0x38)!=0x20){
  Serial.println("EVENT,REJECT,swing_requires_shaft_encoder");return;
 }
 swingShaftGuard=true;
 swingShaftFaultSamples=0;
 mode=1;enable();runAt=millis();controlSamples=controlMaxUs=controlSlowSamples=0;controlTotalUs=0;
 startMotionTrace();
 vectorN=0;vectorStart=vectorAt=runAt;heldAt=0;notifiedUpright=false;omega=0;lastAngle=angle;resetNN();resetActuationHistory();referencePhaseSign=0;referenceCenterCorrection=0;approachAt=0;uprightEstimateSince=0;coastingIntoCapture=false;Serial.printf("EVENT,SWINGUP_BEGIN,%lu_ms\n",(unsigned long)runAt);}else Serial.println("EVENT,REJECT,run_requires_hanging_centered");return;}
 if(!strcmp(s,"POLICY ROBUST")){robustBalanceEnabled=true;robustVelocityPI=false;robustTrim=0;velocityPI=neuralFull=neuralCatch=delayedControl=false;Serial.println("EVENT,POLICY,ROBUST_PREDICTIVE_REAL_ESTIMATE");return;}
 if(!strcmp(s,"POLICY VPI")){robustBalanceEnabled=false;neuralFull=neuralCatch=delayedControl=false;velocityPI=true;Serial.println("EVENT,POLICY,VPI");return;}
 if(!strcmp(s,"POLICY DLQR")){robustBalanceEnabled=false;velocityPI=false;neuralFull=neuralCatch=false;delayedControl=true;Serial.println("EVENT,POLICY,DLQR");return;}
 if(!strncmp(s,"PREDPUMP",8)||!strncmp(s,"REPLAY",6)||!strncmp(s,"COMPENSATED ",12)||!strncmp(s,"REFERENCE ",10)||!strncmp(s,"PLLPHASE ",9)||!strncmp(s,"POSITION ",9)||!strncmp(s,"PUMP ",5))trajectoryPump=false;
 float planRate,planFeedback=.65f;
 if(sscanf(s,"SWINGPLAN %f %f",&planRate,&planFeedback)>=1){
  if(isfinite(planRate)&&planRate>=.94f&&planRate<=1.04f&&
     isfinite(planFeedback)&&planFeedback>=0&&planFeedback<=1.2f){
   plannedSwing=true;
   plannedSwingRate=planRate;
   plannedSwingFeedback=planFeedback;
   Serial.println("EVENT,SWING_PLAN_READY");
  }
  return;
 }
 if(!strncmp(s,"TRAJECTORY",10)||!strncmp(s,"PREDPUMP",8)||!strncmp(s,"REPLAY",6)||
    !strncmp(s,"COMPENSATED ",12)||!strncmp(s,"REFERENCE ",10)||!strncmp(s,"VENERGY ",8)||
    !strncmp(s,"POSITION ",9)||!strncmp(s,"PUMP ",5)||!strncmp(s,"PLLPHASE ",9))plannedSwing=false;
 if(!strcmp(s,"PREDPUMP")){compensatedPump=false;referencePump=false;predictivePump=true;replayPump=positionPump=false;Serial.println("EVENT,PUMP,PREDICTIVE_ENERGY");return;}
 if(!strcmp(s,"REPLAY")){captureReplay=false;compensatedPump=false;referencePump=false;predictivePump=false;replayPump=true;Serial.println("EVENT,PUMP,REPLAY_REAL_TRIAL_01");return;}
 if(!strcmp(s,"POLICY NN")){robustBalanceEnabled=false;velocityPI=false;delayedControl=false;neuralFull=true;neuralCatch=false;Serial.println("EVENT,POLICY,NN");return;}
 if(!strcmp(s,"POLICY HYBRID")){robustBalanceEnabled=false;velocityPI=false;delayedControl=false;neuralFull=false;neuralCatch=true;Serial.println("EVENT,POLICY,HYBRID");return;}
 if(!strcmp(s,"POLICY CLASSIC")){robustBalanceEnabled=false;velocityPI=false;delayedControl=false;neuralFull=neuralCatch=false;Serial.println("EVENT,POLICY,CLASSIC");return;}
 if(!strcmp(s,"BENCH")){uint32_t t=micros();float a=actionNN(nnHistory);Serial.printf("EVENT,NN_BENCH,%lu_us,%.6f\n",(unsigned long)(micros()-t),a);return;}
 float a,b,c,d,e,f,h;
 if(sscanf(s,"REPLAYCAPTURE %f",&a)==1){
  if(isfinite(a)&&a>=.94f&&a<=1.06f){
   captureReplayRate=a;
   captureReplay=replayPump=true;
   trajectoryPump=compensatedPump=referencePump=predictivePump=false;
   Serial.printf("EVENT,REPLAY_CAPTURE,%.5f\n",a);
  }
  return;
 }
 if(sscanf(s,"ARMTRAVEL %f",&a)==1){
  if(!centerReady||stepCount()!=0||!isfinite(a)||a<5||a>100){
   Serial.println("EVENT,REJECT,arm_travel_requires_center");
   return;
  }
  const int steps=(int)floorf(a*PI/180.f/Q+1.e-4f);
  portENTER_CRITICAL(&mx);
  armHardSteps=steps;
  portEXIT_CRITICAL(&mx);
  armSoftLimit=steps*Q-2.f*PI/180.f;
  Serial.printf("EVENT,ARM_TRAVEL,%d,%.4f\n",steps,armSoftLimit*180.f/PI);
  return;
 }
 if(sscanf(s,"SWINGMODEL %f %f %f %d",&a,&b,&c,&restored)==4){if(a>=80&&a<=180&&b>=.3f&&b<=2&&c>=0&&c<=10&&restored>=1&&restored<=30){robustParameters.swingG=a;robustParameters.swingB=b;robustParameters.swingD=c;robustParameters.swingHorizon=restored;robustParameters.separateSwingModel=robustParameters.blendSwingModel=true;Serial.printf("EVENT,SWING_MODEL,%.4f,%.4f,%.4f,%d\n",a,b,c,restored);}return;}
 if(sscanf(s,"CAPTURE %f %f %f %f",&a,&b,&c,&d)==4){if(a>=.05f&&a<=.4f&&b>=.5f&&b<=5&&c>=.1f&&c<=2.25f&&d>=.5f&&d<=5){robustParameters.captureAngle=a;robustParameters.captureSpeed=b;robustParameters.captureArm=c;robustParameters.captureMotor=d;Serial.println("EVENT,CAPTURE_CONFIG");}return;}
 if(sscanf(s,"POWERPHASE %f %f",&a,&b)==2){
  if(isfinite(a)&&a>=0&&a<=.1f&&isfinite(b)&&b>=0&&b<=1.1f){
   powerPhaseLead=a;powerPhaseMix=b;
   Serial.printf("EVENT,POWER_PHASE,%.4f,%.4f\n",a,b);
  }
  return;
 }
 if(sscanf(s,"VENERGY %f %f %f %f %f",&a,&b,&c,&d,&e)==5){if(a>=1&&a<=15&&b>=0&&b<=8&&c>=1.6f&&c<=2.4f&&d>=.2f&&d<=6&&e>=.05f&&e<=1){powerPhaseMix=0;pumpGain=a;pumpCenter=b;targetEnergy=c;velocityEnergyShape=d;compEpsilon=e;velocityEnergyPump=compensatedPump=true;trajectoryPump=referencePump=positionPump=predictivePump=replayPump=false;Serial.printf("EVENT,VELOCITY_ENERGY,%.5f,%.5f,%.5f,%.5f,%.5f\n",a,b,c,d,e);}return;}
 if(sscanf(s,"TRAJECTORY %f %f",&a,&b)==2){if(a>=.05f&&a<=1.f&&b>=0&&b<=3){trajectoryScale=a;trajectoryFeedback=b;trajectoryEarliest=a>=.999f?3.f:0.f;trajectoryPump=true;Serial.printf("EVENT,TRAJECTORY_CONFIG,%.4f,%.4f\n",a,b);}return;}
 if(sscanf(s,"ROBUSTBIAS %f",&a)==1){
  if(isfinite(a)&&abs(a)<=.12f){robustBias=a;robustTrim=0;Serial.printf("EVENT,ROBUST_BIAS,%.6f\n",robustBias);}
  else Serial.println("EVENT,REJECT,robust_bias");
  return;
 }
 if(sscanf(s,"ROBUSTTRIM %f",&a)==1){if(a>=0&&a<=.02f){robustParameters.trimRate=a;Serial.printf("EVENT,ROBUST_TRIM_RATE,%.6f\n",a);}return;}
 if(sscanf(s,"ROBUST %f %f %f %f %f %f %f %d",&a,&b,&c,&d,&e,&f,&h,&restored)==8){if(a>=100&&a<=1200&&b>=5&&b<=100&&c>=0&&c<=30&&d>=0&&d<=30&&e>=80&&e<=180&&f>=.3f&&f<=2&&h>=0&&h<=10&&restored>=1&&restored<=ACTUATION_HISTORY){robustParameters.kp=a;robustParameters.kd=b;robustParameters.armP=c;robustParameters.armD=d;robustParameters.g=e;robustParameters.b=f;robustParameters.d=h;robustParameters.horizon=restored;Serial.printf("EVENT,ROBUST_CONFIG,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%d\n",a,b,c,d,e,f,h,restored);}return;}
 if(sscanf(s,"COMPENSATED %f %f %f %f %f %f %f",&a,&b,&c,&d,&e,&f,&h)==7){if(a>=1&&a<=150&&b>=0&&b<=250&&c>=0&&c<=25&&d>=1.95f&&d<=2.4f&&e>=0&&e<=4&&f>=0&&f<=2&&h>=.1f&&h<=.6f){pumpGain=a;pumpCenter=b;pumpDamping=c;targetEnergy=d;compDamping=e;compCentrifugal=f;compEpsilon=h;compensatedPump=true;positionEnergyPump=velocityEnergyPump=false;referencePump=positionPump=predictivePump=replayPump=leadPosition=false;Serial.println("EVENT,PUMP,COMPENSATED_ENERGY");}return;}
 if(!strcmp(s,"COASTPROBE ON")||!strcmp(s,"COASTPROBE OFF")){coastProbeArmed=!strcmp(s,"COASTPROBE ON");Serial.printf("EVENT,COAST_PROBE_ARMED,%d\n",coastProbeArmed?1:0);return;}
 if(!strcmp(s,"BALANCEVPI ON")||!strcmp(s,"BALANCEVPI OFF")){robustVelocityPI=!strcmp(s,"BALANCEVPI ON");Serial.printf("EVENT,BALANCE_VPI,%d\n",robustVelocityPI?1:0);return;}
 if(!strncmp(s,"APPROACHMODEL ",14)){
  if(mode!=0||on){Serial.println("EVENT,REJECT,approach_model_idle_only");return;}
  if(sscanf(s,"APPROACHMODEL %f %f %f %d",&a,&b,&c,&restored)==4&&isfinite(a)&&isfinite(b)&&isfinite(c)&&a>=80&&a<=180&&b>=.3f&&b<=2&&c>=0&&c<=10&&restored>=1&&restored<=ACTUATION_HISTORY){approachModel=robustParameters;approachModel.g=a;approachModel.b=b;approachModel.d=c;approachModel.horizon=restored;approachModelFixed=true;Serial.println("EVENT,APPROACH_MODEL_FIXED");}else Serial.println("EVENT,REJECT,approach_model_range");return;
 }
 if(sscanf(s,"REFPREDICT %f %f %f %d",&a,&b,&c,&restored)==4){if(isfinite(a)&&isfinite(b)&&isfinite(c)&&a>=80&&a<=180&&b>=.3f&&b<=2&&c>=0&&c<=10&&restored>=1&&restored<=ACTUATION_HISTORY){referencePredictor=robustParameters;referencePredictor.g=a;referencePredictor.b=b;referencePredictor.d=c;referencePredictor.horizon=restored;referencePredictorFixed=true;Serial.println("EVENT,REFERENCE_PREDICTOR_FIXED");}else Serial.println("EVENT,REJECT,reference_predictor_range");return;}
 if(sscanf(s,"REFTAPER %f",&a)==1){if(isfinite(a)&&a>=0&&a<=4){referenceEnergyTaper=a;Serial.printf("EVENT,REFERENCE_TAPER,%.5f\n",a);}else Serial.println("EVENT,REJECT,reference_taper_range");return;}
 if(sscanf(s,"REFERENCE %f %f %f %f %f %f",&a,&b,&c,&d,&e,&f)==6){if(a>=2&&a<=100&&b>=0&&b<=80&&c>=0&&c<=25&&d>=1.95f&&d<=2.4f&&e>=-.06f&&e<=.1f&&f>=.05f&&f<=1){referenceParameters={a,b,c,d,e,f};referenceCenterGain=referenceCenterCorrection=0;referenceEnergyTaper=0;referencePredictorFixed=false;approachModelFixed=false;causalRateEnabled=false;captureVelocityPenalty=0;capturePlanStages=5;captureMinimumEnergy=1.82f;captureApproachWindow=1.15f;capturePositionPenalty=0;swingArmLimit=1000.f;uprightDecayProbe=false;brakedCatchArmed=false;autoProbeDirection=0;autoProbeActive=false;compensatedPump=false;referencePump=true;positionPump=predictivePump=replayPump=leadPosition=false;Serial.printf("EVENT,REFERENCE_PUMP,%.3f,%.3f,%.3f,%.3f,%.4f,%.3f\n",a,b,c,d,e,f);}return;}
 if(sscanf(s,"VPI %f %f %f %f %f",&a,&b,&c,&d,&e)==5){if(a>=1&&a<=30&&b>=50&&b<=400&&c>=0&&c<=.05f&&d>=0&&d<=.05f&&e>=0&&e<=20){vpiP=a;vpiKi=b;vpiBaseP=c;vpiBaseD=d;vpiAW=e;Serial.printf("EVENT,VPI_CONFIG,%.3f,%.3f,%.5f,%.5f,%.3f\n",a,b,c,d,e);}return;}
 if(sscanf(s,"PLLPHASE %f %f",&a,&b)==2){if(a>=-.1f&&a<=.15f&&b>=.05f&&b<=1.5f){compensatedPump=referencePump=false;leadPosition=true;positionPump=true;replayPump=predictivePump=false;phaseLead=a;posEnergyBand=b;Serial.printf("EVENT,PLL_PHASE,%.5f,%.5f\n",a,b);}return;}
 if(sscanf(s,"DURATION %f",&a)==1){if(a>=10&&a<=50){pumpBudgetMs=(uint32_t)(a*1000);Serial.printf("EVENT,PUMP_BUDGET,%lu\n",(unsigned long)pumpBudgetMs);}return;}
 if(sscanf(s,"BALANCELIMITS %f %f",&a,&b)==2){
  if(a>=1&&a<=6&&b>=20&&b<=100){
   swingBalanceMax=a;
   swingBalanceAcc=b;
   Serial.printf("EVENT,BALANCE_LIMITS,%.3f,%.3f\n",a,b);
  }
  return;
 }
 if(sscanf(s,"LIMITS %f %f",&a,&b)==2){if(a>=1&&a<=9&&b>=20&&b<=200){driveMax=a;driveAcc=b;Serial.printf("EVENT,LIMITS_OK,%.3f,%.3f\n",driveMax,driveAcc);}return;}
 if(sscanf(s,"KICK %f",&a)==1){if(a>=-20&&a<=20){initialKick=a;Serial.println("EVENT,KICK_OK");}return;}
 if(sscanf(s,"POSITION %f %f %f %f %f",&a,&b,&c,&d,&e)==5){if(a>=.1f&&a<=1.1f&&b>=4&&b<=30&&c>=.4f&&c<=1.5f&&d>=1.95f&&d<=2.2f&&e>=.1f&&e<=4){compensatedPump=false;referencePump=false;predictivePump=false;replayPump=false;leadPosition=false;posA=a;posW=b;posZ=c;posE=d;posSigma=e;positionPump=true;Serial.println("EVENT,POSITION_PUMP_OK");}return;}
 if(sscanf(s,"PUMP %f %f %f %f",&a,&b,&c,&d)==4){if(a>=1&&a<=150&&b>=0&&b<=250&&c>=0&&c<=25&&d>=1.95&&d<=2.4){compensatedPump=false;referencePump=false;leadPosition=false;predictivePump=false;replayPump=false;positionPump=false;pumpGain=a;pumpCenter=b;pumpDamping=c;targetEnergy=d;Serial.printf("EVENT,PUMP_OK,%.6f,%.6f,%.6f,%.6f\n",pumpGain,pumpCenter,pumpDamping,targetEnergy);}return;}
 if(sscanf(s,"GAINS %f %f %f %f %f",&a,&b,&c,&d,&e)==5){if(a>=150&&a<=1600&&b>=5&&b<=180&&c>=0&&c<=100&&d>=0&&d<=50&&e>=0&&e<=.03){kp=a;kd=b;armP=c;armD=d;predict=e;Serial.println("EVENT,GAINS_OK");}return;}
 if(!strcmp(s,"ZERO")){if(valid&&abs(angle)<.35f&&abs(omega)<.3f)beginZeroCapture();else Serial.println("EVENT,ZERO_REJECTED,hang_still");return;}
 if(sscanf(s,"ZERORECOVER %f",&a)==1){
  if(mode==0&&valid&&isfinite(a)&&abs(a)<=180&&abs(omega)<.3f&&
     abs(wrap(raw*(2*PI/4096.f)-a*PI/180.f))<.05236f)beginZeroCapture();
  else Serial.println("EVENT,ZERO_REJECTED,recovery_reference_mismatch");
  return;
 }
 if(!strcmp(s,"RETURN")){if(centerReady&&valid&&abs(angle)<.25f&&abs(omega)<.5f){finishReason="requested_return";speed=0;beginReturn();}else Serial.println("EVENT,REJECT,return_wait_hanging");return;}
 if(sscanf(s,"HOMETEST %d",&restored)==1){if((mode==0||mode==5)&&centerReady&&valid&&restReady&&abs(angle)<.2f&&abs(omega)<.5f&&stepCount()==0&&abs(restored)>=1&&abs(restored)<=267){testTarget=restored;mode=9;enable();Serial.printf("EVENT,HOME_TEST_BEGIN,target_step=%d\n",restored);}else Serial.println("EVENT,REJECT,home_test_requires_confirmed_center");return;}
 if(!strcmp(s,"ARM")||!strcmp(s,"UPRIGHTPROBE")){
 const bool probe=!strcmp(s,"UPRIGHTPROBE");
 if(probe&&(!robustBalanceEnabled||stepCount()!=0)){
  Serial.println("EVENT,REJECT,probe_requires_robust_at_home");return;
 }
 runAt=0;if(!centerReady||!valid||!restReady){Serial.println("EVENT,REJECT,wait_gravity_zero");return;}uprightProbePending=probe;robustTrim=0;mode=2;enable();resetActuationHistory();modeAt=millis();stableAt=0;Serial.println("EVENT,WAIT_UPRIGHT,place_near_180_then_release_on_engagement");return;}
 Serial.println("EVENT,REJECT,manual_balance_only");
}
void telemetry(){
 int shaftRaw=-1,shaftStatus=0;
 const bool shaftConnected=readShaft(shaftRaw,shaftStatus);
 if(mode==SHAFT_TEST_MODE){
  if(!shaftConnected||(shaftStatus&0x38)!=0x20) stop("shaft_test_encoder_lost");
  else if(fabsf(wrap((shaftRaw-shaftTestOriginRaw)*(2*PI/4096.f)))>28*PI/180)
   stop("shaft_test_travel_limit");
 }
 if(swingShaftGuard){
  const float position=wrap((shaftRaw-swingShaftHomeRaw)*(2*PI/4096.f));
  const bool shaftValid=shaftConnected&&(shaftStatus&0x38)==0x20;
  if(shaftValid&&abs(position)>armHardSteps*Q)stop("shaft_travel_limit");
  else{
   if(shaftTrackingFault(shaftValid,position-stepCount()*Q,swingShaftFaultSamples))
    stop("shaft_tracking_fault");
  }
 }
 Serial.printf("%lu,%d,%.3f,%d,%d,%d,%.6f,%.6f,%d,%.6f,%.6f,%.6f,%.6f,%d,%d,%d,%.6f,%d,%d,%d\n",(unsigned long)millis(),raw,raw*360.f/4096,(status>>5)&1,(status>>4)&1,(status>>3)&1,stepCount()*Q,speed,mode,balanceError(),omega,zero*180/PI,angle*180/PI,valid,restReady,centerReady,wrap(uprightTarget()-zero)*180/PI,stepCount(),shaftRaw,shaftStatus);
}
void setup(){balanceExcitation=new(std::nothrow) BalanceExcitation;captureTransient=new(std::nothrow) CaptureTransient;digitalWrite(EN,HIGH);pinMode(EN,OUTPUT);digitalWrite(STEP,LOW);pinMode(STEP,OUTPUT);digitalWrite(DIR,LOW);pinMode(DIR,OUTPUT);Serial.setTxBufferSize(512);Serial.begin(115200);if(!tmc_uart::begin())Serial.println("EVENT,TMC_UART_INIT_ERROR");initNN();Wire.begin(27,14,100000);Wire.setTimeOut(3);Wire1.begin(SHAFT_SDA,SHAFT_SCL,100000);Wire1.setTimeOut(3);prefs.begin("furuta",false);zero=prefs.getFloat("zero_rad",zero);restAt=motionAt=millis();timer=timerBegin(0,80,true);timerAttachInterrupt(timer,&pulseISR,true);timerAlarmWrite(timer,100,true);timerAlarmEnable(timer);Serial.println("EVENT,BOOT,IDENTIFICATION_AND_CLOSED_LOOP");}
void loop(){
 if(commandLeaseFault){stop("control_command_timeout");portENTER_CRITICAL(&mx);commandLeaseFault=false;portEXIT_CRITICAL(&mx);return;}
 const float referenceEnergyAngle=angle; // Same preceding sensor sample as the validated HIL supervisor.
 while(Serial.available()){char c=Serial.read();if(c=='\n'||c=='\r'){input[used]=0;process(input);used=0;}else if(used<sizeof(input)-1)input[used++]=c;else used=0;}
 if(stopNNFault){stopNNFault=false;stop("neural_deadline");}
 if(limitFault){limitFault=false;stop("hard_arm_limit");}
 if(coastUntil&&int32_t(millis()-coastUntil)>=0){
  coastUntil=0;
  stop("coast_complete");
 }
 if(faultBurstActive&&!on&&mode==0&&uint32_t(micros()-faultBurstLast)>=5000){
  faultBurstLast=micros();uint8_t a[2]={},st=0,m[3]={};
  const bool ao=reg(0x0C,a,2),so=reg(0x0B,&st,1),mo=reg(0x1A,m,3);
  faultBurst[faultBurstN++]={uint32_t(faultBurstLast-faultBurstAt),uint16_t(((a[0]&15)<<8)|a[1]),st,m[0],uint16_t(((m[1]&15)<<8)|m[2]),ao,so,mo};
  if(faultBurstN>=64){faultBurstActive=false;Serial.println("EVENT,FAULT_BURST_BEGIN");
   for(int i=0;i<faultBurstN;i++){const auto &b=faultBurst[i];Serial.printf("EVENT,FAULT_BURST,%lu,%d,%d,%d,%d,%d,%d,%d\n",b.us,b.angleOK,b.statusOK,b.magOK,b.raw,b.status,b.agc,b.magnitude);}
   Serial.println("EVENT,FAULT_BURST_END");
  }
 }
 uint32_t now=micros();if(now-tickAt<2000)return;float dt=tickAt?(now-tickAt)*1e-6f:.002f;tickAt=now;
 if(mode==1||mode==3){uint32_t us=(uint32_t)(dt*1000000.f);controlSamples++;controlTotalUs+=us;controlMaxUs=controlMaxUs>us?controlMaxUs:us;controlSlowSamples+=us>4000;}
 if(mode!=0&&mode!=5&&dt>.02f)stop("control_deadline");
 if(mode==ENCODER_HOME_MODE){
  const uint32_t current=millis();
  if(current-modeAt>(encoderHomeRecovery?20000u:12000u)) stop("encoder_home_timeout");
  else if(current-teleAt>=20){
   int rawPosition,status;
   if(!readShaft(rawPosition,status)||(status&0x38)!=0x20) stop("encoder_home_sensor_lost");
   else{
    const float error=wrap((rawPosition-encoderHomeTarget)*(2*PI/4096.f));
    if(fabsf(error)>100.f*PI/180) stop("encoder_home_travel_limit");
    else if(current-modeAt>=180){
     if(fabsf(error)<.35f*PI/180){
      velocity(0,.020f,encoderHomeRecovery?.5f:3.f,encoderHomeRecovery?.15f:.35f);
      if(!encoderHomeSettled) encoderHomeSettled=current;
      if(current-encoderHomeSettled>=150){
       stop("encoder_home_complete");
       portENTER_CRITICAL(&mx);
       count=0;
       portEXIT_CRITICAL(&mx);
       Serial.printf("EVENT,ENCODER_HOME_COMPLETE,%.3f\n",error*180/PI);
      }
     }else{
      encoderHomeSettled=0;
      const float vmax=encoderHomeRecovery?.15f:.35f;
      velocity(constrain(-2.f*error,-vmax,vmax),.020f,encoderHomeRecovery?.5f:3.f,vmax);
     }
    }
   }
   teleAt=current;
   telemetry();
  }
  return;
 }
 if(mode==SHAFT_TEST_MODE){
  const uint32_t elapsed=millis()-shaftStart;
  const float shaftSpeed=shaftSlow?1.f:4.5f;
  const float shaftAcceleration=shaftSlow?12.f:80.f;
  if(elapsed>16000){stop("shaft_test_timeout");}
  else if(elapsed>=2500){
   const int distance=testTarget-stepCount();
   if(distance!=0){
    const float remaining=distance*Q;
    const float target=copysignf(fminf(shaftSpeed,sqrtf(2.f*shaftAcceleration*fabsf(remaining))),remaining);
    velocity(target,dt,shaftAcceleration,shaftSpeed);
    shaftPause=millis();
   }else{
    velocity(0,dt,shaftAcceleration,shaftSpeed);
    if(millis()-shaftPause>400&&abs(speed)<.001f){
     if(shaftLeg+1==sizeof(shaftTargets)/sizeof(shaftTargets[0])){
      if(millis()-shaftPause>4000){
       stop("shaft_test_complete");
       Serial.println("EVENT,SHAFT_TEST_COMPLETE,step=0");
      }
     }else{testTarget=shaftTargets[++shaftLeg];}
    }
   }
  }
  if(millis()-teleAt>=20){teleAt=millis();telemetry();}
  return;
 }
 const int previousCount=raw;
 uint8_t sample[3];
 I2cReadFailure i2cFailure{};
 const bool bus=reg(0x0B,sample,3,Wire,&i2cFailure);
 if(bus){
  status=sample[0];
  raw=((sample[1]&15)<<8)|sample[2];
 }else status=0;
 // A full I2C read can still contain a corrupt angle with a valid magnet bit.
 const int countChange=((raw-previousCount+6144)%4096)-2048;
 const float angleChange=fabsf(countChange*(2*PI/4096.f));
 const bool implausibleAngle=mode&&previousValid&&bus&&(status&32)&&
     angleChange>70.f*dt+.02f;
 valid=bus&&(status&32)&&!implausibleAngle;
 if(motionTraceActive){
  if(mode==0||(mode==5&&!coastUntil)||motionTraceSize==MOTION_TRACE_CAPACITY)motionTraceActive=false;
  else if(!motionTraceCaptureOnly||coastUntil||mode==3||mode==AUTO_PROBE_BRAKE_MODE||mode==UPRIGHT_PROBE_MODE||(autoProbeActive&&(mode==7||(mode==8&&millis()-modeAt<1000)))||(mode==1&&fabsf(balanceError())<.9f)){
   int traceShaft=-1,traceShaftStatus=0;
   if(!readShaft(traceShaft,traceShaftStatus)){traceShaft=-1;traceShaftStatus=0;}
   motionTrace[motionTraceSize++]={now-motionTraceStart,
      static_cast<int16_t>(stepCount()),static_cast<int16_t>(lroundf(speed*1000)),
      static_cast<uint16_t>(raw),static_cast<uint8_t>(mode),static_cast<uint8_t>(status),
      static_cast<int16_t>(traceShaft),static_cast<uint8_t>(traceShaftStatus)};
  }
 }
 if(!valid){if(captureTransient)captureTransient->reset();previousValid=false;zeroSamples=0;if(mode){
 const int faultMode=mode,faultRaw=raw,faultStatus=status,faultSteps=stepCount();const float faultSpeed=speed;
 stop(implausibleAngle?"pendulum_angle_jump":bus?"magnet_missing":"sensor_I2C"); // Disable the motor before diagnostics or extra bus reads.
 if(!bus)Serial.printf("EVENT,PENDULUM_I2C_DETAIL,register=%u,tx_status=%u,expected=%u,received=%u\n",
     i2cFailure.addressRegister,i2cFailure.txStatus,i2cFailure.expected,i2cFailure.received);
 faultBurstActive=true;faultBurstN=0;faultBurstAt=micros();faultBurstLast=faultBurstAt;
 uint8_t diagnostic[3]={};const bool diagnosticOK=reg(0x1A,diagnostic,3);
 Serial.printf("EVENT,SENSOR_FAULT,us=%lu,bus=%d,status=%d,raw=%d,mode=%d,steps=%d,speed=%.5f,mag_ok=%d,agc=%u,magnitude=%u\n",(unsigned long)now,bus,faultStatus,faultRaw,faultMode,faultSteps,faultSpeed,diagnosticOK,diagnostic[0],((diagnostic[1]&15)<<8)|diagnostic[2]);
 }}
 else{
  if(vectorCapture&&(!vectorUprightOnly||fabsf(wrap(raw*(2*PI/4096.f)-zero-PI))<.35f)&&vectorN<800&&(mode==1||mode==3||mode==7||mode==8)&&millis()-vectorAt>=25){vectorAt=millis();uint8_t z[3];if(reg(0x1A,z,3)){vectorSamples[vectorN++]={(uint16_t)(millis()-vectorStart),(uint16_t)raw,(uint16_t)(((z[1]&15)<<8)|z[2]),(int16_t)stepCount(),z[0],(uint8_t)status};}}
  float absolute=raw*(2*PI/4096.f);angle=wrap(absolute-zero);omega=previousValid?omega+(dt/(.008f+dt))*(wrap(angle-lastAngle)/dt-omega):0;lastAngle=angle;previousValid=true;if(captureTransient)captureTransient->push(now,omega);if(causalRateEnabled)causalRate.push(now,angle);
  if(!rawReady||millis()-motionCheckAt>=100){if(!rawReady||abs(wrap(absolute-previousRaw))>.0087f||abs(omega)>.5f)motionAt=millis();previousRaw=absolute;rawReady=true;motionCheckAt=millis();}
  if(mode==0||mode==5||mode==8){
   // Encoder quantization/noise can create large instantaneous velocity even
   // while hanging. Require a bounded angle window, not a noisy derivative.
   if(!quietWindowAt){quietWindowAt=millis();quietOrigin=absolute;quietMin=quietMax=0;}
   float displacement=wrap(absolute-quietOrigin);quietMin=fminf(quietMin,displacement);quietMax=fmaxf(quietMax,displacement);
   if(quietMax-quietMin>.05236f){quietWindowAt=millis();quietOrigin=absolute;quietMin=quietMax=0;}
   if(mode!=8&&!restReady&&!zeroSamples&&abs(angle)<.35f&&millis()-restAt>=15000&&millis()-quietWindowAt>=2500)beginZeroCapture();
   if(zeroSamples){
    if(abs(wrap(absolute-zeroCaptureOrigin))>.02618f){
     zeroSamples=0;
     quietWindowAt=0;
     Serial.println("EVENT,GRAVITY_ZERO_ABORTED,pendulum_moved");
    }else{
     sumSin+=sin(absolute);sumCos+=cos(absolute);
     if(!--zeroSamples){
      zero=atan2(sumSin,sumCos);prefs.putFloat("zero_rad",zero);
      angle=wrap(absolute-zero);lastAngle=angle;omega=0;vpiTrim=0;restReady=true;
      Serial.printf("EVENT,GRAVITY_ZERO_READY,%.5f\n",zero*180/PI);
     }
    }
   }
  }
  float commandBefore=speed;float error=balanceError(),theta=stepCount()*Q;
  if(autoProbeDirection&&swingShaftGuard&&on&&mode==1&&
     fabsf(error)<.35f&&fabsf(omega)<2.f&&fabsf(speed)<4.f&&
     fabsf(theta+speed*fabsf(speed)/(2.f*driveAcc))<fminf(1.4f,armSoftLimit-.10f)){
   uprightProbeDirection=autoProbeDirection;autoProbeDirection=0;autoProbeActive=true;
   mode=AUTO_PROBE_BRAKE_MODE;modeAt=millis();
   Serial.printf("EVENT,AUTOMATIC_PROBE_BRAKE,%.0f\n",uprightProbeDirection);
  }
  if(mode==AUTO_PROBE_BRAKE_MODE){
   velocity(0,dt,driveAcc,driveMax);
   if(fabsf(speed)<.0001f){
    if(brakedCatchArmed){
     brakedCatchArmed=false;autoProbeActive=false;
     if(fabsf(error)<.20f&&fabsf(omega)<2.5f&&fabsf(theta)<1.4f){mode=3;modeAt=millis();integral=0;heldAt=0;resetVPI(error,theta);Serial.println("EVENT,CATCH_BEGIN,after_braking");}
     else finishTrial("braked_capture_outside_window");
    }else{mode=UPRIGHT_PROBE_MODE;modeAt=millis();Serial.printf("EVENT,AUTOMATIC_PROBE_BEGIN,%.0f\n",uprightProbeDirection);}
   }
   else if(millis()-modeAt>200)finishTrial("automatic_probe_brake_timeout");
  }
  if(uprightDecayProbe&&swingShaftGuard&&on&&(mode==1||mode==3)&&
     fabsf(error)<=.22f&&fabsf(omega)<=2.f&&fabsf(speed)<=1.5f&&
     fabsf(theta+speed*fabsf(speed)/(2.f*driveAcc))<armSoftLimit-.025f){
   // Use the existing brake, energized hold, settle and measured return sequence.
   finishTrial("upright_decay");
  }

  if(coastProbeArmed&&swingShaftGuard&&on&&(mode==1||mode==3)&&
     fabsf(speed)<=.4f&&fabsf(theta)<=1.35f&&fabsf(error)<=.35f&&fabsf(omega)<=12.f){
   coastUntil=millis()+850;
   stop("coast_probe",true);
   swingShaftGuard=true; // Keep the independent shaft/travel checks during hold.
   Serial.println("EVENT,COAST_BEGIN");
  }
  float controlError=error,controlRate=omega;
  const bool observeUpright=UPRIGHT_OBSERVER_ENABLED&&runAt&&robustBalanceEnabled&&powerPhaseMix>0&&
      (mode==1||mode==3)&&fabsf(error)<1.05f;
  if(observeUpright){
   if(!uprightEstimateSince){
    uprightEstimate=error;
    uprightRateEstimate=omega;
    uprightEstimateSince=millis();
   }
   const int lag=constrain(robustParameters.horizon,1,ACTUATION_HISTORY);
   const float motorAcceleration=pastAcceleration[(pastIndex+ACTUATION_HISTORY-lag)%ACTUATION_HISTORY];
   const float pendulumAcceleration=robustParameters.g*sinf(uprightEstimate)+
       robustParameters.b*cosf(uprightEstimate)*motorAcceleration-
       robustParameters.d*uprightRateEstimate;
   uprightRateEstimate+=dt*pendulumAcceleration;
   uprightEstimate+=dt*uprightRateEstimate;
   const float innovation=constrain(wrap(error-uprightEstimate),-.055f,.055f);
   uprightEstimate+=.24f*innovation;
   uprightRateEstimate+=.014f*innovation/dt;
   controlError=uprightEstimate;
   controlRate=uprightRateEstimate;
  }else uprightEstimateSince=0;
  if((mode==1||mode==3)&&(neuralFull||neuralCatch))tickNN(theta);
  if(mode==11){uint32_t elapsed=millis()-modeAt;uint8_t magnetic[3]={};bool magneticValid=reg(0x1A,magnetic,3);if(fieldN<400)fieldSamples[fieldN++]={(uint16_t)elapsed,(uint16_t)raw,(uint16_t)(magneticValid?((magnetic[1]&15)<<8)|magnetic[2]:0),(uint8_t)fieldEnabled,magnetic[0],(uint8_t)status};bool desired=(elapsed>=40&&elapsed<240)||(elapsed>=440&&elapsed<640);if(desired!=fieldEnabled){fieldEnabled=desired;digitalWrite(EN,desired?LOW:HIGH);}if(elapsed>=700){digitalWrite(EN,HIGH);fieldEnabled=false;stop("field_complete");for(int i=0;i<fieldN;i++)Serial.printf("EVENT,FIELD_SAMPLE,%u,%u,%u,%u,%u,%u\n",fieldSamples[i].ms,fieldSamples[i].enabled,fieldSamples[i].raw,fieldSamples[i].agc,fieldSamples[i].mag,fieldSamples[i].status);Serial.println("EVENT,FIELD_END");}}
  if(mode==6&&calSine){float t=(millis()-modeAt)*.001f;
   if(fabsf(angle)>1.65806f)finishTrial("sine_angle_limit");
   else if(t>=calDuration)finishTrial("sine_complete");
   else{float env=1,ed=0;if(t<2){env=.5f*(1-cosf(PI*t/2));ed=PI/4*sinf(PI*t/2);}else if(t>calDuration-2){float tail=calDuration-t;env=.5f*(1-cosf(PI*tail/2));ed=-PI/4*sinf(PI*tail/2);}float phase=2*PI*calFrequency*t;float desired=calAmplitude*env*sinf(phase);float feed=calAmplitude*(ed*sinf(phase)+env*2*PI*calFrequency*cosf(phase));velocity(feed+10*(desired-theta),dt,60,4.5);}
  }
  if(mode==6&&!calSine){float t=(millis()-modeAt)*.001f;if(abs(angle)>.6f)finishTrial("cal_angle_limit");else if(t>=12)finishTrial("cal_complete");else{float f=.5f+5.5f*t/12;float phase=2*PI*(.5f*t+5.5f*t*t/24);float amplitude=fminf(.07f,30/(4*PI*PI*f*f));float envelope=fminf(1,t/.5f)*fminf(1,(12-t)/.5f);float desired=calOrigin+envelope*amplitude*sinf(phase);float feed=(desired-calLast)/dt;calLast=desired;velocity(feed+10*(desired-theta),dt,40,2);}}
  if(mode==7){velocity(0,dt,driveAcc,driveMax);if(abs(speed)<.0001f){mode=8;modeAt=millis();quietWindowAt=0;Serial.printf("EVENT,BRAKE_COMPLETE,step=%d\n",stepCount());}else if(millis()-modeAt>2000)stop("brake_timeout");}
  if(mode==8){if(abs(angle)<.20f&&quietWindowAt&&millis()-quietWindowAt>=2500&&(strcmp(finishReason,"home_check")||millis()-modeAt>=15000)){beginReturn();}else if(millis()-modeAt>35000)stop("home_wait_timeout");}
  if(mode==4){velocity(constrain(-2*theta,-.5f,.5f),dt,3,.5);if(stepCount()==0&&abs(speed)<.01f)completeReturn();else if(millis()-modeAt>15000)stop("return_timeout");}
  if(mode==9){float distance=(testTarget-stepCount())*Q;velocity(constrain(2*distance,-.3f,.3f),dt,3,.3);if(stepCount()==testTarget&&abs(speed)<.01f)finishTrial("home_check");else if(millis()-modeAt>10000)stop("home_test_timeout");}
  if(mode==2){if(millis()-modeAt>=120000)stop("arming_timeout");else if(abs(error)<(robustBalanceEnabled?.0122173f:.034907f)&&abs(omega)<(robustBalanceEnabled?.35f:.8f)){if(!stableAt)stableAt=millis();if(millis()-stableAt>=(robustBalanceEnabled?250u:100u)){if(robustBalanceEnabled){mode=10;modeAt=millis();releaseAngle=angle;startMotionTrace();Serial.println("EVENT,RELEASE_READY,release_now_motor_stationary");}else{mode=3;enable();resetNN();resetVPI(error,theta);Serial.println("EVENT,BALANCE_BEGIN,release");}}}else stableAt=0;}
  if(mode==10){
   if(millis()-modeAt>30000)stop("release_timeout");
   else if(abs(wrap(angle-releaseAngle))>MANUAL_RELEASE_DISPLACEMENT){
    if(abs(error)>.20f)finishTrial("release_outside_capture");
    else if(uprightProbePending){
     uprightProbePending=false;
     uprightProbeDirection=error>0?-1.f:1.f;
     mode=UPRIGHT_PROBE_MODE;
     enable();
     Serial.printf("EVENT,UPRIGHT_PROBE_BEGIN,%.0f\n",uprightProbeDirection);
    }else{
     mode=3;enable();resetNN();resetVPI(error,theta);
     Serial.println("EVENT,BALANCE_BEGIN,release_motion_detected");
    }
   }
  }
  if(mode==UPRIGHT_PROBE_MODE){
   const float elapsed=(millis()-modeAt)*.001f;
   constexpr float pulseSeconds=.16f, peakSpeed=.55f;
   if(abs(error)>(autoProbeActive?1.2f:.22f))finishTrial(autoProbeActive?"automatic_probe_angle_limit":"upright_probe_angle_limit");
   else if(elapsed>=.29f)finishTrial(autoProbeActive?"automatic_probe_complete":"upright_probe_complete");
   else{
    const float target=elapsed<pulseSeconds?
        uprightProbeDirection*peakSpeed*sinf(2*PI*elapsed/pulseSeconds):0;
    velocity(target,dt,25.f,peakSpeed);
   }
  }
  if(mode==1&&!neuralFull){float elapsed=(millis()-runAt)*.001f;float pred=error+predict*omega;if(millis()-runAt>pumpBudgetMs)finishTrial("swingup_timeout");else if((!trajectoryPump||(trajectoryScale>=.999f&&elapsed>=trajectoryEarliest))&&(robustBalanceEnabled?((!observeUpright||millis()-uprightEstimateSince>=24)&&fabsf(error)<.32f&&robustCapture(controlError,controlRate,theta,speed)):delayedControl?predictableCapture(error,omega,speed):(abs(pred)<(velocityPI?.20f:.25f)&&abs(omega)<(velocityPI?2.2f:3)&&(!velocityPI||(abs(theta)<.65f&&abs(speed)<4.5f))))){mode=3;modeAt=millis();integral=0;heldAt=0;resetVPI(error,theta);Serial.println("EVENT,CATCH_BEGIN");}else if(plannedSwing){
 const float sample=plannedSwingRate*elapsed/SWING_PLAN_DT;
 if(sample>=SWING_PLAN_N-1+(.32f/SWING_PLAN_DT))finishTrial("planned_swing_no_capture");
 else{
  const int idx=constrain(int(sample),0,SWING_PLAN_N-2);
  const float blend=constrain(sample-idx,0.f,1.f);
  auto lerp=[&](const float* values){return values[idx]+blend*(values[idx+1]-values[idx]);};
  const float q=lerp(SWING_PLAN_Q),v=plannedSwingRate*lerp(SWING_PLAN_V);
  const float a=lerp(SWING_PLAN_A),w=plannedSwingRate*lerp(SWING_PLAN_W);
  const float feed=sample<SWING_PLAN_N-1?
      plannedSwingRate*plannedSwingRate*(SWING_PLAN_V[idx+1]-SWING_PLAN_V[idx])/SWING_PLAN_DT:0;
  const float errors[]={theta-q,speed-v,wrap(angle-a),omega-w};
  float correction=0;
  for(int i=0;i<4;i++){
   const float gain=SWING_PLAN_K[idx][i]+blend*(SWING_PLAN_K[idx+1][i]-SWING_PLAN_K[idx][i]);
   correction-=gain*errors[i];
  }
  const float tracking=feed+16*(v-speed)+55*(q-theta);
  const float acceleration=tracking+plannedSwingFeedback*constrain(correction,-65.f,65.f);
  velocity(speed+constrain(acceleration,-driveAcc,driveAcc)*dt,dt,driveAcc,driveMax);
 }
}else if(trajectoryPump){
 if(elapsed>=TRAJECTORY_DURATION){finishTrial("trajectory_no_capture");}
 else{int i=constrain(int(elapsed/TRAJECTORY_DT),0,TRAJECTORY_N-2);float f=constrain(elapsed/TRAJECTORY_DT-i,0.f,1.f);
 auto sample=[&](const float* v){return v[i]+f*(v[i+1]-v[i]);};
 float qr=trajectoryScale*sample(TRAJECTORY_Q),vr=trajectoryScale*sample(TRAJECTORY_V);
 if(trajectoryScale<.999f){velocity(vr+12*(qr-theta),dt,driveAcc*trajectoryScale,driveMax*trajectoryScale);}
 else{float pa=angle,pw=omega;RobustBalanceParameters tp=robustParameters;tp.horizon=trajectoryHorizon;tp.g=122.17f;tp.b=1.38f;tp.d=1.18f;robustSwingPrediction(pa,pw,pastAcceleration,pastIndex,ACTUATION_HISTORY,tp,pastDuration);
 float errors[4]={theta-qr,speed-vr,wrap(pa-sample(TRAJECTORY_A)),pw-sample(TRAJECTORY_W)};
 float acc=sample(TRAJECTORY_U);for(int j=0;j<4;j++)acc-=trajectoryFeedback*(TRAJECTORY_K[i][j]+f*(TRAJECTORY_K[i+1][j]-TRAJECTORY_K[i][j]))*errors[j];
 velocity(speed+constrain(acc,-driveAcc,driveAcc)*dt,dt,driveAcc,driveMax);}}
 }else if(compensatedPump){float pa=angle,pw=omega;if(robustBalanceEnabled)robustSwingPrediction(pa,pw,pastAcceleration,pastIndex,ACTUATION_HISTORY,robustParameters,pastDuration);float acc=compensatedSwingAcceleration(pa,pw,theta,speed,pumpGain,pumpCenter,pumpDamping,targetEnergy,compDamping,compCentrifugal,compEpsilon,robustBalanceEnabled?(robustParameters.separateSwingModel?robustParameters.swingG:robustParameters.g):130.84f,robustBalanceEnabled?(robustParameters.separateSwingModel?robustParameters.swingB:robustParameters.b):.85f);if(positionEnergyPump){float energy=.5f*pw*pw/robustParameters.g+1-cosf(pa);float target=-pumpGain*tanhf((targetEnergy-energy)/compEpsilon)*tanhf(velocityEnergyShape*sinf(pa+positionEnergyLead*pw));float acc=pumpCenter*pumpCenter*(target-theta)-2*compDamping*pumpCenter*speed;if(elapsed<.2f)acc=initialKick;velocity(speed+constrain(acc,-driveAcc,driveAcc)*dt,dt,driveAcc,driveMax);}else if(velocityEnergyPump){
 float gravity=robustParameters.g;
 float energy=.5f*pw*pw/gravity+1-cosf(pa), phase=sinf(pa);
 float kickDuration=.2f;
 if(powerPhaseMix>0){
  // Energy transfer is proportional to -cos(angle)*omega*arm_acceleration.
  // Choose arm speed using the phase derivative and anticipate its finite ramp.
  gravity=robustParameters.separateSwingModel?robustParameters.swingG:gravity;
  const float damping=robustParameters.swingD;
  const float gravityTerm=gravity*sinf(angle);
  const float predictedAngle=angle+powerPhaseLead*omega-.5f*powerPhaseLead*powerPhaseLead*gravityTerm;
  const float predictedRate=omega-powerPhaseLead*(gravityTerm+damping*omega);
  energy=.5f*omega*omega/gravity+1-cosf(angle);
  phase=sinf(predictedAngle)*(1+powerPhaseMix*(cosf(predictedAngle)-1+predictedRate*predictedRate/gravity));
  kickDuration=.16f;
 }
 float target=-pumpGain*tanhf((targetEnergy-energy)/compEpsilon)*tanhf(velocityEnergyShape*phase)-pumpCenter*theta;
 if(elapsed<kickDuration)target=initialKick*elapsed;
 if(coastingIntoCapture&&(fabsf(error)>1.f||(error*omega>0&&fabsf(error)>.25f)))
  coastingIntoCapture=false;
 // Start braking before the capture window and account for the remaining arm travel.
 if(captureAssistEnabled&&powerPhaseMix>0&&robustBalanceEnabled&&fabsf(error)<1.15f&&
    ((energy>1.82f&&error*omega<0)||fabsf(error)<.36f)){
  const uint32_t current=millis();
  if(!coastingIntoCapture&&(!approachAt||current-approachAt>=10)){
   float e=controlError,w=controlRate;
   robustPrediction(e,w,pastAcceleration,pastIndex,ACTUATION_HISTORY,robustParameters,pastDuration);
   coastingIntoCapture=CAPTURE_COAST_ENABLED&&canCoastToCapture(e,w,theta,speed,robustParameters.g,
       robustParameters.b,robustParameters.d,driveAcc);
   if(coastingIntoCapture) Serial.println("EVENT,CAPTURE_COAST");
   else approachAcceleration=captureApproachAcceleration(e,w,theta,speed,robustParameters.g,
       robustParameters.b,robustParameters.d,motionSoftLimit()-.015f,driveMax,driveAcc,captureVelocityPenalty,capturePlanStages,capturePositionPenalty);
   approachAt=current;
  }
  target=speed+approachAcceleration*dt;
 }else approachAt=0;
 if(coastingIntoCapture) target=0;
 velocity(target,dt,driveAcc,driveMax);
}else{if(elapsed<.2f)acc=initialKick;velocity(speed+constrain(acc,-driveAcc,driveAcc)*dt,dt,driveAcc,driveMax);}}else if(referencePump){
 float pa=angle,pw=omega;
 if(robustBalanceEnabled)robustSwingPrediction(pa,pw,pastAcceleration,pastIndex,ACTUATION_HISTORY,referencePredictorFixed?referencePredictor:robustParameters,pastDuration);
 ReferenceSwingParameters scheduled=referenceParameters;
 scheduled.energy=referenceScheduledEnergy(referenceParameters.energy,referenceEnergyAngle,referenceEnergyTaper);
 float acc=referenceSwingAcceleration(pa,pw,theta,speed,130.84f,motionSoftLimit(),driveMax,scheduled,referencePhaseSign);
 // Remove persistent one-sided pump orbits with a bounded, slow centering bias.
 referenceCenterCorrection=constrain(referenceCenterCorrection+referenceCenterGain*theta*dt,-20.f,20.f);
 acc-=referenceCenterCorrection;
 const RobustBalanceParameters& approach=approachModelFixed?approachModel:robustParameters;
 const float energy=.5f*omega*omega/approach.g+1-cosf(angle);
 if(captureAssistEnabled&&robustBalanceEnabled&&fabsf(error)<captureApproachWindow&&
    ((energy>captureMinimumEnergy&&error*omega<0)||fabsf(error)<.36f)){
  const uint32_t current=millis();
  if(!approachAt||current-approachAt>=10){
   float e=error,w=omega;
   robustPrediction(e,w,pastAcceleration,pastIndex,ACTUATION_HISTORY,approach,pastDuration);
   approachAcceleration=captureApproachAcceleration(e,w,theta,speed,approach.g,
       approach.b,approach.d,motionSoftLimit()-.015f,driveMax,driveAcc,captureVelocityPenalty,capturePlanStages,capturePositionPenalty);
   approachAt=current;
  }
  acc=approachAcceleration;
 }else approachAt=0;
 if(elapsed<.2f)acc=initialKick;
 velocity(speed+constrain(acc,-driveAcc,driveAcc)*dt,dt,driveAcc,driveMax);
}else if(replayPump&&captureReplay){
   const float playbackTime=elapsed*captureReplayRate;
   if(playbackTime>CAPTURE_REPLAY_T[CAPTURE_REPLAY_N-1]+.14f)finishTrial("capture_replay_no_capture");
   else if(playbackTime>=CAPTURE_REPLAY_T[CAPTURE_REPLAY_N-1])velocity(0,dt,driveAcc,driveMax);
   else{
    while(replayIndex<CAPTURE_REPLAY_N-2&&CAPTURE_REPLAY_T[replayIndex+1]<playbackTime)replayIndex++;
    const float fraction=(playbackTime-CAPTURE_REPLAY_T[replayIndex])/(CAPTURE_REPLAY_T[replayIndex+1]-CAPTURE_REPLAY_T[replayIndex]);
    const float q=CAPTURE_REPLAY_Q[replayIndex]+fraction*(CAPTURE_REPLAY_Q[replayIndex+1]-CAPTURE_REPLAY_Q[replayIndex]);
    const float v=CAPTURE_REPLAY_V[replayIndex]+fraction*(CAPTURE_REPLAY_V[replayIndex+1]-CAPTURE_REPLAY_V[replayIndex]);
    velocity(v*captureReplayRate+8*(q-theta),dt,driveAcc,driveMax);
   }
  }else if(replayPump){if(elapsed>=REPLAY_T[REPLAY_N-1])finishTrial("replay_complete_no_capture");else{while(replayIndex<REPLAY_N-2&&REPLAY_T[replayIndex+1]<elapsed)replayIndex++;float fraction=(elapsed-REPLAY_T[replayIndex])/(REPLAY_T[replayIndex+1]-REPLAY_T[replayIndex]);float target=REPLAY_V[replayIndex]+fraction*(REPLAY_V[replayIndex+1]-REPLAY_V[replayIndex]);velocity(target,dt,80,6);}}else{float phaseAngle=angle,phaseOmega=omega;if(predictivePump)predictPendulum(phaseAngle,phaseOmega,speed);float energy=.5f*phaseOmega*phaseOmega/(predictivePump?144.29f:130.84f)+1-cosf(phaseAngle);float phase=angle+phaseLead*omega-.5f*130.84f*sinf(angle)*phaseLead*phaseLead;float leadOmega=omega-130.84f*sinf(angle)*phaseLead;float phaseSignal=leadPosition?leadOmega*cosf(phase):omega;float scale=leadPosition?constrain((posE-energy)/posEnergyBand,-1.f,1.f):constrain(posE-energy,0.f,1.f);float targetq=posA*scale*tanhf(phaseSignal/posSigma);float acc=positionPump?(posW*posW*(targetq-theta)-2*posZ*posW*speed):(-pumpGain*(targetEnergy-energy)*phaseOmega*cosf(phaseAngle)-pumpCenter*theta-pumpDamping*speed);if(elapsed<.2f)acc=initialKick;velocity(speed+constrain(acc,-driveAcc,driveAcc)*dt,dt,driveAcc,(positionPump&&!leadPosition)?fminf(4,driveMax):driveMax);}}
  if(mode==3&&!neuralFull){if(runAt&&abs(error)>.55f){if(plannedSwing)finishTrial("planned_swing_lost_capture");else{if(balanceExcitation){bool wasRunning=balanceExcitation->running;balanceExcitation->cancel();balanceExcitation->takeEvent();if(wasRunning)Serial.println("EVENT,BALANCE_EXCITE_ABORT,lost_capture");}mode=1;heldAt=0;integral=0;Serial.println("EVENT,LOST_CAPTURE");}}else if(!runAt&&abs(error)>.30f)finishTrial("pendulum_fallen");else if(runAt&&millis()-runAt>=90000)finishTrial("run_90s_complete");else if(!runAt&&millis()-modeAt>=10000)finishTrial("trial_10s_complete");else{if(robustBalanceEnabled&&!robustVelocityPI){robustTrim=constrain(robustTrim+robustParameters.trimRate*theta*dt,-.04f,.04f);float e=controlError,w=controlRate;RobustBalanceParameters bp=robustParameters;double measuredRate=0;if(causalRateEnabled&&causalRate.rate(measuredRate)&&fabs(measuredRate)<50){w=measuredRate;bp.rateFilterLag=0;}robustPrediction(e,w,pastAcceleration,pastIndex,ACTUATION_HISTORY,bp,pastDuration);float u=robustParameters.armP*theta+robustParameters.armD*speed-robustParameters.kp*e-robustParameters.kd*w;
 const float accelLimit=runAt?fminf(driveAcc,swingBalanceAcc):driveAcc;
 const float speedLimit=runAt?fminf(driveMax,swingBalanceMax):driveMax;
 if(CAPTURE_BLEND_ENABLED){
  const uint32_t current=millis();
  if(!approachAt||current-approachAt>=10){
   approachAcceleration=captureApproachAcceleration(e,w,theta,speed,robustParameters.g,
       robustParameters.b,robustParameters.d,armSoftLimit-.015f,speedLimit,accelLimit);
   approachAt=current;
  }
  const float blend=1.f-(millis()-modeAt)/500.f;
  u+=blend*(approachAcceleration-u);
 }
 velocity(speed+constrain(u,-accelLimit,accelLimit)*dt,dt,accelLimit,speedLimit);}else if(velocityPI||(robustBalanceEnabled&&robustVelocityPI))velocityBalance(error,theta,dt);else if(neuralCatch)velocity(nnTarget,dt,fminf(80.f,driveAcc),fminf(8.f,driveMax));else{float acceleration=delayedControl?delayedLQR(error,omega,theta,speed):balanceAcceleration(error,omega,theta,speed,dt,integral,kp,kd,armP,armD,predict);velocity(speed+constrain(acceleration,-80.f,80.f)*dt,dt);}if(runAt&&abs(error)<.15f&&abs(omega)<1.5f){if(!heldAt)heldAt=millis();if(millis()-heldAt>=30000)finishTrial("success_30s_upright");else if(millis()-heldAt>=5000&&!notifiedUpright){notifiedUpright=true;Serial.println("EVENT,SUCCESS,5s_continuous_upright");}}else heldAt=0;}}
  if(neuralFull&&runAt&&(mode==1||mode==3)){if(millis()-runAt>=80000)finishTrial("neural_80s_timeout");else{mode=(abs(error)<.35f&&abs(omega)<4)?3:1;velocity(nnTarget,dt,fminf(80.f,driveAcc),fminf(8.f,driveMax));if(abs(error)<.15f&&abs(omega)<1.5f){if(!heldAt)heldAt=millis();if(millis()-heldAt>=30000)finishTrial("success_30s_upright");else if(millis()-heldAt>=5000&&!notifiedUpright){notifiedUpright=true;Serial.println("EVENT,SUCCESS,5s_continuous_upright");}}else heldAt=0;}}

  if(mode==1||mode==3||mode==AUTO_PROBE_BRAKE_MODE){pastAcceleration[pastIndex]=(speed-commandBefore)/dt;pastDuration[pastIndex]=dt;pastIndex=(pastIndex+1)%ACTUATION_HISTORY;}

 }
 if(millis()-teleAt>=20){teleAt=millis();telemetry();}
}
