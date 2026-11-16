#include "Arduino.h"
#include <cstdlib>
#include <algorithm>
using std::isfinite;
#include "main.cpp"
// Actual firmware loop/ISR against an approximate plant identified from trials.
// This tests algorithms, not the physical stepper's loss of synchronism or slip.
int main(int argc,char**argv){
 if(argc!=14 && argc!=19 && argc!=20)return 2;
 double p=atof(argv[1]),ki=atof(argv[2]),gain=atof(argv[3]),center=atof(argv[4]),cd=atof(argv[5]),energy=atof(argv[6]);
 int lag=atoi(argv[7]),sensorLag=atoi(argv[8]);double g=atof(argv[9]),b=atof(argv[10]),d=atof(argv[11]),bias=atof(argv[12]);bool upright=atoi(argv[13]);
 double a=upright?PI+(getenv("INITIAL_ANGLE")?atof(getenv("INITIAL_ANGLE")):.025):0,w=getenv("INITIAL_OMEGA")?atof(getenv("INITIAL_OMEGA")):0,v=0,uh[4096]={},vh[4096]={},qh[4096]={},ah[4096];
 std::fill(ah,ah+4096,a);simulatedMicros=1000;setup();zero=4.1+(getenv("CAL_BIAS")?atof(getenv("CAL_BIAS")):0);angle=lastAngle=wrap(4.1+a+bias-zero+(upright&&getenv("UPRIGHT_BIAS")?atof(getenv("UPRIGHT_BIAS")):0));omega=0;previousValid=valid=centerReady=restReady=true;raw=((lround((angle+zero)*4096/(2*PI))%4096)+4096)%4096;
 compensatedPump=velocityPI=true;vpiP=p;vpiKi=ki;pumpGain=gain;pumpCenter=center;pumpDamping=1;compDamping=cd;compCentrifugal=0;compEpsilon=.25;targetEnergy=energy;
 if(argc>=19){velocityPI=false;kp=atof(argv[14]);kd=atof(argv[15]);armP=atof(argv[16]);armD=atof(argv[17]);predict=atof(argv[18]);}
 if(argc==20){robustBalanceEnabled=true;robustParameters.kp=kp;robustParameters.kd=kd;robustParameters.armP=armP;robustParameters.armD=armD;robustParameters.horizon=atoi(argv[19]);if(getenv("TRIM_RATE"))robustParameters.trimRate=atof(getenv("TRIM_RATE"));if(getenv("MODEL"))sscanf(getenv("MODEL"),"%f %f %f",&robustParameters.g,&robustParameters.b,&robustParameters.d);if(getenv("SWING_MODEL")){robustParameters.separateSwingModel=true;sscanf(getenv("SWING_MODEL"),"%f %f %f",&robustParameters.swingG,&robustParameters.swingB,&robustParameters.swingD);}if(getenv("CAPTURE_ANGLE"))robustParameters.captureAngle=atof(getenv("CAPTURE_ANGLE"));if(getenv("CAPTURE_SPEED"))robustParameters.captureSpeed=atof(getenv("CAPTURE_SPEED"));}
 if(getenv("DRIVE_MAX"))driveMax=atof(getenv("DRIVE_MAX"));if(getenv("DRIVE_ACC"))driveAcc=atof(getenv("DRIVE_ACC"));
 if(const char* plan=getenv("SWING_PLAN")){plannedSwing=true;plannedSwingRate=atof(plan);}
 positionEnergyPump=getenv("POSITION_ENERGY")!=nullptr;if(getenv("POSITION_LEAD"))positionEnergyLead=atof(getenv("POSITION_LEAD"));
 velocityEnergyPump=getenv("VELOCITY_ENERGY")!=nullptr;if(getenv("VELOCITY_SHAPE"))velocityEnergyShape=atof(getenv("VELOCITY_SHAPE"));if(getenv("ENERGY_BAND"))compEpsilon=atof(getenv("ENERGY_BAND"));
 if(const char* phase=getenv("POWER_PHASE"))sscanf(phase,"%f %f",&powerPhaseLead,&powerPhaseMix);
 if(const char* kick=getenv("INITIAL_KICK"))initialKick=atof(kick);
 if(getenv("CLASSIC_PUMP"))compensatedPump=false;
 if(getenv("REFERENCE_PUMP")){compensatedPump=false;referencePump=true;referenceParameters={pumpGain,pumpCenter,getenv("REFERENCE_V")?(float)atof(getenv("REFERENCE_V")):2.f,targetEnergy,getenv("REFERENCE_LEAD")?(float)atof(getenv("REFERENCE_LEAD")):0.f,compEpsilon};}
 if(getenv("REFERENCE_CENTER_I"))referenceCenterGain=atof(getenv("REFERENCE_CENTER_I"));
 if(getenv("REFERENCE_TAPER"))referenceEnergyTaper=atof(getenv("REFERENCE_TAPER"));
 // Match the live runner's SWINGMODEL g b d 7 command, including blending.
 if(getenv("SWING_MODEL")){robustParameters.swingHorizon=7;robustParameters.blendSwingModel=true;}
 if(getenv("SWING_HORIZON"))robustParameters.swingHorizon=atoi(getenv("SWING_HORIZON"));
 if(getenv("BLEND_PREDICTOR"))robustParameters.blendSwingModel=atoi(getenv("BLEND_PREDICTOR"))!=0;
 if(getenv("CAPTURE_ARM"))robustParameters.captureArm=atof(getenv("CAPTURE_ARM"));if(getenv("CAPTURE_MOTOR"))robustParameters.captureMotor=atof(getenv("CAPTURE_MOTOR"));if(getenv("TRAJECTORY_EARLIEST"))trajectoryEarliest=atof(getenv("TRAJECTORY_EARLIEST"));
 if(getenv("TRAJECTORY_HORIZON"))trajectoryHorizon=atoi(getenv("TRAJECTORY_HORIZON"));
 if(getenv("TRAJECTORY")){trajectoryPump=true;trajectoryScale=atof(getenv("TRAJECTORY"));trajectoryFeedback=getenv("TRAJECTORY_FEEDBACK")?atof(getenv("TRAJECTORY_FEEDBACK")):1;}
 if(const char* replayRate=getenv("CAPTURE_REPLAY_RATE")){
  char command[48];snprintf(command,sizeof(command),"REPLAYCAPTURE %s",replayRate);process(command);
 }
 if(const char* travel=getenv("ARM_TRAVEL")){
  if(!std::isfinite(atof(travel))||atof(travel)<5||atof(travel)>100)return 2;
  char command[48];snprintf(command,sizeof(command),"ARMTRAVEL %s",travel);process(command);
 }
 if(const char* initialBias=getenv("CONTROLLER_UPRIGHT_BIAS"))robustBias=atof(initialBias);
 if(const char* limits=getenv("BALANCE_LIMITS"))sscanf(limits,"%f %f",&swingBalanceMax,&swingBalanceAcc);
 if(getenv("CAPTURE_ASSIST"))captureAssistEnabled=atoi(getenv("CAPTURE_ASSIST"));
 if(getenv("REFERENCE_PREDICTOR")){char command[100];snprintf(command,sizeof(command),"REFPREDICT %s",getenv("REFERENCE_PREDICTOR"));process(command);}
 if(getenv("BALANCE_VPI")){robustVelocityPI=atoi(getenv("BALANCE_VPI"));if(getenv("VPI_PARAMETERS"))sscanf(getenv("VPI_PARAMETERS"),"%f %f %f %f %f",&vpiP,&vpiKi,&vpiBaseP,&vpiBaseD,&vpiAW);}
 if(getenv("COMPENSATED_PARAMS")){char cmd[128];snprintf(cmd,sizeof(cmd),"COMPENSATED %s",getenv("COMPENSATED_PARAMS"));process(cmd);}
 if(getenv("CAPTURE_WINDOW")){char c[80];snprintf(c,sizeof(c),"CAPTUREWINDOW %s",getenv("CAPTURE_WINDOW"));process(c);}
 if(getenv("CAPTURE_QPENALTY")){char c[80];snprintf(c,sizeof(c),"CAPTUREQPENALTY %s",getenv("CAPTURE_QPENALTY"));process(c);}
 if(getenv("DECAY_PROBE")){process("DECAYPROBE ON");swingShaftGuard=true;swingShaftHomeRaw=0;swingShaftFaultSamples=0;}
 if(getenv("SWING_TRAVEL")){char c[80];snprintf(c,sizeof(c),"SWINGTRAVEL %s",getenv("SWING_TRAVEL"));process(c);}
 if(getenv("AUTO_PROBE")){char c[80];snprintf(c,sizeof(c),"AUTOPROBE %s",getenv("AUTO_PROBE"));process(c);swingShaftGuard=true;swingShaftHomeRaw=0;}
 if(getenv("APPROACH_MODEL")){char c[100];snprintf(c,sizeof(c),"APPROACHMODEL %s",getenv("APPROACH_MODEL"));process(c);}
 if(getenv("BRAKED_CATCH")){process("BRAKEDCATCH ON");swingShaftGuard=true;swingShaftHomeRaw=0;}
 if(getenv("CAUSAL_RATE"))process("RATEOBSERVER ON");
 if(getenv("BALANCE_EXCITE"))process("BALANCEEXCITE ON");
 double initialBase=upright&&getenv("INITIAL_BASE")?atof(getenv("INITIAL_BASE")):0;
 double initialBaseSpeed=upright&&getenv("INITIAL_BASE_SPEED")?atof(getenv("INITIAL_BASE_SPEED")):0;
 count=lround(initialBase/Q);
 if(abs(count)>armHardSteps)return 2;
 initialBase=count*Q;
 // Preload the motor delay line with the constant velocity preceding capture.
 int initialLag=lag;
 if(const char* up=getenv("UP_MODEL")){
  double unusedG,unusedB,unusedD,upLag;
  if(sscanf(up,"%lf %lf %lf %lf",&unusedG,&unusedB,&unusedD,&upLag)==4)
   initialLag=static_cast<int>(lround(upLag));
 }
 for(int i=0;i<4096;i++)qh[i]=initialBase+(i-4096)*.0001*initialBaseSpeed;
 if(getenv("CAPTURE_MIN_ENERGY"))captureMinimumEnergy=atof(getenv("CAPTURE_MIN_ENERGY"));
 if(getenv("CAPTURE_STAGES"))capturePlanStages=atoi(getenv("CAPTURE_STAGES"));
 if(getenv("CAPTURE_VPENALTY"))captureVelocityPenalty=atof(getenv("CAPTURE_VPENALTY"));
 pumpBudgetMs=getenv("PUMP_BUDGET")?atoi(getenv("PUMP_BUDGET")):(robustBalanceEnabled?50000:25000);mode=upright?3:1;enable();speed=initialBaseSpeed;v=initialBaseSpeed;runAt=getenv("MANUAL_TEST")?0:millis();if(upright){resetVPI(wrap(a+bias-PI),initialBase);omega=w;commandLeaseTicks=200;requestedDir=initialBaseSpeed>=0?1:-1;inc=(uint32_t)(abs(speed)/Q*(4294967296.0/10000.0));}
 if((getenv("AUTO_PROBE")||getenv("BRAKED_CATCH"))&&upright)mode=1;
 double ug=g,ub=b,ud=d,ulag=lag;if(getenv("UP_MODEL"))sscanf(getenv("UP_MODEL"),"%lf %lf %lf %lf",&ug,&ub,&ud,&ulag);
 double coulomb=getenv("COULOMB")?atof(getenv("COULOMB")):0;double physicalQ=initialBase-initialBaseSpeed*(initialLag*.0001+.005),maxPhysicalQ=abs(physicalQ);double positionBias=getenv("POSITION_BIAS")?atof(getenv("POSITION_BIAS")):0,upBias=getenv("UPRIGHT_BIAS")?atof(getenv("UPRIGHT_BIAS")):0;
 double angleHeld=0,bestAngleHeld=0,errorSquared=0,errorSamples=0;double best=0,held=0,maxq=0,closest=PI;int catches=0,lastmode=mode;bool success=false;
 int nextControlTick=0,controlPatternIndex=0;
 const bool jitterControl=getenv("CONTROL_JITTER")!=nullptr;
 const int controlIntervals[]={20,20,43,20,20,24};
 FILE* trace=getenv("OUTPUT_TRACE")?fopen(getenv("OUTPUT_TRACE"),"w"):nullptr;
 const double uprightFrictionRatio=getenv("UP_FRICTION_RATIO")?atof(getenv("UP_FRICTION_RATIO")):1.;
 const int couplingDelay=getenv("COUPLING_DELAY_MS")?atoi(getenv("COUPLING_DELAY_MS"))*10:0;
 double accelerationHistory[4096]={};
 for(int tick=0;tick<1500000;tick++){
  int ix=tick%4096;qh[ix]=stepCount()*Q;
  double blend=.5*(1-cos(a)),gg=g+(ug-g)*blend,bb=b+(ub-b)*blend,dd=d+(ud-d)*blend;
  int aj=(ix-(int)round(lag+(ulag-lag)*blend)+4096)%4096;
  // Actual emitted STEP positions drive a critically damped motor response.
  // Its low-frequency delay is 5 ms, matching the previous motor-lag setting.
  double motorAcceleration=std::clamp(160000*(qh[aj]-physicalQ)-800*v,-2000.,2000.);
  v+=motorAcceleration*.0001;physicalQ+=v*.0001;maxPhysicalQ=fmax(maxPhysicalQ,fabs(physicalQ));
  accelerationHistory[ix]=motorAcceleration;
  const int delay=(int)round(couplingDelay*blend);
  const double coupledAcceleration=accelerationHistory[(ix+4096-delay)%4096];
  w+=(-gg*sin(a)-bb*cos(a)*coupledAcceleration-dd*w-coulomb*(1.-blend*(1.-uprightFrictionRatio))*tanh(w/.3)+.971524*v*v*sin(a)*cos(a))*.0001;
  a+=w*.0001;ah[ix]=a;
  shaftPlantAngle=physicalQ;
  sensorPlantAngle=4.1+ah[(ix-sensorLag+4096)%4096]+bias+positionBias*physicalQ+upBias*(1-cos(a))*.5+.002*sin(tick*.024);
  simulatedMicros+=100;pulseISR();if(!jitterControl||tick>=nextControlTick){loop();nextControlTick=tick+controlIntervals[(controlPatternIndex++)%6];}if(getenv("BALANCE_ONLY")&&upright&&mode==1&&(!getenv("BRAKED_CATCH")||lastmode==3))finishTrial("lost_balance");if(getenv("DISTURBANCE")&&tick==50000)w+=atof(getenv("DISTURBANCE"));double e=fabs(atan2(sin(a-PI),cos(a-PI)));closest=fmin(closest,e);maxq=fmax(maxq,abs(stepCount()*Q));
  angleHeld=(mode==3&&e<.15)?angleHeld+.0001:0;bestAngleHeld=fmax(bestAngleHeld,angleHeld);if(mode==3&&tick>10000){errorSquared+=e*e;errorSamples++;}
  held=(mode==3&&e<.15&&fabs(w)<1.5)?held+.0001:0;best=fmax(best,held);if(mode==3&&lastmode!=3)catches++;lastmode=mode;
  if(!strcmp(finishReason,"success_30s_upright"))success=true;
  if(trace&&tick%100==0)fprintf(trace,"%.5f,%.7f,%.7f,%.7f,%.7f,%d\n",tick*.0001,a-PI,w,physicalQ,speed,mode);
  if(mode==0||mode==5)break;
 }
 if(trace)fclose(trace);
 printf("{\"best_hold_s\":%.4f,\"angle_hold_s\":%.4f,\"balance_rms_deg\":%.4f,\"closest_deg\":%.4f,\"max_arm_deg\":%.3f,\"max_physical_arm_deg\":%.3f,\"captures\":%d,\"success\":%s,\"final_mode\":%d,\"final_steps\":%d,\"reason\":\"%s\"}\n",best,bestAngleHeld,sqrt(errorSquared/fmax(1.,errorSamples))*180/PI,closest*180/PI,maxq*180/PI,maxPhysicalQ*180/PI,catches,success?"true":"false",mode,stepCount(),finishReason);
}
