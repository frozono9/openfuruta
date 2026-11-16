"""Offline PI versus v2 using the real firmware loop and paired plant states."""
import concurrent.futures
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
FW = ROOT / "firmware/closed_loop"
HIL = ROOT / "sim/hardware_balance/hil"
FIXTURE = ROOT / "sim/hardware_balance/native"
PI_TRIAL = 1794415855
CAPTURES = (1794411578, 1794411799, 1794412996, 1794413663, 1794413757,
            1794413837, 1794414492, 1794414651, PI_TRIAL)
SENSOR_BIAS = -.0286
POLICIES = ("pi", "v2", "v2_transition", "v2_tuned", "v2_tuned_transition", "v2_delay_tuned")

def replace_once(source, old, new):
    if source.count(old) != 1:
        raise ValueError(f"Expected exactly one occurrence of {old}")
    return source.replace(old, new)

def firmware_source():
    src = (ROOT / "sim/hardware_balance/baseline/main.cpp").read_text()
    src = '#include "furuta_core.h"\nstatic furuta::Core v2Core;\nstatic bool useV2=false;\n' + src
    transition_state = (
        "static bool useTransition=false;\n"
        "static float v2Blend=0, v2ReadyTime=0, v2FullSeconds=0;\n"
        "static float v2BalanceTarget=0;\n"
    )
    src = transition_state + src
    src = replace_once(src, " if(balanceExcitation){",
        """ if(useTransition && v2Blend>0){
  target += v2Blend*(v2BalanceTarget-target);
 }
 if(balanceExcitation){""")
    src = """static bool adaptiveTracking=false, fastTracking=false, slowTracking=false;
static bool approachV2=false;
static unsigned long v2PlanAt=0;
static float v2PlanAcceleration=0;
static int trackingSamples=0;
static float trackingLag=0;
static float slowPump[4]={2.4f,2.035f,100.f,.332f};
""" + src
    src = replace_once(src, "  if(shaftValid&&abs(position)>armHardSteps*Q)",
        """  if(adaptiveTracking && shaftValid && (mode==1||mode==3) && fabsf(speed)>.8f){
   const float lag=(stepCount()*Q-position)/speed;
   if(lag>=0 && lag<.05f){
    trackingLag=trackingSamples? .65f*trackingLag+.35f*lag:lag;
    ++trackingSamples;
    if(trackingSamples>=3){
     if(trackingLag>.009f)slowTracking=true;
     fastTracking=!slowTracking && trackingLag<.0075f;
    }
   }
  }
  if(shaftValid&&abs(position)>armHardSteps*Q)""")
    start = "  if(mode==1&&!neuralFull){float elapsed="
    end = "  if(mode==1||mode==3||mode==AUTO_PROBE_BRAKE_MODE){pastAcceleration"
    branch = """  if(useV2 && (mode==1||mode==3)){
   if(millis()-runAt>=90000)finishTrial("run_90s_complete");
   else if(mode==1 && millis()-runAt>pumpBudgetMs)finishTrial("swingup_timeout");
   else{
    if(adaptiveTracking){
     const bool fast=fastTracking&&!slowTracking;
     v2Core.p.swSpeed=fast?7.5f:3.5f;
     v2Core.p.balSpeed=fast?5.f:3.5f;
     if(!fast){
      v2Core.p.pumpRate=slowPump[0];v2Core.p.eTarget=slowPump[1];
      v2Core.p.regGain=slowPump[2];v2Core.p.learn=slowPump[3];
     }else{
      v2Core.p.pumpRate=1.188285884f;v2Core.p.eTarget=2.024310425f;
      v2Core.p.regGain=166.541032581f;v2Core.p.learn=.331706582f;
     }
    }
    float target=0;
    const int previousMode=mode;
    if(useTransition && mode==3){
     v2Core.observe(dt,true,angle,speed);
     if(fabsf(wrap(angle-PI))>.55f){
      v2Core.mode=furuta::SWING;
      v2Core.dist=0;
      v2Core.modeTime=0;
      v2Blend=v2ReadyTime=v2FullSeconds=0;
      target=speed;
     }else{
      if(v2Blend==0){
       const bool quiet=fabsf(wrap(angle-PI))<.08f && fabsf(omega)<.6f &&
           fabsf(theta)<.65f && fabsf(speed)<1.5f;
       v2ReadyTime=quiet?v2ReadyTime+dt:0;
      }
      if(v2ReadyTime>=.5f)v2Blend=fminf(1.f,v2Blend+dt);
      if(v2Blend>=1.f)v2FullSeconds+=dt;
      const float u=v2Core.balanceAccel(v2Core.eCorrected(),v2Core.wh,theta,speed);
      v2BalanceTarget=constrain(speed+dt*u,-swingBalanceMax,swingBalanceMax);
     }
    }else target=v2Core.update(dt,true,angle,theta,omega);
    if(approachV2 && mode==1 && v2Core.mode==furuta::SWING &&
       fabsf(wrap(angle-PI))<1.05f && wrap(angle-PI)*omega<0){
     if(!v2PlanAt || millis()-v2PlanAt>=10){
      v2PlanAt=millis();
      v2PlanAcceleration=captureApproachAcceleration(wrap(angle-PI),omega,theta,speed,
         134.2f,1.17f,1.f,motionSoftLimit()-.015f,v2Core.p.swSpeed,driveAcc,.005f,5,.01f);
     }
     target=constrain(speed+v2PlanAcceleration*dt,-v2Core.p.swSpeed,v2Core.p.swSpeed);
    }else v2PlanAt=0;
    if(v2Core.sensorFault)stop("v2_observer_fault");
    else{
     mode=v2Core.mode;
     if(useTransition && mode==3){
      if(previousMode!=3){
       resetVPI(error,theta);
       v2Blend=v2ReadyTime=v2FullSeconds=0;
      }
      velocityBalance(error,theta,dt);
     }else velocity(target,dt,driveAcc,mode==3?swingBalanceMax:driveMax);
     v2Core.speed=speed;
     v2Core.accelCmd=(speed-commandBefore)/dt;
     if(mode==3 && fabsf(wrap(angle-PI))<.15f && fabsf(omega)<1.5f){
      if(!heldAt)heldAt=millis();
      if(millis()-heldAt>=30000)finishTrial("success_30s_upright");
     }else heldAt=0;
    }
   }
  }else{
"""
    src = replace_once(src, start, branch + start)
    src = replace_once(src, end, "  }\n" + end)
    return src

def build(directory):
    for path in (FW / "src").iterdir():
        if path.suffix == ".h":
            shutil.copy2(path, directory)
    shutil.copy2(FIXTURE / "tmc_uart.h", directory)
    shutil.copy2(ROOT / "v2/core/furuta_core.h", directory)
    params = json.loads((ROOT / "v2/sim/best_std.json").read_text())
    params.update(balAcc=65, balSpeed=5, swAcc=65, swSpeed=7.5)
    assignments = "\n".join(f" v2Core.p.{k}={float(v):.9f}f;" for k,v in params.items())
    src = firmware_source()
    (directory / "main.cpp").write_text(src)
    harness = (FIXTURE / "trial.cpp").read_text()
    # Seed both controllers from the same delayed encoder sample and rate.
    harness = replace_once(harness, " double angleHeld=", assignments + """
 if(const char* gains=getenv("V2_GAINS"))
  sscanf(gains,"%f %f %f %f",&v2Core.p.kE,&v2Core.p.kW,&v2Core.p.kQ,&v2Core.p.kV);
 if(const char* pump=getenv("V2_PUMP"))
  sscanf(pump,"%f %f %f %f",&v2Core.p.pumpRate,&v2Core.p.eTarget,&v2Core.p.regGain,&v2Core.p.learn);
 if(const char* swingCap=getenv("V2_SWING_CAP"))
  v2Core.p.swSpeed=fminf(v2Core.p.swSpeed,atof(swingCap));
 if(const char* balanceCap=getenv("V2_BALANCE_CAP"))
  v2Core.p.balSpeed=fminf(v2Core.p.balSpeed,atof(balanceCap));
 v2Core.p.requireMeasuredCapture=getenv("MEASURED_CAPTURE")!=nullptr;
 v2Core.p.softLimit=armSoftLimit;
 useV2=getenv("USE_V2")!=nullptr;
 adaptiveTracking=getenv("ADAPTIVE_TRACKING")!=nullptr;
 approachV2=getenv("V2_APPROACH")!=nullptr;
 useTransition=getenv("USE_TRANSITION")!=nullptr;
 const bool hostGuard=getenv("HOST_GUARD")!=nullptr;
 uint32_t hostLast=0, hostBadSince=0;
 double peakTracking=0;
 const bool injectJump=getenv("INJECT_JUMP")!=nullptr;
 const bool injectLease=getenv("INJECT_LEASE")!=nullptr;
 const bool injectShaft=getenv("INJECT_SHAFT")!=nullptr;
 swingShaftGuard=true;
 swingShaftHomeRaw=0;
 if(upright){
  for(int i=0;i<4096;i++)ah[i]=a+(i-4096)*.0001*w;
  angle=lastAngle=wrap(a-sensorLag*.0001*w+bias+upBias*(1-cos(a))*.5);
  raw=((lround((angle+zero)*4096/(2*PI))%4096)+4096)%4096;
  omega=w;
  resetVPI(wrap(angle-PI-robustBias),initialBase);
  if(useV2)v2Core.startBalance(angle);
 }else if(useV2)v2Core.start(angle);
 v2Core.wh=omega;
 v2Core.prevW=omega;
 v2Core.speed=speed;
 double angleHeld=""")
    harness = replace_once(harness, "  simulatedMicros+=100;pulseISR();if(!jitterControl||tick>=nextControlTick)",
        """  if(injectJump && tick>=6000)sensorPlantAngle+=1.5;
  if(injectShaft && tick>=6000)shaftPlantAngle+=.25;
  simulatedMicros+=100;pulseISR();
  if((!injectLease || tick<6000 || tick>=6300) && (!jitterControl||tick>=nextControlTick))""")
    harness = replace_once(harness, "  angleHeld=", """  if((mode==1||mode==3)&&millis()-hostLast>=20){
   hostLast=millis();
   const double error=fabs(physicalQ-stepCount()*Q)*180/PI;
   peakTracking=fmax(peakTracking,error);
   if(error>3){if(!hostBadSince)hostBadSince=millis();}
   else hostBadSince=0;
   if(hostGuard && (error>5||(hostBadSince&&millis()-hostBadSince>=40)))
    stop("host_shaft_tracking");
  }
  angleHeld=""")
    harness = replace_once(harness, " const int couplingDelay=", """
 const bool captureTransientDelay=getenv("CAPTURE_TRANSIENT_DELAY")!=nullptr;
 const int couplingDelay=""")
    harness = replace_once(harness,
        "  const double coupledAcceleration=accelerationHistory[(ix+4096-delay)%4096];",
        """  const double delayedAcceleration=accelerationHistory[(ix+4096-delay)%4096];
  const double delayMix=captureTransientDelay?std::clamp((.35-tick*.0001)/.20,0.,1.):1.;
  const double coupledAcceleration=motorAcceleration+delayMix*(delayedAcceleration-motorAcceleration);""")
    harness = replace_once(harness, " double accelerationHistory[4096]={};", """
 double accelerationHistory[4096]={};
 if(const char* history=getenv("CAPTURE_ACCEL_HISTORY")){
  double aPast[60];
  for(double& sample:aPast){
   char* end=nullptr;
   sample=strtod(history,&end);
   if(end==history)return 2;
   history=end;
  }
  for(int i=0;i<600;i++){
   const int left=i/10,right=std::min(left+1,59);
   accelerationHistory[4096-600+i]=aPast[left]+(aPast[right]-aPast[left])*(i%10)*.1;
  }
 }
""")
    # Track longest strict interval in the plant, independently of the controller's own success event.
    # Use the observed firmware threshold for the recorded audit and true plant state as a second metric.
    harness = replace_once(harness, " double angleHeld=", " double measuredHeld=0,maxMeasuredHeld=0;\n double angleHeld=")
    harness = replace_once(harness, "  angleHeld=", """  measuredHeld=(mode==3&&valid&&fabs(wrap(angle-PI))<.15&&fabs(omega)<1.5)
      ? measuredHeld+.0001 : 0;
  maxMeasuredHeld=fmax(maxMeasuredHeld,measuredHeld);
  angleHeld=""")
    harness = replace_once(harness, ' printf("{\\"best_hold_s', ' printf("AUDIT %.6f %d %u %.6f %.6f %d %d %.6f\\n",maxMeasuredHeld,on,inc,v2FullSeconds,peakTracking,fastTracking,slowTracking,trackingLag);\n printf("{\\"best_hold_s')
    (directory / "trial.cpp").write_text(harness)
    binary = directory / "run"
    subprocess.run(["clang++", "-O3", "-std=c++17", "-I", str(HIL),
                    str(directory/"trial.cpp"), "-o", str(binary)], check=True)
    return binary

def entries():
    for stamp in CAPTURES:
        d = json.loads((FW/f"tests/velocity_phase_current-{stamp}.json").read_text())
        r = next(r for r in d["rows"] if r["mode"] == 3 and r["sensor_valid"] and r["shaft_valid"])
        e = math.remainder(math.radians(r["angle_deg"])-math.pi, math.tau)-SENSOR_BIAS
        yield dict(trial=stamp, device_ms=r["device_ms"], physical_success=any(
            "TRIAL_FINISH,success_30s_upright" in x for x in d["events"]),
            initial=[e,r["pendulum_speed_rad_s"],r["arm_counted_rad"],r["motor_speed_rad_s"]])

def trial(binary, entry, coupling, delay, policy, fault=None, motor_lag=90, gains=None, extra=None):
    upright = entry is not None
    env = dict(PATH=os.environ["PATH"], BALANCE_VPI="1", VPI_PARAMETERS="27 300 .03 .02 5",
        UPRIGHT_BIAS=str(SENSOR_BIAS), MODEL="134.2 1.17 .25", UP_MODEL=f"134.2 {coupling} 1.5 {motor_lag}",
        COULOMB="1.83037", DRIVE_MAX="7.5", DRIVE_ACC="65", ARM_TRAVEL="100",
        TRIM_RATE=".01", REFERENCE_PUMP="1", REFERENCE_V="6", INITIAL_KICK="16",
        REFERENCE_TAPER="1", REFERENCE_PREDICTOR="134.4 .844106 1.904 4",
        SWING_MODEL="134.4 .844106 1.904", APPROACH_MODEL="134.2 1.17 .9934 4",
        CAPTURE_ANGLE=".2", CAPTURE_SPEED="2.5", CAPTURE_ARM="1.5", CAPTURE_MOTOR="2.5",
        BALANCE_LIMITS="5 100", CAPTURE_ASSIST="1", CAPTURE_WINDOW="1.15", CAPTURE_MIN_ENERGY="1.8",
        CAPTURE_STAGES="5", CAPTURE_VPENALTY=".005", PUMP_BUDGET="50000", CONTROL_JITTER="1")
    if upright:
        env["BALANCE_ONLY"]="1"
        env.update(zip(("INITIAL_ANGLE","INITIAL_OMEGA","INITIAL_BASE","INITIAL_BASE_SPEED"),
                       map(str,entry["initial"])))
    if policy != "pi":
        env["USE_V2"]="1"
    if policy in ("v2_transition", "v2_tuned_transition"):
        env["USE_TRANSITION"]="1"
    if gains is None and policy in ("v2_tuned", "v2_tuned_transition"):
        gains = json.loads((HERE/"entry_gain_search.json").read_text())["best"]["gains"]
    if gains is None and policy == "v2_delay_tuned":
        gains = json.loads((HERE/"delay_entry_gain_search.json").read_text())["best"]["gains"]
    if gains:
        env["V2_GAINS"]=" ".join(map(str,gains))
    if extra:
        env.update(extra)
    if fault:
        env["INJECT_"+fault.upper()]="1"
    cmd=[str(binary),"1","0","70","25","6","2.05",str(motor_lag),str(delay),
         "134.2",".844106",".9934","0",str(int(upright)),"400","55","5","8","0","1"]
    completed=subprocess.run(cmd,env=env,capture_output=True,text=True,check=True,timeout=30)
    lines=completed.stdout.splitlines()
    result=json.loads(lines[-1])
    _, measured, enabled, pulses, full_seconds, peak_tracking, fast, slow, tracking_lag=next(line.split() for line in lines if line.startswith("AUDIT "))
    result.update(entry=entry,policy=policy,coupling=coupling,sensor_delay_ms=delay/10,
                  motor_delay_ms=motor_lag/10, measured_hold_s=float(measured),motor_enabled=bool(int(enabled)),pulse_increment=int(pulses),
                  v2_full_seconds=float(full_seconds), peak_tracking_deg=float(peak_tracking), tracking_fast=bool(int(fast)), tracking_slow=bool(int(slow)), estimated_lag_ms=float(tracking_lag)*1000, fault=fault,stop_events=[line for line in completed.stderr.splitlines() if line.startswith("EVENT,STOP,")])
    result["audited_success"]=result["success"] and result["measured_hold_s"]>=29.9 and result["final_steps"]==0 and result["max_physical_arm_deg"]<=100 and result["stop_events"]==["EVENT,STOP,centered"]
    return result

def main():
    with tempfile.TemporaryDirectory(prefix="furuta-v2-hil-") as directory:
        binary=build(Path(directory))
        # Six paired plant cases for each measured entry and for the hanging start.
        cases=list(itertools.product([*entries(),None],[.84,1.17,1.4],[80,150],POLICIES))
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
            results=list(pool.map(lambda c: trial(binary,*c),cases))
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
            results.extend(pool.map(lambda c: trial(binary,*c,motor_lag=0),
                itertools.product([None],[.84,1.17,1.4],[80,150],POLICIES)))
        fault_kinds = ("jump","lease","shaft")
        faults=[trial(binary,None,1.17,80,policy,kind)
                for policy in ("v2_transition","v2_tuned","v2_tuned_transition","v2_delay_tuned")
                for kind in fault_kinds]
    reasons = {"jump":"pendulum_angle_jump","lease":"control_command_timeout","shaft":"shaft_tracking_fault"}
    for case in faults:
        reason = reasons[case["fault"]]
        assert not case["motor_enabled"] and case["pulse_increment"]==0 and case["final_mode"]==0, case
        assert any(reason in s or (case["fault"]=="shaft" and "shaft_tracking_fault" in s)
                   for s in case["stop_events"]),case
    report=dict(
        scope="Offline real ESP32 firmware loop and STEP ISR with simulated I2C and motor. Captures seed counterfactual states, not prerecorded feedback. Current plant is approximate; no hardware commands.",
        baseline_trial=PI_TRIAL, firmware_sha256=hashlib.sha256((FW/"src/main.cpp").read_bytes()).hexdigest(),
        criterion="Firmware 30 s hold event at abs(error)<0.15 rad and abs(rate)<1.5 rad/s plus sampled duration >=29.9 s, home to step zero, and no fault or travel violation. True plant hold is reported separately.",
        limits=dict(travel_deg=100,acceleration=65,swing_speed=7.5,balance_speed=5),
        cases=results,fault_tests=faults)
    (HERE/"delay_entry_comparison.json").write_text(json.dumps(report,indent=2)+"\n")
    for entry in entries():
        subset=[r for r in results if r["entry"]==entry]
        print(entry["trial"] if entry else "hang",[(policy,sum(r["audited_success"] for r in subset if r["policy"]==policy)) for policy in POLICIES])
    for lag in (0.,9.):
        for policy in POLICIES:
            subset=[r for r in results if r["entry"] is None and r["motor_delay_ms"]==lag and r["policy"]==policy]
            print("hang",lag,policy,sum(r["audited_success"] for r in subset),"/",len(subset))
    print("Fault tests:",[r["stop_events"] for r in faults])

if __name__=="__main__":
    main()
