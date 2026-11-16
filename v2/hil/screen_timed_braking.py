"""Screen approach timing with the physical firmware source and unchanged guards."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import re
import tempfile

import compare

SOURCE = compare.ROOT / 'firmware/closed_loop/src/main.cpp'
TRAIN = ((.844, 80, 70), (1.0, 80, 80), (1.17, 80, 90),
         (1.22, 100, 70), (1.30, 120, 80), (1.17, 150, 150))
TUNING_CHECK = ((.90, 70, 75), (1.02, 100, 85), (1.12, 90, 75),
           (1.19, 110, 85), (1.25, 110, 75))
HOLDOUT = TUNING_CHECK
FRESH = ((.88, 75, 78), (1.03, 92, 82), (1.13, 94, 76),
         (1.18, 103, 87), (1.26, 105, 79))
BASELINE = None
BEST_TUNING_CANDIDATE = (2.1, 32., .025, 1.15, .005, 1, 34., .095)


def firmware(candidate):
    src = SOURCE.read_text()
    if candidate is None:
        return src
    arrival_speed, braking_accel, anticipation, window, velocity_penalty, *predictive = candidate
    time_estimate = "fabsf(errorToVertical)/fmaxf(fabsf(omega),.5f)"
    mode = predictive[0] if predictive else 0
    if mode:
        time_estimate = "crossingRatio<.995f ? atanhf(crossingRatio)/naturalRate : 1.f"
    blend = ""
    if mode >= 2:
        gain = (mode - 1) * .2
        blend = f"""const float blend={gain}f*constrain((.16f-timeToVertical)/.12f,0.f,1.f);
      const float balanceAcceleration=(*v2Storage).balanceAccel(errorToVertical,omega,theta,speed);
      target+=(speed+balanceAcceleration*dt-target)*blend;"""
    return compare.replace_once(src, '    }else v2PlanAt=0;',
        f"""    }}else v2PlanAt=0;
    if(mode==1 && (*v2Storage).mode==furuta::SWING){{
     const float errorToVertical=wrap(angle-PI);
     if(fabsf(errorToVertical)<{window}f && errorToVertical*omega<0){{
      // Predict the crossing from the unforced upright dynamics.
      const float naturalRate=sqrtf(134.2f);
      const float crossingRatio=naturalRate*fabsf(errorToVertical)/fmaxf(fabsf(omega),.5f);
      const float timeToVertical={time_estimate};
      const float brakingTime=fmaxf(0.f,timeToVertical-{anticipation}f);
      const float arrivalCap={arrival_speed}f+{braking_accel}f*brakingTime;
      {blend}
      target=constrain(target,-arrivalCap,arrivalCap);
     }}
    }}""").replace('driveAcc,.005f,5,.01f);', f'driveAcc,{velocity_penalty}f,5,.01f);', 1)


def configured_firmware(candidate):
    src = firmware(candidate[:6] if candidate and len(candidate)>6 else candidate)
    if candidate and len(candidate)>6:
        kick, kick_time, *pump = candidate[6:]
        src = compare.replace_once(src, "p.kickAcc=20.134495550f;",
            f"p.kickAcc={kick}f; (*v2Storage).p.kickTime={kick_time}f;")
        if pump:
            rate, energy, gain, learn = pump
            for old, new in (
                ("2.4f,2.035f,100.f,.332f", f"{rate}f,{energy}f,{gain}f,{learn}f"),
                ("p.pumpRate=1.188285884f;", f"p.pumpRate={rate}f;"),
                ("p.eTarget=2.024310425f;", f"p.eTarget={energy}f;"),
                ("p.regGain=166.541032581f;", f"p.regGain={gain}f;"),
                ("p.learn=.331706582f;", f"p.learn={learn}f;"),
            ):
                src = src.replace(old, new)
    return src


def screen(candidate, cases):
    original = compare.firmware_source
    try:
        def source():
            return configured_firmware(candidate) + '\n#define v2Core (*v2Storage)\n'
        compare.firmware_source = source
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            binary = compare.build(directory)
            for header in SOURCE.parent.glob('*.h'):
                if header.name != 'tmc_uart.h':
                    (directory / header.name).write_bytes(header.read_bytes())
            harness = directory / 'trial.cpp'
            text = harness.read_text()
            text = re.sub(r'(?m)^ v2Core\.p\.\w+=.*?;\n', '', text)
            text = compare.replace_once(text, ' if(const char* gains=getenv("V2_GAINS"))',
                ''' mode=0; on=false;
 process("POLICY V2");
 process("RUN");
 if(!v2Storage || !useV2 || mode!=1)return 3;
 if(const char* gains=getenv("V2_GAINS"))''')
            text = text.replace('v2Core.p.requireMeasuredCapture=getenv("MEASURED_CAPTURE")!=nullptr;', '')
            text = text.replace('else if(useV2)v2Core.start(angle);', '')
            text = compare.replace_once(text, ' double measuredHeld=0,maxMeasuredHeld=0;',
                ' double measuredHeld=0,maxMeasuredHeld=0,firstCatch=-1,activeCatch=-1,lastQualifiedStart=-1;')
            text = compare.replace_once(text, '  measuredHeld=', '''  if(mode==3 && activeCatch<0)activeCatch=tick*.0001;
  if(mode!=3)activeCatch=-1;
  if(activeCatch>=0 && firstCatch<0)firstCatch=activeCatch;
  if(!strcmp(finishReason,"success_30s_upright") && lastQualifiedStart<0)
   lastQualifiedStart=tick*.0001-30.;\n  measuredHeld=''')
            text = compare.replace_once(text, ' printf("AUDIT ', ' fprintf(stderr,"EVENT,STOP,screen_catch_times=%.4f,%.4f,%.6f\\n",firstCatch,lastQualifiedStart,physicalQ*180/PI);\n printf("AUDIT ')
            harness.write_text(text)
            compare.subprocess.run(['clang++','-O3','-std=c++17','-I',str(compare.HIL),
                                    str(harness),'-o',str(binary)],check=True)
            def run(case):
                coupling, delay, lag = case
                row = compare.trial(binary, None, coupling, delay, 'v2', motor_lag=lag,
                    extra={'ADAPTIVE_TRACKING':'1','V2_APPROACH':'1','HOST_GUARD':'1',
                           'MEASURED_CAPTURE':'1','BALANCE_VPI':'0','UPRIGHT_BIAS':'0'})
                marker = row['stop_events'].pop()
                row['first_capture_s'], row['qualified_balance_start_s'], row['final_center_error_deg'] = map(float, marker.rsplit('=',1)[1].split(','))
                row['audited_success'] = (row['success'] and row['measured_hold_s'] >= 29.9
                    and abs(row['final_center_error_deg']) <= .7 and row['final_steps'] == 0 and row['max_physical_arm_deg'] <= 100
                    and row['stop_events'] == ['EVENT,STOP,centered'])
                return row
            with ThreadPoolExecutor(max_workers=6) as pool:
                return list(pool.map(run, cases))
    finally:
        compare.firmware_source = original


if __name__ == '__main__':
    from itertools import product
    import sys
    if "--validate" in sys.argv:
        validation = {}
        for label, cases in (("tuning_check", TUNING_CHECK), ("fresh", FRESH)):
            validation[label] = {
                "cases": cases,
                "baseline": screen(BASELINE, cases),
                "candidate": screen(BEST_TUNING_CANDIDATE, cases),
            }
            print(label, [r["qualified_balance_start_s"] for r in validation[label]["candidate"]], flush=True)
        report = {"parameters": BEST_TUNING_CANDIDATE, "validation": validation}
        (compare.HERE / "timed_braking_validation_final.json").write_text(json.dumps(report, indent=2) + "\n")
        sys.exit(0)
    predictive = '--predictive' in sys.argv
    candidates = [BASELINE, *product((1.8, 2.3), (26., 40.), (.015, .035), (1.15,), (.005,), (1,))] if predictive else [BASELINE, *product((1.6, 2.1), (18., 32., 50.), (.025, .07), (1.15,), (.005, .02))]
    filename = "predictive_braking_screen.json" if predictive else "timed_braking_screen.json"
    output = []
    for candidate in candidates:
        runs = screen(candidate, TRAIN)
        output.append({"candidate": candidate, "runs": runs})
        scores = [r["first_capture_s"] for r in runs if r["audited_success"] and 0 <= r["qualified_balance_start_s"] < 10]
        print(candidate, len(scores), [(r["first_capture_s"],r["reason"]) for r in runs], flush=True)
        (compare.HERE / filename).write_text(json.dumps(output, indent=2) + "\n")
