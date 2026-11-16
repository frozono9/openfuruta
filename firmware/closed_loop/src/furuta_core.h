// furuta_core.h -- FurutaDIY controller core (v2).
//
// One header, no Arduino dependencies. The ESP32 firmware and the PC simulator
// both compile exactly this file, so what is tested in simulation is the code
// that runs on the robot.
//
// Conventions (same as the existing firmware):
//   a  pendulum angle from hanging-down, rad, wrapped to (-pi, pi]
//   e  pendulum angle from upright, e = wrap(a - pi)
//   q  arm angle from home, rad (step count * 2*pi/3200)
//   v  commanded arm speed, rad/s
//   u  commanded arm acceleration, rad/s^2
//   plant:  a'' = -g sin a - b cos a * u + c v^2 sin a cos a - d a'
#pragma once
#include <math.h>
#include <stdint.h>

namespace furuta {

constexpr float kPi = 3.14159265358979f;
inline float wrapPi(float x) { return atan2f(sinf(x), cosf(x)); }
inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

enum Mode : uint8_t { IDLE = 0, SWING = 1, BALANCE = 3 };

struct Params {
  // ---- plant model (identified from the post-repair hanging sweeps) ----
  float g = 133.5f;      // gravity / effective length, rad/s^2
  float b = 1.02f;       // arm-acceleration coupling
  float c = 0.9715f;     // centrifugal coupling (CAD)
  float d = 0.4f;        // viscous damping, 1/s
  float cd = 0.15f;      // speed-squared loss, 1/rad (free-decay fit: large and real)
  float actLag = 0.009f; // command -> pendulum acceleration lag, s

  // ---- observer ----
  float obsW = 90.f;     // angle/rate observer bandwidth, rad/s
  float obsZ = 0.8f;     // observer damping
  float distW = 6.f;     // upright-bias estimator bandwidth, rad/s
  float distMax = 0.10f; // max upright bias it may absorb, rad
  float jumpReject = 0.08f; // residual above this is a sensor glitch, rad
  int   maxInvalid = 25;   // consecutive bad samples before a sensor fault

  // ---- balance (state feedback on [e, w, q, v]) ----
  float kE = 560.f, kW = 50.f, kQ = 28.f, kV = 15.f;
  float balAcc = 65.f, balSpeed = 5.f;
  float fricComp = 1.0f, fricRate = 0.25f; // hinge Coulomb friction feed-forward (rad/s^2, rad/s)
  bool requireMeasuredCapture = false;
  float lostAngle = 0.75f; // |e| beyond this in BALANCE -> back to SWING

  // ---- swing-up ----
  // The pendulum loses a lot of energy at speed (cd), so the pump has to be
  // close to the best the arm can do: reverse the arm at full acceleration as
  // the pendulum passes the bottom, stop it while the pendulum is high, and
  // wind it up again just before the next pass ("stop and go").
  float swAcc = 100.f, swSpeed = 7.5f;
  float swLimit = 1.45f;    // arm travel used while swinging, rad
  float pumpRate = 2.0f;    // w*cos(a) at which the pump saturates, rad/s
  float eTarget = 2.03f;    // wanted energy on arrival at the top (upright = 2)
  float eBand = 0.12f;      // predicted-arrival energy error for full pump
  float cdSwing = 0.15f;    // loss used to predict the coast up to the top
  float aUp = 1.75f;        // |a| above this: pendulum is "high", manage the arm
  float holdQ = 40.f, holdV = 22.f; // arm parking while the pendulum is high
  float windMargin = 0.02f; // wind-up lead time, s
  float regGain = 300.f;    // energy trim during the final rise, rad/s^2 per unit energy
  float learn = 0.7f;       // how fast the arrival-energy error is learned (0 = off)
  float eLow = 1.0f;        // below this energy: gentle pump with arm centring
  float cenQ = 14.f, cenV = 3.5f;
  float kickAcc = 25.f, kickTime = 0.10f;

  // ---- catch test (forward simulation of the balance law) ----
  float catchAngle = 0.55f;   // only test when |e| below this
  float catchHorizon = 0.6f;  // s
  float catchStep = 0.005f;   // s
  float catchMargin = 0.10f;  // model perturbation for the pessimistic rollout

  // ---- arm travel ----
  float softLimit = 1.7087f;  // 97.9 deg, same as the existing firmware
};

struct Core {
  Params p;
  Mode mode = IDLE;

  // commanded arm state
  float speed = 0;       // rad/s, what the stepper is asked to do
  float accelCmd = 0;    // last commanded acceleration
  float accelLag = 0;    // lagged copy used by the observer

  // estimates
  float ah = 0, wh = 0;  // pendulum angle from hanging / rate
  float dist = 0;        // acceleration disturbance (upright bias * -g)
  bool  tracking = false;
  int   invalidRun = 0, rejectedRun = 0;
  float lastRejected = 0;
  bool  sensorFault = false;

  // bookkeeping
  float modeTime = 0, runTime = 0, heldTime = 0;
  float eBias = 0;       // learned correction of the arrival-energy prediction
  float prevA = 0, prevW = 0;
  uint32_t catches = 0, losses = 0, rejected = 0;

  // ------------------------------------------------------------------
  float e() const { return wrapPi(ah - kPi); }
  float eCorrected() const { return e() + dist / p.g; }
  float energy() const { return 0.5f * wh * wh / p.g + 1.f - cosf(ah); }

  void start(float aMeas) {
    mode = SWING; speed = accelCmd = accelLag = 0;
    ah = aMeas; wh = 0; dist = 0; tracking = true; invalidRun = rejectedRun = 0; sensorFault = false;
    modeTime = runTime = heldTime = 0; catches = losses = rejected = 0;
    eBias = 0; prevA = aMeas; prevW = 0;
  }
  // Manual start: pendulum is being held near upright by hand.
  void startBalance(float aMeas) { start(aMeas); mode = BALANCE; }
  void stop() { mode = IDLE; speed = accelCmd = accelLag = 0; }

  // Rate-limit a speed target and keep the arm inside +-limit (same envelope
  // as the old firmware's velocity()).
  void drive(float target, float dt, float q, float accel, float vmax, float limit) {
    target = clampf(target, -vmax, vmax);
    const float room = limit - fabsf(q) - 0.015f;
    const float brake = sqrtf(2.f * accel * (room > 0 ? room : 0));
    if (target * q > 0 && fabsf(target) > brake) target = copysignf(brake, target);
    const float before = speed;
    speed += clampf(target - speed, -accel * dt, accel * dt);
    accelCmd = (speed - before) / dt;
  }

  // ------------------------------------------------------------------
  void observe(float dt, bool valid, float aMeas, float v) {
    const float sa = sinf(ah), ca = cosf(ah);
    accelLag += dt / (p.actLag + dt) * (accelCmd - accelLag);
    const float acc = -p.g * sa - p.b * ca * accelLag + p.c * v * v * sa * ca - p.d * wh - p.cd * wh * fabsf(wh) + dist;
    wh += dt * acc;
    ah = wrapPi(ah + dt * wh);
    float r = 0;
    bool use = valid;
    if (use) {
      r = wrapPi(aMeas - ah);
      // A real pendulum cannot move this far between samples: treat as a glitch.
      const float gate = p.jumpReject + 0.03f * (float)invalidRun;
      if (tracking && fabsf(r) > gate) {
        use = false; ++rejected;
        // ...unless the sensor keeps insisting: then the estimate is what is wrong.
        if (fabsf(wrapPi(aMeas - lastRejected)) < 0.15f) { if (++rejectedRun >= 5) { ah = aMeas; r = 0; use = true; } }
        else rejectedRun = 1;
        lastRejected = aMeas;
      } else rejectedRun = 0;
    }
    if (!use) {
      if (++invalidRun > p.maxInvalid) sensorFault = true;
      return;
    }
    invalidRun = 0; tracking = true;
    const float dtc = dt < 0.004f ? dt : 0.004f;
    const float k1 = 2.f * p.obsZ * p.obsW * dtc;
    const float k2 = p.obsW * p.obsW * dtc;
    ah = wrapPi(ah + k1 * r);
    wh += k2 * r;
    if (mode == BALANCE && fabsf(e()) < 0.35f) {
      dist += p.obsW * p.obsW * p.distW * dtc * r;
      dist = clampf(dist, -p.distMax * p.g, p.distMax * p.g);
    }
  }

  // ------------------------------------------------------------------
  float balanceAccel(float ec, float w, float q, float v) const {
    const float ff = p.fricComp / p.b * clampf(w / p.fricRate, -1.f, 1.f);
    return clampf(-p.kE * ec - p.kW * w + p.kQ * q + p.kV * v + ff, -p.balAcc, p.balAcc);
  }

  // Would the balance law recover from this state? Rolled out on the model.
  bool rollout(float ec, float w, float q, float v, float g, float b) const {
    const float h = p.catchStep;
    const int n = (int)(p.catchHorizon / h);
    float lag = accelLag;
    for (int i = 0; i < n; i++) {
      float target = v + balanceAccel(ec, w, q, v) * h;
      target = clampf(target, -p.balSpeed, p.balSpeed);
      const float room = p.softLimit - fabsf(q) - 0.015f;
      const float brake = sqrtf(2.f * p.balAcc * (room > 0 ? room : 0));
      if (target * q > 0 && fabsf(target) > brake) target = copysignf(brake, target);
      const float vn = v + clampf(target - v, -p.balAcc * h, p.balAcc * h);
      const float u = (vn - v) / h;
      lag += h / (p.actLag + h) * (u - lag);
      q += h * 0.5f * (v + vn); v = vn;
      // small-angle-safe trig: |ec| stays below ~0.9 here
      const float s = sinf(ec), co = cosf(ec);
      w += h * (g * s + b * co * lag + p.c * v * v * s * co - p.d * w - p.cd * w * fabsf(w));
      ec += h * w;
      if (fabsf(ec) > 0.9f || fabsf(q) > p.softLimit - 0.12f) return false;
    }
    return fabsf(ec) < 0.06f && fabsf(w) < 0.8f;
  }
  bool catchFeasible(float q, float v) const {
    const float ec = e(), w = wh;
    if (fabsf(ec) > p.catchAngle) return false;
    const float m = p.catchMargin;
    return rollout(ec, w, q, v, p.g, p.b) &&
           rollout(ec, w, q, v, p.g * (1 + m), p.b * (1 - m)) &&
           rollout(ec, w, q, v, p.g * (1 - m), p.b * (1 + m));
  }

  // Energy after coasting (no arm motion) from |a| = x0 up to |a| = x1 > x0,
  // with the speed-squared loss: dE/dx = -k (E - 1 + cos x), k = 2 cd.
  static float coastUp(float E, float x0, float x1, float k) {
    const float L = x1 - x0, ek = expf(k * L);
    const float I = (ek - 1.f) / k - (ek * (k * cosf(x1) + sinf(x1)) - (k * cosf(x0) + sinf(x0))) / (k * k + 1.f);
    return (E + k * I) / ek;
  }
  // Same, coasting down from x0 to x1 < x0.
  static float coastDown(float E, float x0, float x1, float k) {
    const float L = x0 - x1, ek = expf(k * L);
    const float I = (ek - 1.f) / k - (ek * (k * cosf(x1) - sinf(x1)) - (k * cosf(x0) - sinf(x0))) / (k * k + 1.f);
    return (E + k * I) / ek;
  }
  // Energy the pendulum would have at the top if the arm did nothing more.
  float predictedTopEnergy() const {
    const float k = 2.f * p.cdSwing, x = fabsf(ah);
    float E = energy();
    if (ah * wh >= 0) return coastUp(E, x, kPi, k);        // rising
    E = coastDown(E, x, 0.f, k);                             // falling: to the bottom...
    return coastUp(E, 0.f, kPi, k);                          // ...and up the other side
  }

  float swingAccel(float q, float v) const {
    if (runTime < p.kickTime) return p.kickAcc;
    const float a = ah, w = wh, x = fabsf(a);
    const float sw = w >= 0 ? 1.f : -1.f;
    const float en = energy();
    const float eTop = predictedTopEnergy();
    const float want = clampf((p.eTarget + eBias - eTop) / p.eBand, -1.f, 1.f); // +1 pump, -1 dump
    const float park = -p.holdQ * q - p.holdV * v;
    float u;
    if (x < p.aUp) {
      // low: pump. u = -U sign(w cos a) adds energy.
      const float phase = clampf(w * cosf(a) / p.pumpRate, -1.f, 1.f);
      u = -p.swAcc * want * phase;
      if (en < p.eLow) u = u * (0.45f + 0.55f * en / p.eLow) - p.cenQ * q - p.cenV * v;
    } else if (a * w >= 0) {
      // high and rising: stop the arm, trim the arrival energy
      u = sw * p.regGain * (p.eTarget + eBias - eTop) + park;
    } else {
      // high and falling: park, then wind the arm up just before the pendulum
      // passes horizontal so the reversal is centred on the bottom
      const float dx = x - 0.5f * kPi;
      const float alpha = p.g * sinf(x) + 1.f;
      const float tH = dx > 0 ? (-fabsf(w) + sqrtf(w * w + 2.f * alpha * dx)) / alpha : 0.f;
      const float vWind = p.swSpeed * want;
      if (tH < fabsf(vWind) / p.swAcc + p.windMargin) u = (sw * vWind - v) * 400.f;
      else u = park;
    }
    return clampf(u, -p.swAcc, p.swAcc);
  }

  // ------------------------------------------------------------------
  // One control tick. Returns the arm speed to command (rad/s).
  //   dt     seconds since the previous tick
  //   valid  pendulum sample is usable
  //   aMeas  pendulum angle from hanging (rad)
  //   q      arm angle from the step counter (rad)
  float update(float dt, bool valid, float aMeas, float q, float measuredRate=0) {
    if (mode == IDLE) { speed = 0; return 0; }
    if (dt < 0.0005f) dt = 0.0005f;
    if (dt > 0.02f) dt = 0.02f;
    observe(dt, valid, aMeas, speed);
    modeTime += dt; runTime += dt;
    const float err = e();

    if (mode == SWING) {
      const float measuredError=wrapPi(aMeas-kPi);
      const bool measuredReady=valid && fabsf(measuredError)<.20f &&
          fabsf(measuredRate)<3.125f && fabsf(q)<1.5f && fabsf(speed)<2.5f;
      if ((!p.requireMeasuredCapture || measuredReady) && catchFeasible(q, speed)) {
        if(p.requireMeasuredCapture){
          ah=aMeas;
          wh=measuredRate;
          dist=0;
        }
        mode = BALANCE; modeTime = 0; heldTime = 0; ++catches;
      }
    } else if (fabsf(err) > p.lostAngle) {
      mode = SWING; modeTime = 0; heldTime = 0; dist = 0; ++losses;
    }

    if (mode == BALANCE) {
      const float u = balanceAccel(eCorrected(), wh, q, speed);
      drive(speed + u * dt, dt, q, p.balAcc, p.balSpeed, p.softLimit);
      if (fabsf(err) < 0.15f) heldTime += dt; else heldTime = 0;
    } else {
      // Learn from how the last pass ended: over the top with speed left
      // (too much energy) or a peak short of the top (too little).
      const float x = fabsf(ah);
      if (x > p.aUp && p.learn > 0) {
        float arrived = -1;
        if (ah * prevA < 0 && x > 2.6f) arrived = energy();                 // went over the top
        else if (wh * prevW < 0 && ah * prevW > 0) arrived = 1.f - cosf(x); // peaked below the top
        if (arrived >= 0) eBias = clampf(eBias + p.learn * (p.eTarget - arrived), -1.5f, 1.5f);
      }
      prevA = ah; prevW = wh;
      const float u = swingAccel(q, speed);
      drive(speed + u * dt, dt, q, p.swAcc, p.swSpeed, p.swLimit);
    }
    return speed;
  }
};

} // namespace furuta
