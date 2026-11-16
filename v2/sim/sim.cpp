// sim.cpp -- plant simulator + Monte Carlo harness for furuta_core.h
//
// Build:  g++ -O2 -std=c++17 -I../core sim.cpp -o sim
// Use:    ./sim mc <trials> <seed> [key=value ...]      Monte Carlo summary + one line per trial
//         ./sim trace <seed> <out.csv> [key=value ...]  one trial with a time series
// Keys:   any controller parameter (kE=..., pumpAcc=...), plus
//         spread=<0..2>   scale of the plant randomisation (0 = nominal plant)
//         start=hang|upright   ctrl=v2|vpi   seconds=<run length>   glitch=<0|1>
#include "furuta_core.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <random>
#include <string>
#include <vector>
#include <algorithm>

using furuta::kPi;
static double wrapd(double x) { return atan2(sin(x), cos(x)); }

struct Plant {
  // pendulum
  double g = 133.0, b = 1.02, c = 0.9715, d = 0.26, fc = 1.2, cd = 0.152;
  // base tilt (rad) and its direction
  double tilt = 0, tiltPhase = 0;
  // stepper: transport delay + second-order rotor response
  double stepDelay = 0.004, wn = 180, zeta = 0.4;
  // encoder
  double inl1 = 0, phi1 = 0, inl2 = 0, phi2 = 0; // nonlinearity harmonics (rad)
  double sensLag = 0.0015, noise = 0.0005;        // s, rad rms
  double restOffset = 0;                           // error of the hanging zero (rad)
  // glitches
  double dropProb = 0, burstProb = 0, jumpProb = 0;
  // timing
  double tick = 0.002, teleExtra = 0.0022, slowProb = 0.003;
  // initial condition
  double a0 = 0;
};

struct Result {
  bool ok = false;        // caught and then held 30 s without leaving +-0.15 rad
  double catchTime = -1;  // s from start to the catch that held
  int catches = 0, losses = 0;
  double maxArmSwing = 0, maxArmBal = 0; // rad
  double rmsErr = 0, rmsArm = 0, maxErr = 0; // during the hold (after 2 s settle)
  double maxAcc = 0;
  bool hitLimit = false, sensorFault = false;
  double biasEst = 0;
};

static std::map<std::string, std::string> opts;
static double optd(const char* k, double def) { auto it = opts.find(k); return it == opts.end() ? def : atof(it->second.c_str()); }
static std::string opts_(const char* k, const char* def) { auto it = opts.find(k); return it == opts.end() ? def : it->second; }

static void applyParams(furuta::Params& p) {
#define P(name) if (opts.count(#name)) p.name = (decltype(p.name))atof(opts[#name].c_str());
  P(g) P(b) P(c) P(d) P(cd) P(actLag) P(obsW) P(obsZ) P(distW) P(distMax) P(jumpReject) P(maxInvalid)
  P(kE) P(kW) P(kQ) P(kV) P(balAcc) P(balSpeed) P(lostAngle) P(fricComp) P(fricRate)
  P(swAcc) P(swSpeed) P(swLimit) P(pumpRate) P(eTarget) P(eBand) P(cdSwing) P(aUp) P(holdQ) P(holdV) P(windMargin) P(regGain) P(learn) P(eLow) P(cenQ) P(cenV) P(kickAcc) P(kickTime)
  P(catchAngle) P(catchHorizon) P(catchStep) P(catchMargin) P(softLimit)
#undef P
}

static Plant randomPlant(std::mt19937& rng, double s) {
  auto U = [&](double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); };
  Plant pl;
  if (s <= 0) return pl;
  pl.g = 133.0 * (1 + s * U(-0.07, 0.07));
  pl.b = 1.02 * (1 + s * U(-0.15, 0.15));
  pl.c = 0.9715 * (1 + s * U(-0.3, 0.3));
  pl.d = std::max(0.05, 0.26 + s * U(-0.2, 0.7));
  pl.cd = std::max(0.0, 0.152 + s * U(-0.06, 0.06));
  pl.fc = std::max(0.0, 1.2 + s * U(-1.0, 1.6));
  pl.tilt = s * U(0, 0.035);            // up to 2 deg of base tilt
  pl.tiltPhase = U(-kPi, kPi);
  pl.stepDelay = std::max(0.0005, 0.004 + s * U(-0.003, 0.006));
  pl.wn = std::max(95.0, 180 * (1 + s * U(-0.4, 0.6)));
  pl.zeta = std::max(0.1, 0.4 + s * U(-0.25, 0.4));
  pl.inl1 = s * U(0, 0.026);            // up to 1.5 deg first harmonic
  pl.phi1 = U(-kPi, kPi);
  pl.inl2 = s * U(0, 0.009);
  pl.phi2 = U(-kPi, kPi);
  pl.sensLag = std::max(0.0003, 0.0015 + s * U(-0.001, 0.0015));
  pl.noise = std::max(0.0001, 0.0005 + s * U(-0.0003, 0.0008));
  pl.restOffset = s * U(-0.012, 0.012);
  pl.a0 = s * U(-0.02, 0.02);
  pl.teleExtra = 0.0022 + s * U(-0.001, 0.002);
  pl.slowProb = 0.003 * (1 + s);
  return pl;
}

struct LegacyVPI { // the 27/300/.03/.02/5 controller that was on the robot, for comparison
  double P = 27, Ki = 300, baseP = .03, baseD = .02, aw = 5, trimRate = .01;
  double integral = 0, baseV = 0, trim = 0, omega = 0, last = 0; bool have = false;
};

static Result run(const Plant& pl, uint32_t seed, FILE* trace) {
  std::mt19937 rng(seed * 7919u + 17u);
  auto U = [&](double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); };
  std::normal_distribution<double> N(0, 1);
  furuta::Core core; applyParams(core.p);
  const bool upright = opts_("start", "hang") == "upright";
  const bool legacy = opts_("ctrl", "v2") == "vpi";
  const bool legacyRef = opts_("ctrl", "v2") == "ref"; int refSign = 0; double peakA = 0, firstTop = -1;
  const bool glitch = optd("glitch", 1) > 0;
  const double seconds = optd("seconds", 75);
  const double holdNeed = optd("hold", 30);
  const double h = 1e-4;                 // physics step = stepper ISR period
  const double Qs = 2 * kPi / 3200;

  double a = upright ? kPi + optd("e0", 0.03) : pl.a0, w = 0;
  double qc = 0, q = 0, qd = 0;          // commanded (continuous), rotor, rotor speed
  double speedCmd = 0;
  const int nd = std::max(1, (int)(pl.stepDelay / h));
  std::vector<double> qcBuf(nd, 0.0); int qi = 0;
  const int ns = std::max(1, (int)(pl.sensLag / h));
  std::vector<double> aBuf(ns, a); int si = 0;
  auto inl = [&](double x) { return pl.inl1 * sin(x + pl.phi1) + pl.inl2 * sin(2 * x + pl.phi2); };
  const double zeroMeas = inl(0) + pl.restOffset;

  Result R; LegacyVPI L;
  double t = 0, nextTick = 0, lastTick = -pl.tick; int tickNo = 0, burst = 0;
  double holdStart = -1, sumE = 0, sumQ = 0; long nHold = 0;
  double lastMeas = 0; bool started = false;
  double jumpHold = 0; int jumpLeft = 0;

  while (t < seconds) {
    if (t >= nextTick) {
      const double dt = t - lastTick; lastTick = t; ++tickNo;
      double extra = 0;
      if (tickNo % 10 == 0) extra += pl.teleExtra * U(0.6, 1.2);
      if (U(0, 1) < pl.slowProb) extra += U(0.002, 0.007);
      nextTick = t + pl.tick + U(0, 0.0003) + extra;
      // --- sensor ---
      const double aOld = aBuf[si];
      double meas = aOld + inl(aOld) + pl.noise * N(rng);
      meas = round(meas / (2 * kPi / 4096)) * (2 * kPi / 4096);
      double reported = wrapd(meas - zeroMeas);
      bool valid = true;
      if (glitch && started) {
        if (burst > 0) { --burst; valid = false; }
        else if (U(0, 1) < pl.dropProb) valid = false;
        else if (U(0, 1) < pl.burstProb) { burst = (int)U(2, 12); valid = false; }
        if (jumpLeft > 0) { --jumpLeft; reported = jumpHold; }
        else if (U(0, 1) < pl.jumpProb) { jumpLeft = (int)U(1, 3); jumpHold = U(-kPi, kPi); reported = jumpHold; }
      }
      lastMeas = reported;
      const double qStep = round(qc / Qs) * Qs;
      // --- controller ---
      if (!started) {
        started = true;
        if (upright) core.startBalance((float)reported); else core.start((float)reported);
        if (legacy) core.mode = furuta::BALANCE;
      } else if (legacy) {
        // old firmware: filtered derivative + velocity PI
        double err = wrapd(reported - kPi - L.trim);
        double angle = reported;
        if (L.have) L.omega += dt / (.008 + dt) * (wrapd(angle - L.last) / dt - L.omega);
        L.last = angle; L.have = true;
        L.trim = std::clamp(L.trim + L.trimRate * qStep * dt, -.04, .04);
        L.baseV += dt / (.2 + dt) * (core.speed - L.baseV);
        double ref = std::clamp(L.baseP * qStep + L.baseD * L.baseV, -.08, .08);
        double inner = err - ref; L.integral += inner * dt;
        double target = -L.P * inner - L.Ki * L.integral - 0.1 * L.omega;
        core.drive((float)target, (float)dt, (float)qStep, 65.f, 5.f, core.p.softLimit);
        L.integral += (target - core.speed) * L.aw / L.Ki * dt;
        L.integral = std::clamp(L.integral, -.15, .15);
        core.ah = (float)reported; core.wh = (float)L.omega;
        if (fabs(err) > 0.6) { R.losses = 1; break; }
      } else if (legacyRef) {
        // old firmware "REFERENCE 70,25,6,2.05,0,.25" energy pump with log barriers, no capture
        core.observe((float)dt, valid, (float)reported, core.speed); core.runTime += dt;
        const double pa = core.ah, pw = core.wh, lim = core.p.softLimit, vlim = 7.5;
        const double transfer = pw * cos(pa);
        if (transfer > .15) refSign = 1; else if (transfer < -.15) refSign = -1;
        const double en = .5 * pw * pw / 130.84 + 1 - cos(pa);
        const double target = 2 + (2.05 - 2) * std::max(0.0, (1 + cos(reported)) * .5);
        const double er = std::clamp((en - target) / .25, -1.0, 1.0);
        const double qr = std::min(.995, fabs(qStep) / lim), vr = std::min(.995, fabs((double)core.speed) / vlim);
        double acc = 70 * er * refSign + (qStep > 0 ? 1 : qStep < 0 ? -1 : 0) * 25 * log10(1 - qr) + (core.speed > 0 ? 1 : core.speed < 0 ? -1 : 0) * 6 * log10(1 - vr);
        if (core.runTime < .2) acc = 16;
        const double la = optd("refAcc", 65);
        core.drive((float)(core.speed + std::clamp(acc, -la, la) * dt), (float)dt, (float)qStep, (float)la, 7.5f, core.p.softLimit);
        peakA = std::max(peakA, fabs(wrapd(a))); if (fabs(wrapd(a)) > 177 * kPi / 180 && firstTop < 0) firstTop = t;
      } else {
        core.update((float)dt, valid, (float)reported, (float)qStep);
      }
      speedCmd = core.speed;
      if (core.sensorFault) { R.sensorFault = true; break; }
      // --- metrics ---
      const double e = wrapd(a - kPi);
      const bool bal = core.mode == furuta::BALANCE;
      if (bal) {
        R.maxArmBal = std::max(R.maxArmBal, fabs(q));
        if (fabs(e) < 0.15) {
          if (holdStart < 0) { holdStart = t; sumE = sumQ = 0; nHold = 0; R.maxErr = 0; }
          if (t - holdStart > 2) { sumE += e * e; sumQ += q * q; ++nHold; R.maxErr = std::max(R.maxErr, fabs(e)); }
          if (t - holdStart >= holdNeed && !R.ok) { R.ok = true; R.catchTime = holdStart; }
        } else if (!R.ok) holdStart = -1;
      } else { if (!R.ok) holdStart = -1; R.maxArmSwing = std::max(R.maxArmSwing, fabs(q)); }
      R.maxAcc = std::max(R.maxAcc, (double)fabs(core.accelCmd));
      if (trace && tickNo % 2 == 0)
        fprintf(trace, "%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%d,%.5f,%.5f,%.4f,%.4f,%d\n", t, a, w, q, qd, speedCmd,
                (int)core.mode, core.ah, core.wh, core.dist / core.p.g, core.accelCmd, valid ? 1 : 0);
      if (R.ok && t - R.catchTime > holdNeed + 0.5 && optd("stopAfterHold", 1) > 0) break;
    }
    // --- physics (100 us) ---
    qc += speedCmd * h;
    const double qcDelayed = qcBuf[qi];
    qcBuf[qi] = round(qc / Qs) * Qs; qi = (qi + 1) % nd;
    const double qdd = pl.wn * pl.wn * (qcDelayed - q) - 2 * pl.zeta * pl.wn * qd;
    qd += h * qdd; q += h * qd;
    const double delta = pl.tilt * sin(q - pl.tiltPhase);
    const double sa = sin(a), ca = cos(a);
    const double acc = -pl.g * sin(a - delta) - pl.b * ca * qdd + pl.c * qd * qd * sa * ca - pl.d * w - pl.cd * w * fabs(w) - pl.fc * tanh(w / 0.05);
    w += h * acc; a += h * w;
    aBuf[si] = a; si = (si + 1) % ns;
    if (fabs(q) > 100 * kPi / 180) { R.hitLimit = true; break; }
    t += h;
  }
  R.catches = core.catches; R.losses += core.losses;
  if (nHold) { R.rmsErr = sqrt(sumE / nHold); R.rmsArm = sqrt(sumQ / nHold); }
  R.biasEst = core.dist / core.p.g;
  if (legacyRef) { R.rmsErr = peakA; R.catchTime = firstTop; }
  if (legacy && !R.hitLimit && R.losses == 0 && holdStart >= 0 && seconds - holdStart >= holdNeed) R.ok = true;
  (void)lastMeas;
  return R;
}

int main(int argc, char** argv) {
  if (argc < 3) { fprintf(stderr, "usage: sim mc N seed [k=v] | sim trace seed out.csv [k=v]\n"); return 2; }
  const std::string cmd = argv[1];
  const int first = cmd == "mc" ? 4 : 4;
  for (int i = first; i < argc; i++) { char* eq = strchr(argv[i], '='); if (eq) opts[std::string(argv[i], eq - argv[i])] = eq + 1; }
  const double spread = optd("spread", 1.0);
  auto mk = [&](uint32_t seed) {
    std::mt19937 rng(seed);
    Plant pl = randomPlant(rng, spread);
    if (optd("glitch", 1) > 0) { pl.dropProb = optd("drop", 0.002); pl.burstProb = optd("burst", 0.0003); pl.jumpProb = optd("jump", 0.0002); }
#define O(name) if (opts.count("pl." #name)) pl.name = atof(opts["pl." #name].c_str());
    O(g) O(b) O(c) O(d) O(cd) O(fc) O(tilt) O(tiltPhase) O(stepDelay) O(wn) O(zeta) O(inl1) O(phi1) O(inl2) O(phi2) O(sensLag) O(noise) O(restOffset) O(a0) O(teleExtra) O(slowProb) O(tick)
#undef O
    return pl;
  };
  if (cmd == "trace") {
    const uint32_t seed = atoi(argv[2]);
    FILE* f = fopen(argv[3], "w");
    fprintf(f, "t,a,w,q,qd,speed,mode,ah,wh,bias,acc,valid\n");
    Plant pl = mk(seed); Result r = run(pl, seed, f); fclose(f);
    printf("ok=%d catch=%.2f catches=%d losses=%d armSwing=%.1fdeg armBal=%.1fdeg rmsErr=%.2fdeg rmsArm=%.1fdeg maxErr=%.2fdeg limit=%d fault=%d bias=%.2fdeg | g=%.1f b=%.2f d=%.2f fc=%.2f tilt=%.2fdeg delay=%.1fms inl1=%.2fdeg\n",
           r.ok, r.catchTime, r.catches, r.losses, r.maxArmSwing * 57.3, r.maxArmBal * 57.3, r.rmsErr * 57.3, r.rmsArm * 57.3, r.maxErr * 57.3, r.hitLimit, r.sensorFault, r.biasEst * 57.3,
           pl.g, pl.b, pl.d, pl.fc, pl.tilt * 57.3, pl.stepDelay * 1e3, pl.inl1 * 57.3);
    return 0;
  }
  const int n = atoi(argv[2]); const uint32_t seed0 = atoi(argv[3]);
  int ok = 0, lim = 0, fault = 0; std::vector<double> ct, re, ra, sw, mb; int totalLoss = 0;
  const bool verbose = optd("verbose", 0) > 0;
  for (int i = 0; i < n; i++) {
    Plant pl = mk(seed0 + i); Result r = run(pl, seed0 + i, nullptr);
    ok += r.ok; lim += r.hitLimit; fault += r.sensorFault; totalLoss += r.losses;
    if (r.ok) { ct.push_back(r.catchTime); re.push_back(r.rmsErr * 57.3); ra.push_back(r.rmsArm * 57.3); mb.push_back(r.maxArmBal * 57.3); }
    sw.push_back(r.maxArmSwing * 57.3);
    if (verbose || !r.ok)
      printf("seed=%u ok=%d catch=%.2f catches=%d losses=%d armSwing=%.0f armBal=%.0f rmsErr=%.2f rmsArm=%.1f limit=%d fault=%d | g=%.1f b=%.2f d=%.2f fc=%.2f cd=%.3f tilt=%.2f/%.1f delay=%.1fms wn=%.0f inl=%.2f/%.2f rest=%.2f\n",
             seed0 + i, r.ok, r.catchTime, r.catches, r.losses, r.maxArmSwing * 57.3, r.maxArmBal * 57.3, r.rmsErr * 57.3, r.rmsArm * 57.3, r.hitLimit, r.sensorFault,
             pl.g, pl.b, pl.d, pl.fc, pl.cd, pl.tilt * 57.3, pl.tiltPhase, pl.stepDelay * 1e3, pl.wn, pl.inl1 * 57.3, pl.inl2 * 57.3, pl.restOffset * 57.3);
  }
  auto pct = [](std::vector<double> v, double p) { if (v.empty()) return 0.0; std::sort(v.begin(), v.end()); return v[std::min(v.size() - 1, (size_t)(p * v.size()))]; };
  printf("SUMMARY n=%d ok=%d (%.1f%%) limit=%d fault=%d losses=%d | catch s p50=%.1f p90=%.1f max=%.1f | hold rmsErr deg p50=%.2f p90=%.2f | rmsArm deg p50=%.1f p90=%.1f | armBal max p90=%.0f | armSwing p50=%.0f max=%.0f\n",
         n, ok, 100.0 * ok / n, lim, fault, totalLoss, pct(ct, .5), pct(ct, .9), pct(ct, .999), pct(re, .5), pct(re, .9), pct(ra, .5), pct(ra, .9), pct(mb, .9), pct(sw, .5), pct(sw, .999));
  return 0;
}
