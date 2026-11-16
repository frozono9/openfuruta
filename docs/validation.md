# Validation

Each physical attempt starts with the pendulum hanging and the arm near its independently measured center. A successful cycle requires a firmware-confirmed 30-second hold and return to center. The sampled duration can be a few milliseconds shorter than the firmware's internal timer.

| Test | Completion | Evidence |
| --- | --- | --- |
| v2 approach, physical, November 13 | 4 of 5. One attempt timed out before capture. | [Report](results/approach-2026-11-13.json) |
| v2 approach, physical, November 16 | 2 of 5 attempts. Two timed out and the fifth was interrupted by loss of telemetry. | [Report](results/approach-2026-11-16.json) |
| Predictive braking, native simulation | 5 of 5 held-out plants met the stricter target of swing-up under 10 s, 30 seconds balanced and return to center. | [Report](../v2/hil/timed_braking_round2_validation.json) |
| Predictive braking, physical, November 16 | 0 of 5 full cycles. One attempt first reached capture at 4.978 s and held for 0.266 s. All five returned to center. | [Report](results/predictive-2026-11-16.json) |

The native simulation estimates motor transport delay, sensor delay and coupling. The five predictive swing-up times were 7.523, 5.635, 5.158, 9.969 and 4.247 seconds. The high-delay stress case tripped a tracking guard. The physical tests show that the simulated capture result did not transfer reliably to this prototype.

The release firmware is the v2 approach build. The predictive variant is generated in the [native simulation](../sim/README.md#reproduce-the-controller-test). Raw [physical telemetry](results/telemetry/) includes events, command settings, firmware hashes and traces. Each physical report references its data relative to the report.

The two PlatformIO targets build. MuJoCo behavioral checks and the native predictive test pass on the released source. The bed-oriented STLs pass mesh closure and volume checks. The original physical trials establish behavior on one assembled unit. The documented build, revised print orientations and portable serial runner still need a physical reproduction.
