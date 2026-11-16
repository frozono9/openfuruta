# Print and assemble OpenFuruta

Use the [bill of materials](../hardware/BOM.csv) and [STEP assembly](../hardware/cad/OpenFuruta.step) together. The base is 72 × 72 mm. The stepper drives the rotary arm about the vertical axis. The bearing axis lies along the arm, and the counterweight sits 75 mm from the pendulum pivot. With the counterweight down, the pendulum angle is zero.

## Print

| File | Qty | Starting orientation |
| --- | ---: | --- |
| [Bottom](../hardware/print/bottom.stl) | 1 | Broad base face down |
| [Motor holder](../hardware/print/motor-holder.stl) | 1 | Broad motor mounting face down |
| [Rotary arm](../hardware/print/rotary-arm.stl) | 1 | Flat side down |
| [Pendulum arm](../hardware/print/pendulum-arm.stl) | 1 | Flat side down |

Import in millimetres. The supplied meshes already have a face at Z=0. A starting point is a 0.4 mm nozzle, 0.2 mm layers, four walls and 35% infill in PLA or PETG. Preview bridges and use removable supports where the slicer shows unsupported geometry. Keep support scars out of bores and mating surfaces. Use a brim if the pendulum arm lifts. Measure the printed bores and test the interfaces on your printer before final assembly.

## Assemble

1. **Fit the motor to the holder.** Mount the motor from inside the holder with its front shaft pointing up. Install four M3 screws. Select a length that fits the stack-up and the motor's allowable thread engagement. Route the cable through the side opening.
2. **Fit the shaft encoder and bottom.** Mount a diametric magnet concentric with the rear motor shaft. Mount the lower AS5600 rigidly to the base, with its chip centered over the magnet and enough air gap to avoid contact. Use the STEP and your board to select spacers and fasteners before closing the bottom.
3. **Fit the rotary arm.** Use the front shaft and arm socket as shown in the STEP. The prototype uses a friction fit. Test for play and mark both sides of the joint so slip is visible. Correct any movement at the joint before a powered trial. The rear encoder reads the shaft, so coupling slip can move the arm without a corresponding encoder error.
4. **Fit the hinge.** Seat the 8 × 22 × 7 mm bearing and insert the pendulum journal. The CAD pocket is about 22.4 mm and the journal about 7.8 mm. Measure the printed fit. Retain the outer race in its seat and the journal in the inner race without loading the bearing axially. Check that the pendulum swings freely. Keep adhesive out of the bearing if you use it for retention.
5. **Fit the coin and hinge encoder.** Retain the coin securely in the recess. Mount the second diametric magnet so it rotates with the pendulum. Mount the upper AS5600 to the driven arm facing the magnet, with the chip centered on the pivot. Check clearance through a full pendulum revolution.
6. **Route the harness and secure the base.** Strain-relieve the upper sensor cable at both ends of its service loop. Anchor the base on a rigid surface. With motor power disconnected, sweep both axes by hand and mark the arm center where the cable has slack in both directions. Confirm cable and part clearance for the firmware's full ±100° commanded arm envelope around that center before running it.

Continue with the [wiring guide](../hardware/electronics/README.md).
