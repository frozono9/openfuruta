# Simulation

The MuJoCo model and URDF use the published STEP geometry. The Gymnasium environment models one pendulum encoder and a step-counted arm. The second encoder and the motor current loop are outside that environment's observation model. Estimated inertias, a rigid arm coupling and simplified stepper dynamics make this useful for control experiments. Mesh collision is disabled.

## Run and train

Use Python 3.12. From the repository root:

~~~sh
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r sim/requirements.txt
python sim/verify.py
mjpython sim/view.py --live
~~~

On Linux and Windows use **python sim/view.py --live**. On Windows activate with **.venv\Scripts\activate**. The viewer shows passive motion without a policy.

~~~sh
python -m pip install -r sim/requirements-learning.txt
mjpython sim/view.py --live --policy sim/results/balance_policy.zip
python sim/evaluate.py --policy sim/results/balance_policy.zip
python sim/train.py --task balance --steps 300000 --out runs/my_balance
~~~

The included PPO policy starts near upright and was trained on an earlier geometry export. Its [evaluation](results/final_evaluation.json) is the historical run on that model. The commands above use the released geometry. To train from hanging down, use **--task swingup** with a new output directory. Policies in this section are for the simulator.

## Reproduce the controller test

A host C++ compiler named **clang++** and Python 3 are sufficient for the native simulation.

~~~sh
python v2/hil/validate_timed_braking_round2.py
~~~

The command builds the released ESP32 loop against simulated peripherals and checks the earlier v2 controller and frozen predictive-braking variant on the same plant cases. It writes **timed_braking_reproduced.json**. The [archived result](../v2/hil/timed_braking_round2_validation.json) is included. The independent, header-only prototype can also run by itself:

~~~sh
clang++ -O3 -std=c++17 -I v2/core v2/sim/sim.cpp -o /tmp/openfuruta-plant
/tmp/openfuruta-plant mc 20 1
~~~

The paired PI/v2 comparison uses nine [minimal recorded entry states](../firmware/closed_loop/tests/) and a hanging start. Run **python v2/hil/compare.py** for this longer experiment.

## Regenerate models and print files

The STEP is the source of geometry. Import a modified assembly by passing its path to the import command. The explicit motor and hinge origins and component mapping in **build_model.py** must match the revised assembly, and the report hashes the published STEP.

~~~sh
python -m pip install -r sim/requirements-step.txt
python sim/import_step.py
python sim/build_model.py
python hardware/print/export.py
python sim/verify.py
~~~

The importer writes assembly-space millimetre meshes and a GLB to **sim/source/meshes/**. The builder uses those meshes for the model and converts them to metres in **sim/assets/**. The print exporter selects the four plastic parts and orients them on the bed in millimetres.
