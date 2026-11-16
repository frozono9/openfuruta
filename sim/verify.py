"""Behavioral checks for geometry, dynamics and the single-sensor contract."""
import json
import numpy as np
import mujoco
from gymnasium.utils.env_checker import check_env
from env import FurutaEnv,wrap,HERE

def main():
    results={};env=FurutaEnv(randomize=False)
    check_env(env,skip_render_check=True);results['gymnasium_api']='pass'
    assert env.model.nq==2 and env.model.nv==2 and env.model.nu==1
    results['two_joints_one_actuator']='pass'
    urdf=mujoco.MjModel.from_xml_path(str(HERE/'furuta.urdf'));assert urdf.nq==2
    results['urdf_import']='pass'
    env.reset(seed=1,options={'pendulum_angle':0})
    p0=env.data.site('coin_center').xpos.copy()
    env.data.qpos[1]=np.pi;mujoco.mj_forward(env.model,env.data)
    p1=env.data.site('coin_center').xpos.copy()
    assert abs((p1[2]-p0[2])-.15)<1e-6
    assert abs(p0[1]-.075)<1e-6
    results['coin_swing_diameter_m']=float(p1[2]-p0[2])
    # True arm position/velocity and true pendulum velocity cannot leak to policy.
    before=env._observation().copy();env.data.qpos[0]+=.4;env.data.qvel[:]=[7,11]
    np.testing.assert_array_equal(before,env._observation())
    results['no_privileged_state_in_observation']='pass'
    a=FurutaEnv();b=FurutaEnv();oa,_=a.reset(seed=19);ob,_=b.reset(seed=19)
    for _ in range(20):
        oa,*_=a.step([.02]);ob,*_=b.step([.02]);np.testing.assert_array_equal(oa,ob)
    results['seed_reproducibility']='pass';a.close();b.close()
    # Passive pendulum at nonzero angle must fall; zero drive does not inject energy.
    env.reset(seed=1,options={'pendulum_angle':.5})
    env.model.opt.enableflags|=int(mujoco.mjtEnableBit.mjENBL_ENERGY)
    mujoco.mj_forward(env.model,env.data);e0=float(sum(env.data.energy))
    for _ in range(4000):env.data.ctrl[:]=0.;mujoco.mj_step(env.model,env.data)
    mujoco.mj_forward(env.model,env.data);e1=float(sum(env.data.energy))
    assert e1<=e0+1e-6 and abs(env.data.qpos[1]-.5)>.05
    results['passive_energy_j']={'start':e0,'end':e1}
    # Small positive/negative commands must move arm in corresponding directions.
    tracking=[]
    for sign in [-1,1]:
        env.reset(seed=1,options={'pendulum_angle':0})
        for _ in range(20):env.step([sign*.04])
        assert env.data.qpos[0]*sign>.02
        tracking.append(float(abs(env.command_angle-env.data.qpos[0])))
    assert max(tracking)<.03
    results['small_command_tracking_error_rad']=tracking
    # A forced lost motor cycle is invisible in observation but persists physically.
    env.reset(seed=1,options={'pendulum_angle':0})
    env.data.qpos[0]=2*np.pi/env.cfg['motor_electrical_cycles_per_rev'];mujoco.mj_forward(env.model,env.data)
    for _ in range(100):env.step([0])
    assert abs(env.data.qpos[0]-env.command_angle)>.08
    assert env._observation()[3]==0
    results['lost_steps_not_magically_corrected']='pass'
    env.reset(seed=1,options={'pendulum_angle':np.pi,'arm_angle':env.limit+.1})
    _,_,term,_,info=env.step([0]);assert term and info['termination_reason']=='arm_limit'
    results['physical_arm_limit_termination']='pass'
    env.reset(seed=1,options={'pendulum_angle':np.pi})
    for _ in range(100):
        old=env.command_speed;env.step([1]);assert abs(env.command_speed-old)<=env.cfg['max_accel_rad_s2']*env.dt+1e-10
    results['command_acceleration_limit']='pass'
    env.close()
    swing=FurutaEnv(task='swingup');obs,_=swing.reset(seed=71)
    assert obs[1]>.98
    for k in range(200):
        obs,reward,term,trunc,info=swing.step([.04*np.sin(k/20)])
        assert np.isfinite(obs).all() and np.isfinite(reward)
        assert info['termination_reason']!='fallen'
        if term or trunc:swing.reset()
    swing.close();results['swingup_environment_smoke']='pass (not trained swing-up)'
    (HERE/'results/verification.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
if __name__=='__main__':main()
