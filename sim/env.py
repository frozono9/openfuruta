"""Single-AS5600 Furuta environment. Angles in radians; q_pend=0 is hanging down."""
from collections import deque
from pathlib import Path
import json
import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np

HERE=Path(__file__).resolve().parent

def wrap(x):return (x+np.pi)%(2*np.pi)-np.pi

class FurutaEnv(gym.Env):
    metadata={'render_modes':['rgb_array'],'render_fps':50}
    def __init__(self, task='balance', randomize=True, render_mode=None, config=None):
        if task not in ('balance','swingup'):raise ValueError(task)
        self.cfg=json.loads((HERE/'config.json').read_text())
        if config:self.cfg.update(config)
        self.task=task;self.randomize=randomize;self.render_mode=render_mode
        self.model=mujoco.MjModel.from_xml_path(str(HERE/'furuta.xml'))
        self.model.opt.timestep=self.cfg['physics_dt']
        self.data=mujoco.MjData(self.model)
        self.dt=self.cfg['control_dt'];self.substeps=round(self.dt/self.model.opt.timestep)
        if not np.isclose(self.dt,self.substeps*self.model.opt.timestep):raise ValueError('control_dt must be an integer multiple of physics_dt')
        self.limit=np.deg2rad(self.cfg['arm_limit_deg'])
        self.quantum=2*np.pi/(self.cfg['full_steps_per_rev']*self.cfg['microsteps'])
        self.sensor_quantum=2*np.pi/(2**self.cfg['encoder_bits'])
        self.action_space=spaces.Box(-1.,1.,shape=(1,),dtype=np.float32)
        # sin(q), cos(q), filtered pendulum speed/30, counted arm position/limit,
        # ramp speed/max_speed, previous requested action. No true arm state.
        self.observation_space=spaces.Box(-np.inf,np.inf,shape=(6,),dtype=np.float32)
        self._nominal={n:getattr(self.model,n).copy() for n in ('body_mass','body_inertia','dof_damping','dof_frictionloss')}
        self._renderer=None

    def _sample_encoder(self):
        raw=self.data.qpos[1]+self.np_random.normal(0,self.cfg['encoder_noise_rad'])
        return wrap(np.round(raw/self.sensor_quantum)*self.sensor_quantum)

    def _observation(self):
        return np.array([np.sin(self.measured_angle),np.cos(self.measured_angle),self.measured_velocity/30.,self.command_angle/self.limit,self.command_speed/self.cfg['max_speed_rad_s'],self.last_action],dtype=np.float32)

    def _info(self):
        return {'arm_angle_true':float(self.data.qpos[0]),'arm_angle_counted':self.command_angle,'tracking_error':float(self.command_angle-self.data.qpos[0]),'pendulum_error':float(wrap(self.data.qpos[1]-np.pi)), 'upright':bool(abs(wrap(self.data.qpos[1]-np.pi))<np.deg2rad(10)), 'elapsed_seconds':self.steps*self.dt}

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed);options=options or {}
        for n,a in self._nominal.items():getattr(self.model,n)[:]=a
        self.torque_scale=1.
        if self.randomize:
            frac=self.cfg['randomization_fraction']
            for name in ('arm','pendulum'):
                idx=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_BODY,name)
                factor=self.np_random.uniform(1-frac,1+frac)
                self.model.body_mass[idx]*=factor;self.model.body_inertia[idx]*=factor
            self.model.dof_damping[:]*=self.np_random.uniform(.7,1.3,2)
            self.model.dof_frictionloss[:]*=self.np_random.uniform(.7,1.3,2)
            self.torque_scale=self.np_random.uniform(1-frac,1+frac)
        mujoco.mj_setConst(self.model,self.data);mujoco.mj_resetData(self.model,self.data)
        # Manual centering is required on hardware; initial arm position is known here.
        theta=float(options.get('arm_angle',0.))
        default=np.pi+self.np_random.uniform(-.08,.08) if self.task=='balance' else self.np_random.uniform(-.12,.12)
        alpha=float(options.get('pendulum_angle',default))
        self.data.qpos[:]=[theta,alpha];self.data.qvel[:]=options.get('velocities',[0.,0.])
        self.phase_position=theta;self.command_angle=theta;self.command_speed=0.;self.last_action=0.;self.steps=0
        self.measured_velocity=0.;self.measured_angle=self._sample_encoder()
        delay=int(self.cfg['sensor_delay_steps'])
        self.sensor_queue=deque([self.measured_angle]*(delay+1),maxlen=delay+1)
        mujoco.mj_forward(self.model,self.data)
        return self._observation(),self._info()

    def step(self, action):
        value=np.asarray(action,dtype=float)
        if value.shape!=(1,) or not np.isfinite(value).all():raise ValueError('Action must be one finite number in an array')
        action=float(np.clip(value[0],-1,1));old_action=self.last_action
        c=self.cfg;target=action*c['max_speed_rad_s'];h=self.model.opt.timestep
        for _ in range(self.substeps):
            # Same braking envelope can run using counted steps on the ESP32.
            distance=max(0.,self.limit-abs(self.phase_position))
            braking_speed=max(0.,np.sqrt(2*c['max_accel_rad_s2']*distance)-2*c['max_accel_rad_s2']*h)
            if target*self.phase_position>0:limited_target=np.sign(target)*min(abs(target),braking_speed)
            else:limited_target=target
            self.command_speed+=max(-c['max_accel_rad_s2']*h,min(c['max_accel_rad_s2']*h,limited_target-self.command_speed))
            self.phase_position+=self.command_speed*h
            if abs(self.phase_position)>self.limit:
                self.phase_position=max(-self.limit,min(self.limit,self.phase_position));self.command_speed=0.
            self.command_angle=float(np.round(self.phase_position/self.quantum)*self.quantum)
            # Approximate stepper load-angle torque, NOT an identified electrical model.
            # Periodic stable wells permit missed steps instead of magically correcting them.
            available=c['motor_peak_torque_nm']*self.torque_scale/(1+(abs(self.data.qvel[0])/c['motor_rolloff_rad_s'])**2)
            error=self.command_angle-self.data.qpos[0]
            torque=available*np.sin(c['motor_electrical_cycles_per_rev']*error)-c['motor_damping_nm_s']*self.data.qvel[0]
            self.data.ctrl[0]=max(-available,min(available,torque))
            mujoco.mj_step(self.model,self.data)
        self.steps+=1;self.last_action=action
        self.sensor_queue.append(self._sample_encoder());measured=self.sensor_queue[0]
        derivative=wrap(measured-self.measured_angle)/self.dt
        a=c['velocity_filter_alpha'];self.measured_velocity=(1-a)*self.measured_velocity+a*derivative
        self.measured_angle=measured
        error=wrap(self.data.qpos[1]-np.pi)
        upright=(1+np.cos(error))/2
        reward= float(upright+.5*np.exp(-(error/.2)**2)-.1*(self.data.qpos[0]/self.limit)**2-.005*(self.data.qvel[1]/10)**2-.005*action**2-.01*(action-old_action)**2)
        finite=np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all()
        boundary=abs(self.data.qpos[0])>=self.limit
        fallen=self.task=='balance' and abs(error)>.65
        terminated=bool(boundary or fallen or not finite)
        if terminated:reward-=2.
        truncated=bool(self.steps>=round(c['episode_seconds']/self.dt))
        info=self._info();info['termination_reason']='arm_limit' if boundary else 'fallen' if fallen else 'nonfinite' if not finite else 'time_limit' if truncated else None
        return self._observation(),reward,terminated,truncated,info

    def render(self):
        if self._renderer is None:self._renderer=mujoco.Renderer(self.model,height=720,width=960)
        camera=mujoco.MjvCamera();camera.lookat[:]=[0,.025,.09];camera.distance=.30;camera.azimuth=35;camera.elevation=-20
        self._renderer.update_scene(self.data,camera=camera)
        return self._renderer.render()

    def close(self):
        if self._renderer is not None:self._renderer.close();self._renderer=None
