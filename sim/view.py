"""Use mjpython on macOS for --live. Offscreen video uses regular Python."""
import argparse,time
import numpy as np
import mujoco
from env import FurutaEnv,HERE

def main():
    p=argparse.ArgumentParser();p.add_argument('--policy');p.add_argument('--task',choices=['balance','swingup'],default='balance');p.add_argument('--live',action='store_true');p.add_argument('--video',default='results/preview.mp4');p.add_argument('--seconds',type=float,default=8);p.add_argument('--seed',type=int,default=123);a=p.parse_args()
    env=FurutaEnv(task=a.task if a.policy else 'swingup',randomize=False,render_mode='rgb_array')
    if a.policy:
        from stable_baselines3 import PPO
        model=PPO.load(a.policy,device='cpu')
    else:model=None
    obs,_=env.reset(seed=a.seed,options=None if model else {'pendulum_angle':.8})
    def step():
        nonlocal obs
        action=model.predict(obs,deterministic=True)[0] if model else np.array([0.],dtype=np.float32)
        obs,_,term,trunc,_=env.step(action)
        if term or trunc:obs,_=env.reset(options=None if model else {'pendulum_angle':.8})
    if a.live:
        import mujoco.viewer
        with mujoco.viewer.launch_passive(env.model,env.data) as viewer:
            viewer.cam.lookat[:]=[0,.025,.09];viewer.cam.distance=.30;viewer.cam.azimuth=35;viewer.cam.elevation=-20
            while viewer.is_running():
                start=time.monotonic();step();viewer.sync();time.sleep(max(0,env.dt-(time.monotonic()-start)))
    else:
        import imageio.v2 as imageio
        target=HERE/a.video;target.parent.mkdir(parents=True,exist_ok=True)
        with imageio.get_writer(target,fps=50) as writer:
            for k in range(round(a.seconds/env.dt)):
                step()
                if k%2==0:
                    frame=env.render()
                    from PIL import Image,ImageDraw,ImageFont
                    image=Image.fromarray(frame);draw=ImageDraw.Draw(image)
                    draw.rectangle((0,0,960,48),fill=(20,28,36))
                    label=f"{'PPO '+a.task if model else 'Passive release'} | episode time {env.steps*env.dt:.2f} s | simulation only"
                    draw.text((16,15),label,fill='white',font=ImageFont.load_default(size=18))
                    frame=np.asarray(image);writer.append_data(frame)
                    if k==0:imageio.imwrite(target.with_suffix('.png'),frame)
        print(target)
    env.close()
if __name__=='__main__':main()
