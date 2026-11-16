"""Export deterministic PPO actor as plain JSON; verify independent NumPy inference."""
import argparse,json
import numpy as np
import torch
from stable_baselines3 import PPO
from env import HERE

def main():
    p=argparse.ArgumentParser();p.add_argument('policy');p.add_argument('--out',default='results/actor.json');a=p.parse_args()
    model=PPO.load(a.policy,device='cpu');net=model.policy
    layers=[]
    for module in [*net.mlp_extractor.policy_net,net.action_net]:
        if isinstance(module,torch.nn.Linear):layers.append({'type':'linear','weight':module.weight.detach().numpy().tolist(),'bias':module.bias.detach().numpy().tolist()})
        elif isinstance(module,torch.nn.Tanh):layers.append({'type':'tanh'})
        else:raise ValueError(f'Unsupported layer {module}')
    def predict(x):
        for layer in layers:
            if layer['type']=='linear':x=x@np.asarray(layer['weight']).T+np.asarray(layer['bias'])
            else:x=np.tanh(x)
        return np.clip(x,-1,1)
    obs=np.random.default_rng(44).uniform(-1,1,(100,6)).astype(np.float32)
    expected=model.predict(obs,deterministic=True)[0];actual=predict(obs)
    error=float(np.max(np.abs(expected-actual)));assert error<1e-5
    out={'layers':layers,'output_clip':[-1,1],'observation_order':['sin_pendulum_angle','cos_pendulum_angle','filtered_pendulum_velocity_div_30','counted_arm_angle_div_limit','ramp_speed_div_max_speed','previous_action'],'angle_convention':'pendulum 0=down, pi=up; arm manually centered at 0','normalization':'fixed scales above; no running normalization','max_export_error':error,'hardware_validated':False}
    path=HERE/a.out;path.parent.mkdir(exist_ok=True,parents=True);path.write_text(json.dumps(out,indent=2));print(f'{path}: max prediction error {error:.3g}')
if __name__=='__main__':main()
