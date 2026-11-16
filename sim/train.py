"""Train a CPU PPO policy. Checkpoints are simulation-only until calibrated."""
from pathlib import Path
import argparse,json,time,shutil
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.logger import configure
from env import FurutaEnv,HERE

def evaluate(model,task,seeds=range(100,110),randomize=False):
    env=FurutaEnv(task=task,randomize=randomize);rows=[]
    for seed in seeds:
        obs,info=env.reset(seed=seed);reward_sum=0.;upright=0;max_error=0.
        while True:
            action,_=model.predict(obs,deterministic=True)
            obs,reward,term,trunc,info=env.step(action)
            reward_sum+=reward;upright+=info['upright'];max_error=max(max_error,abs(info['tracking_error']))
            if term or trunc:break
        rows.append({'seed':seed,'return':reward_sum,'seconds':info['elapsed_seconds'],'upright_fraction':upright/env.steps,'completed':bool(trunc and not term),'max_tracking_error_rad':max_error,'end':info['termination_reason']})
    env.close()
    return {'episodes':rows,'mean_seconds':float(np.mean([r['seconds'] for r in rows])),'mean_upright_fraction':float(np.mean([r['upright_fraction'] for r in rows])),'completion_rate':float(np.mean([r['completed'] for r in rows]))}

def main():
    p=argparse.ArgumentParser();p.add_argument('--steps',type=int,default=300_000);p.add_argument('--task',choices=['balance','swingup'],default='balance');p.add_argument('--seed',type=int,default=7);p.add_argument('--envs',type=int,default=4);p.add_argument('--out',default='runs/balance');p.add_argument('--resume');p.add_argument('--nominal',action='store_true');a=p.parse_args()
    torch.set_num_threads(1)
    out=HERE/a.out;out.mkdir(parents=True,exist_ok=True)
    env=DummyVecEnv([lambda:Monitor(FurutaEnv(task=a.task,randomize=not a.nominal)) for _ in range(a.envs)])
    env.seed(a.seed)
    if a.resume:model=PPO.load(a.resume,env=env,device='cpu')
    else:model=PPO('MlpPolicy',env,learning_rate=3e-4,n_steps=512,batch_size=256,n_epochs=10,gamma=.995,gae_lambda=.95,ent_coef=.005,policy_kwargs={'net_arch':dict(pi=[64,64],vf=[64,64])},seed=a.seed,device='cpu',verbose=1)
    model.set_logger(configure(str(out),['csv','stdout']))
    for filename in ('config.json','furuta.xml','env.py','train.py','model_report.json','requirements-lock.txt'):
        shutil.copy2(HERE/filename,out/filename)
    (out/'arguments.json').write_text(json.dumps(vars(a),indent=2))
    start=time.time()
    model.learn(total_timesteps=a.steps,callback=CheckpointCallback(save_freq=max(1,50000//a.envs),save_path=str(out),name_prefix='checkpoint'),reset_num_timesteps=not bool(a.resume))
    model.save(out/'policy')
    report={'training_seconds':time.time()-start,'total_timesteps':model.num_timesteps,'task':a.task,'nominal':evaluate(model,a.task),'randomized':evaluate(model,a.task,range(200,210),True),'hardware_validated':False}
    (out/'evaluation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));env.close()
if __name__=='__main__':main()
