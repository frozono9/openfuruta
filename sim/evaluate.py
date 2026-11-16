import argparse,json
import numpy as np
from stable_baselines3 import PPO
from train import evaluate
from env import HERE

class ZeroPolicy:
    def predict(self,obs,deterministic=True):return np.zeros(1,dtype=np.float32),None

def main():
    p=argparse.ArgumentParser();p.add_argument('--policy');p.add_argument('--task',default='balance',choices=['balance','swingup']);p.add_argument('--episodes',type=int,default=20);p.add_argument('--seed-base',type=int,default=1000);p.add_argument('--out',default='results/evaluation.json');a=p.parse_args()
    model=PPO.load(a.policy,device='cpu') if a.policy else ZeroPolicy()
    result={'policy':a.policy or 'zero command','task':a.task,'nominal':evaluate(model,a.task,range(a.seed_base,a.seed_base+a.episodes)),'randomized':evaluate(model,a.task,range(a.seed_base+1000,a.seed_base+1000+a.episodes),True),'hardware_validated':False}
    path=HERE/a.out;path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if __name__=='__main__':main()
