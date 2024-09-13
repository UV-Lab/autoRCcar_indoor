# Reinforcement Learning
### Task
- 'avoid-v0'

 This task is the navigation and obstacle avoidance.
### Dependencies
    pip install -e .
    
### Train
```
python train_model.py <suffix>
ex) python train_model.py tqc1
```
```
tensorboard --logdir=log
```
- http://localhost:6006/
### Run
- single goal
```
python run_model.py best_avoid-v0_ppo1
python run_model.py avoid-v0_tqc1
```
- Multiple waypoint
```
cd eval
python eval_model.py best_avoid-v0_ppo1
```
    

