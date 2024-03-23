# autoRCcar Reinforcement Learning
### Environment
- autoRCcar_straight
- autoRCcar_waypoint
  
### Dependencies
    pip install requriments.txt
    pip install -e .
    
### Train
    python train.py

    tensorboard --logdir=log
    http://localhost:6006/

### Run
    python run_model.py <model_name>
    ex) python run_model.py best_waypoint_v1

    python eval_model.py <model_name>
    

