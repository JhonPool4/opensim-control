from envs import ArmEnv 
import numpy as np
import matplotlib.pyplot as plt
from utils import Plotter

# ==========
#   MODIFY
# ==========
_INIT_POS = {'r_elbow':np.deg2rad(15)} # intial elbow angle
_DES_POS = {'r_elbow':np.deg2rad(90)} # desired elbow angle

kp = 9 # proportional gain
kd = 2*np.sqrt(6) # derivative gain
# ==========


# create environemnt
arm_model = ArmEnv( sim_time=10,  # simulation time
                    visualize=True, # to visualize the opensim simualtion
                    show_plots=True, # to show control singal and position error
                    init_pos=_INIT_POS, 
                    des_pos=_DES_POS)

for _ in range(1):
    # set initial conditions and reset simulation parameters
    obs, obs_dict = arm_model.reset(verbose=True)
    done = False
    act_biceps = np.zeros(3,) # control singal for biceps
    act_triceps = np.zeros(3,) # control signal for triceps

    while not done:
        # compute error: position and velocity
        e = obs_dict['des_elbow_pos']-obs_dict['med_elbow_pos']
        de = 0.0-obs_dict['med_elbow_vel']

        # compute control law
        u = kp*e + kd*de
        if e>0:
            act_biceps += 0.0001*abs(u)*np.ones(3,)
        else:
            act_triceps += 0.0001*abs(u)*np.ones(3,) 
        
        # apply action
        action=np.concatenate((act_triceps, act_biceps), axis=0)

        # new observations
        obs, obs_dict, done, info = arm_model.step(action=action)



        if info['sim_timesteps']%50 ==0:
            print(f"\n=============")
            print(f"des angle: {obs_dict['des_elbow_pos']:.3f}")
            print(f"med angle: {obs_dict['med_elbow_pos']:.3f}")
            print(f"error angle: {e:.3f}")
            print(f"biceps:{act_biceps[0]}")
            print(f"triceps:{act_triceps[0]}")


    

