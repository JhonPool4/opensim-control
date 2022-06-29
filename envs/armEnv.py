import os
from utils.plotter import Plotter
from utils.color_print import print_warning
import opensim as osim
import numpy as np

_JOINT_LIST = ["r_elbow"] 
_AXIS_LIST = ["x", "y"]
_MUSCLE_LIST = ["TRIlong", "TRIlat", "TRImed", \
                "BIClong", "BICshort", "BRA"]

_INIT_FRAME = {'x':-0.013, 'y':0.51} 
_OBS_LIST = ['des_elbow_pos',
            'med_elbow_pos',
            'med_elbow_vel',
            'TRIlong_act',
            'TRIlat_act',
            'TRImed_act',
            'BIClong_act',
            'BICshort_act',
            'BRA_act']

class ArmEnv(object):
    def __init__(self, sim_time=3,
                integrator_accuracy = 1e-5, 
                step_size=0.01, 
                visualize=False,
                show_plots=False,
                init_pos = None,
                des_pos = None):
        # simulation parameters
        self._int_acc = integrator_accuracy
        self._step_size = step_size
        self._show_plots = show_plots
        self._visualize = visualize
        self._max_sim_timesteps = sim_time/step_size
        self._sim_timesteps = 0 # current simulation steps 

        # RL environemnt parameters
        self._des_pos = des_pos # desried joint configuraiton
        self._init_pos = init_pos # initial joint configuraiton 



        # create model from .osim file
        self._model = osim.Model(os.path.join(os.path.dirname(__file__), '../models/Arm2D/arm2dof6musc.osim') )
        # enable the visualizer
        self._model.setUseVisualizer(self._visualize)  
        # animation of muscle's activation
        if self._visualize and self._show_plots:
            self._plots = Plotter(  nrows=2, 
                                    ncols=2,
                                    max_simtime=sim_time,
                                    headers=['triceps', 'biceps','position error', 'velocity error'], 
                                    labels=['act','act','rad', 'rad/s'])
        
        # Body: https://simtk.org/api_docs/opensim/api_docs/classOpenSim_1_1Body.html
        # humerus link
        goal_body = osim.Body("goal_body", # name
                                0.0001, # mass
                                osim.Vec3(0), # center of mass
                                osim.Inertia(1,1,1,0,0,0)) # inertia's moment [Ixx Iyy Izz Ixy Ixz Iyz]
        # add display geometry
        _geom = osim.Ellipsoid(0.025,0.025,0.025)
        _geom.setColor(osim.Green)
        goal_body.attachGeometry(_geom)                                
        # PlanarJoint: https://simtk.org/api_docs/opensim/api_docs/classOpenSim_1_1PlanarJoint.html
        self.goal_joint = osim.PlanarJoint("goal_joint", # name
                                self._model.getGround(), # parent frame
                                osim.Vec3(0,0,0), # location
                                osim.Vec3(0,0,0), # rotation
                                goal_body, # child frame
                                osim.Vec3(0,0,-0.25), # location
                                osim.Vec3(0,0,0)) # rotation           

        # add goal body and joint to model
        self._model.addBody(goal_body)
        self._model.addJoint(self.goal_joint)
        
        # get handles for model parameters
        self._muscles = self._model.getMuscles() # get muscle
        self._bodies = self._model.getBodySet() # get bodies
        self._joints = self._model.getJointSet() # get joints
        self._markers = self._model.getMarkerSet() # get markers  
        

        # add control to each muscle
        # PrescribedController: https://simtk.org/api_docs/opensim/api_docs/classOpenSim_1_1PrescribedController.html
        # basic ontrol template
        self._brain = osim.PrescribedController()
        # quantity of muscles
        self._n_muscles = self._muscles.getSize()

        # Add muscle-actuators with constant-type function
        for idx in range(self._n_muscles):
            self._brain.addActuator(self._muscles.get(idx)) # add actuator for each muscle
            self._brain.prescribeControlForActuator(idx, osim.Constant(1.0)) # add a function to each muscle-actuator

        # add muscle's controllers to model
        self._model.addController(self._brain)
        # get func handle
        self._control_functions=self._brain.get_ControlFunctions()

        # initialize the model and check model consistency (important!)
        self._model.initSystem()
        self._state = None  # model's state
        self._manager = None # model's manager


        
    def get_observations(self):
        # compute forces: muscles
        self._model.realizeAcceleration(self._state)
        # bullet angle
        # - pos
        # joint elbow
        # - pos
        # - vel       
        # muscles
        # - triceps
        # - biceps

        # observation list
        obs = []
        obs_dict={}

        # desired wrist position
        for val in self._des_pos.values():
            obs.append(val)
        
        # joint position and velocity
        for joint_name in _JOINT_LIST:
            obs.append(self._joints.get(joint_name).getCoordinate().getValue(self._state))      
            obs.append(self._joints.get(joint_name).getCoordinate().getSpeedValue(self._state))          

        # muscle activation
        for muscle_name in _MUSCLE_LIST:             
            obs.append(self._muscles.get(muscle_name).getActivation(self._state))

        return obs, dict(zip(_OBS_LIST, obs))


    def actuate(self, action):
        """
        @info: apply stimulus to muscles
        """
        if np.any(np.isnan(action)): # safety condition
            action = np.nan_to_num(action)
            action = np.clip(action, 0, 1)
            
        # apply action
        for idx in range(self._control_functions.getSize()):
            # get control function of actuator "idx"
            func = osim.Constant.safeDownCast(self._control_functions.get(idx))
            # apply action
            func.setValue( float(action[idx]) )

    def integrate(self):
        # update simulation step
        self._sim_timesteps = self._sim_timesteps + 1

        # compute next dynamic configuration
        self._state = self._manager.integrate(self._step_size*self._sim_timesteps)        

    def step_model(self, action):   
        self.actuate(action)
        self.integrate()        

    def reset_manager(self):
        self._manager = osim.Manager(self._model)
        self._manager.setIntegratorAccuracy(self._int_acc)
        self._manager.initialize(self._state)


    def reset_model(self, init_pos=None, bullet_pos=None):
        # fixed shoulder
        self._joints.get("r_shoulder").upd_coordinates(0).setDefaultLocked(True)

        # set initial position
        if init_pos is not None:
            for joint_name in _JOINT_LIST:
                self._joints.get(joint_name).upd_coordinates(0).setDefaultValue(init_pos[joint_name])
                self._joints.get(joint_name).upd_coordinates(0).setDefaultSpeedValue(0.0)
                self._joints.get(joint_name).upd_coordinates(0).setDefaultClamped(True)
        
        # compute initial state (important!)
        self._state = self._model.initializeState()
        if bullet_pos is not None:
            #_joints = self._model.getJointSet() # get joints
            #goal_joint = self._model.getJointSet().get("goal_joint")
            self.goal_joint.get_coordinates(1).setValue(self._state, _INIT_FRAME['x']+0.24*np.sin(bullet_pos['r_elbow']), False)
            self.goal_joint.get_coordinates(2).setLocked(self._state, False)
            self.goal_joint.get_coordinates(2).setValue(self._state, _INIT_FRAME['y']-0.24*np.cos(bullet_pos['r_elbow']), False)
            self.goal_joint.get_coordinates(2).setLocked(self._state, True)

        # compute length of fibers based on the state (important!)
        self._model.equilibrateMuscles(self._state)
        # simulation parameters
        self._sim_timesteps = 0
        self._state.setTime(self._sim_timesteps)
        # forward dynamics manager
        self.reset_manager()

    def reset(self, verbose=False):

        # reset model variables
        self.reset_model(init_pos=self._init_pos, bullet_pos=self._des_pos) 

        # get observations
        obs, obs_dict = self.get_observations()
        if verbose:
            print(f"init elbow angle: {self._init_pos['r_elbow']:.3f}")
            print(f"des elbow angle: {self._des_pos['r_elbow']:.3f}")

        # muscle's activation
        if self._visualize and self._show_plots:
            self._plots.reset()

        return obs, obs_dict



    def step(self, action):
        
        # apply action
        self.step_model(action=action)

        # get environemnt observations
        obs, obs_dict = self.get_observations()

        # plots
        if self._visualize and self._show_plots:
            e = obs_dict['des_elbow_pos']-obs_dict['med_elbow_pos']
            de = 0.0-obs_dict['med_elbow_vel']
            self._plots.add_data(time=self._sim_timesteps*self._step_size, data=np.array([action[0],action[5],e, de]))
            if self._show_plots and self._sim_timesteps==(self._max_sim_timesteps-1):
                self._plots.update_figure()     


        # terminal condition: nan values
        if np.isnan(obs).any(): # check is there are nan values 
            print_warning(f"terminal state for nan observations")
            obs = np.nan_to_num(obs)
            obs = np.clip(obs, 0, 1)
            return obs, obs_dict, True, {'sim_timesteps':self._sim_timesteps}

        # terminal condition: max simulation steps reached
        if not self._sim_timesteps < self._max_sim_timesteps:
            print_warning(f"terminal state for maximum simulation steps")
            return obs, obs_dict, True, {'sim_timesteps':self._sim_timesteps}

              

        # all fine
        return obs, obs_dict, False, {'sim_timesteps':self._sim_timesteps}
        