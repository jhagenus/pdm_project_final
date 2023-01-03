import gym
import numpy as np
import os
from urdfenvs.urdf_common.bicycle_model import BicycleModel
from urdfenvs.urdf_common.generic_robot import GenericRobot
from urdfenvs.urdf_common.holonomic_robot import HolonomicRobot


"""
Classes use inheritance, there is a bycicle model already defined which we can use
If we want to decide to create our own bycicle model we can use the GenericRobot
class and use that as inhertiance. The path to and the import are already
define up top to make it easier to switch if we want to.
"""
#switch BicycleModel and HolonomicRobot around for quick switching
class PDM_Model(HolonomicRobot):
    def __init__(self, mode,urdf):
        # for some reason the .urdf file search is different for the prius than for the other generic .urdf files
        self.check = urdf
        if urdf == None:
            n = 2
            urdf_file = os.path.join(os.path.dirname(__file__), 'prius.urdf')
            self._scaling: float = 0.3
            super().__init__(n, urdf_file, mode)
            self._wheel_radius = 0.31265
            self._wheel_distance = 0.494
            self._spawn_offset: np.ndarray = np.array([-0.435, 0.0, 0.05])
        else:
            self._urdf = urdf
            # search for urdf in package if not found in cwd
            if not os.path.exists(urdf):
                root_dir = os.path.dirname(os.path.abspath(__file__))
                urdf = None
                for root, _, files in os.walk(root_dir):
                    for file in files:
                        if file == self._urdf:
                            urdf = os.path.join(root, file)
                if urdf is None:
                    raise Exception(f"the request urdf {self._urdf} can not be found")
                self._urdf = urdf

    def set_joint_names(self):
        """Set joint indices.

        For the bicycle model robots, the steering joints and the forward joints
        are determined.
        """

        # allows to switch between holonomic model and bycycle model quickly for testing
        if self.check== None:
            self._robot_joints = [2, 4]
            self._steering_joints = [2, 4]
            self._forward_joints = [3, 5, 6, 7]
        else:
            self._joint_names = [joint.name for joint in self._urdf_robot._actuated_joints]
    
    # not sure what this does
    def set_acceleration_limits(self):
        if self.check == None:
            acc_limit = np.array([1.0, 1.0])
            self._limit_acc_j[0, :] = -acc_limit
            self._limit_acc_j[1, :] = acc_limit
        else:
            acc_limit = np.array(
            [1.0, 1.0, 15.0, 15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0, 1.0, 1.0]
        )
        self._limit_acc_j[0, :] = -acc_limit[0 : self.n()]
        self._limit_acc_j[1, :] = acc_limit[0 : self.n()]


    def check_state(self, pos, vel):
        if self.check == None:
            if (
                not isinstance(pos, np.ndarray)
                or not pos.size == self.n() + 1
            ):
                pos = np.zeros(self.n() + 1)
            if not isinstance(vel, np.ndarray) or not vel.size == self.n():
                vel = np.zeros(self.n())
            return pos, vel   
        else:
            if not isinstance(pos, np.ndarray) or not pos.size == self.n():
                center_position = (self._limit_pos_j[0] + self._limit_pos_j[1])/2
                pos = center_position
            if not isinstance(vel, np.ndarray) or not vel.size == self.n():
                vel = np.zeros(self.n())
            return pos, vel
   




def run_test_robot(n_steps=1000, render=False, goal=True, obstacles=True):
    
    setUrdf = "pointRobot.urdf" #None 

    #I get the following error when running setUrdf with None param:
    #ValueError: string is not a file: /home/jessedolfin/Delft/PDM/Project/gym_envs_urdf/examples/pdm_project_final/meshes/hybrid_body.obj

    robots = [
        PDM_Model(mode="vel",urdf = setUrdf),
    ]

    env = gym.make(
            "urdf-env-v0",
            dt=0.01, robots=robots, render=render
        )

    pos0 = np.array([0.0, 0.0, 0.0])
    action = np.array([0.,0.,0.])
    env.reset(pos=pos0)

    env.add_walls()

    while(True):
        ob, _, _, _ = env.step(action)

    return None

run_test_robot(render=True)   
