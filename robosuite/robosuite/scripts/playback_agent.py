#creating dummy agent via huerestics to simulate
import numpy as np
import robosuite as suite
import time
import json
import os 
from robosuite.utils.sim_utils import check_contact

class PickPlaceAgent:
    """
    Scripted agent for peg-in-hole test with subtasks:
    1. Reach peg and close gripper
    2. Lift peg
    3. rotate arm to match slot config
    4. bring arm down closer to hole pose
    5. release gripper
    """
    def __init__(self,lift_height=0.15, jaw_open=0.0, jaw_close=0.5):
        self.step=0
        self.reach_pos=None
        self.lift_height = lift_height
        self.jaw_open = jaw_open
        self.jaw_close = jaw_close
        self.reach2_pos=None
        self.subtask = 1
    
    def perform_action(self,state,env=None):
        """
        Performs actions via 5 subtask to place cube on peg given fixed location
        Args:
            state: Simulation State
            env :Sim Environment Variable. Defaults to None.
        """
        success = False
        done = False
        if self.reach_pos is None:
            if env is not None:
                #location of cube and cuboid slot
                self.reach_pos=env.sim.data.get_site_xpos("peg_default_site").copy() 
                self.reach2_pos = env.sim.data.get_site_xpos("slot_default_site").copy()
        #using right griopper as site since its one that closes
        gripper_pos = env.sim.data.get_site_xpos("gripper0_right_grip_site")

        # print(self.reach_pos,gripper_pos)

        if self.subtask == 1:
            target_pos = [self.reach_pos[0]+0.01,self.reach_pos[1], self.reach_pos[2]-0.01]
            jaw = self.jaw_open
            # Close jaw when near peg
            print("Norm dist between gripper and slot",np.linalg.norm(gripper_pos - target_pos))
            if np.allclose(gripper_pos, target_pos, atol=1e-2):
                breakpoint()
                print("Closing gripper on peg")
                jaw = self.jaw_close
                if check_contact(env.sim,env.robots[0].gripper['right'].contact_geoms[10],env.objects[0].contact_geoms[0]):
                    self.subtask = 2
            pos = 3.0*(target_pos - gripper_pos)
            print("pos:", pos)
            ori = np.zeros(3)
        self.step += 1
        return np.concatenate([pos, ori, [jaw]]),done,success

#TestCall
if __name__ == "__main__":
    agent=PickPlaceAgent()
    