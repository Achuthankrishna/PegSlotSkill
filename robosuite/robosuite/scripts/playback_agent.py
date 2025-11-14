#creating dummy agent via huerestics to simulate
import numpy as np
import robosuite as suite
import time
import json
import os 
from robosuite.utils.sim_utils import check_contact
import robosuite.macros as macros
import argparse
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
                # breakpoint()
                print("Closing gripper on peg")
                jaw = self.jaw_close
                if check_contact(env.sim,env.robots[0].gripper['right'].contact_geoms[10],env.objects[0].contact_geoms[0]):
                    self.subtask = 2
            pos = 3.0*(target_pos - gripper_pos)
            print("pos:", pos)
            ori = np.zeros(3)
        #subtask2 : lift the  peg
        elif self.subtask==2:
            print("lifting Peg")
            #target as height
            target_pos = np.array([self.reach_pos[0],self.reach_pos[1],self.lift_height])
            jaw=self.jaw_close
            print("target_pos:", target_pos)
            print("gripper_pos:", gripper_pos)
            if np.allclose(gripper_pos, target_pos, atol=1e-2):
                print("Reached lift height, holding position")
                self.subtask=3
            #update pos
            pos=1.5*(target_pos-gripper_pos)
            #flip the eef by 90 deg by roll
            target_ori=[np.pi/4,0,0]
            ori=0.1*(np.array(target_ori)-np.zeros(3))
        #Subtask 3: Go near the slot
        elif self.subtask==3:
            print("Moving peg to slot")
            #rech2pos as target + some hegith threshold
            target_pos=[self.reach2_pos[0],self.reach2_pos[1],self.reach2_pos[2]+self.lift_height]
            jaw=self.jaw_close
            print("L2 distance for task 3",np.linalg.norm(gripper_pos-target_pos))
            #threshold selection : closer to the target + threshold
            if np.allclose(gripper_pos, target_pos, atol=1e-2):
                self.subtask=4
            pos=1.0*(target_pos-gripper_pos)
            ori=np.zeros(3)

        #subtask4: Rotate along pitch by 120
        elif self.subtask==4:
            print("Bringing closer to placing")
            # target_pos = [self.reach2_pos[0]+0.025,self.reach2_pos[1]-0.03, self.reach2_pos[2]+0.1]
            target_pos = [self.reach2_pos[0]+0.025,self.reach2_pos[1]-0.03, self.reach2_pos[2]+0.1]
            jaw=self.jaw_close
            if np.allclose(gripper_pos, target_pos, atol=1e-2):
                print("Reached lift height, holding position")
                #break the loop
                self.subtask = 5  # Hold position
            pos =1.5*(target_pos - gripper_pos)
            target_ori=[0.0,2*np.pi/3,0.0]
            #Calculate ori error with current ori of gripper
            from scipy.spatial.transform import Rotation as R
            current_rot_mat=env.sim.data.get_site_xmat("gripper0_right_grip_site").reshape(3,3)
            # print(current_rot_mat)
            #RotMat to Euler
            current_euler_ori=R.from_matrix(current_rot_mat).as_euler('xyz')
            orien_error=target_ori-current_euler_ori
            #L2 ori error
            ori_norm=np.linalg.norm(orien_error)
            #Dont rotate much - so stop ori early
            if ori_norm > 2.0:
                #slow update
                ori=0.05*orien_error
            else:
                ori=np.zeros(3)
        #subtask 5: place into the slot
        elif self.subtask==5:
            print("Final Subtask")
            target_pos = [self.reach2_pos[0]-0.02,self.reach2_pos[1]-0.018, self.reach2_pos[2]+0.025]
            jaw=self.jaw_close
            if np.allclose(gripper_pos, target_pos, atol=1e-2):
                print("Placed on slot")
                #break the loop
                jaw = -1.0
                done=True
            pos = 3.0*(target_pos-gripper_pos) #pretty faster
            target_ori = [0,0,0]
            ori= 0.1*(np.array(target_ori) - np.zeros(3))
            from scipy.spatial.transform import Rotation as R
            current_rot_mat = env.sim.data.get_site_xmat("gripper0_right_grip_site").reshape(3,3)
            current_ori = R.from_matrix(current_rot_mat).as_euler('xyz')
            ori_error = target_ori - current_ori
            ori_norm = np.linalg.norm(ori_error)
            print("ori_error:", ori_norm)
            # Stop rotating once near target
            if ori_norm < 1.5:
                ori = -0.05 * ori_error
            else:
                print("Reached target orientation — holding steady")
                ori = np.zeros(3)

        else:
            pos = np.zeros(3)
            ori = np.zeros(3)
            jaw = -1.0
        if env._check_success():
            success=True



        self.step += 1
        return np.concatenate([pos, ori, [jaw]]),done,success

def play_sim(agent,speed=1.0,episode_len=100,log_path="sim_log.json",control_freq=20):
    """
    Main Simulating function
    Args:
        agent : Agent Class
        speed : Simulation speed. Defaults to 1.0.
        episode_len : Length of the episode. Defaults to 100.
        log_path : file name with path. Defaults to "sim_log.json".
        control_freq: Frequency where IK updates are made . Deaults to 20
    """
    #make env

    env=suite.make(
        "CubePlace",
        has_renderer=True,
        has_offscreen_renderer=False,
        use_camera_obs=False,
        control_freq=control_freq,
        robots="PandaRobotiq",

    )
    data=[]
    #set state with current env observation (static state)
    obs=env.reset()
    #sim warmup : Good practice to not make the objects to fall
    for _ in range(30):
        env.sim.step()
    time.sleep(0.1)
    breakpoint()
    for s in range(episode_len):
        print("step_number",s)
        action,done,success=agent.perform_action(obs,env=env)
        #so the action modality is 6 + 1 for jaw
        pose=action[:6]
        jaw=action[6]
        #apply to env and step
        obs,_,_,_=env.step(np.concatenate([pose,[jaw]]))
        #to update per speed

        time.sleep(0.01/speed)
        if success:
            break
    env.close()
    print("Done simulating")

#TestCall
if __name__ == "__main__":
    agent=PickPlaceAgent()
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--sim_speed",
        type=float,
        default=0.002,
        help="Effective simulation time in seconds ; ie - how often env.step() updates"
    )
    parser.add_argument("--episode_length", 
                        type=int, 
                        default=5000,
                        help="Entire episode length")
    parser.add_argument(
        "--log_path",
        type=str,
        default="./",
        help="default log path",
    )
    parser.add_argument("--step_speed", 
                        type=int, 
                        default=1,
                        help="Speed at which playback loop runs")
    args = parser.parse_args()

    macros.SIMULATION_TIMESTEP=args.sim_speed
    play_sim(agent,episode_len=args.episode_length,speed=args.step_speed)
    