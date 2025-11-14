# PegSlotSkill
This repo is a supporting simulation software for peg slot skill which are used for data generation, testing and eval of robot skills learning. The current simulation architecture adopts Robosuite which can be supported by ,odular architectures like mimicgen for data generation purposes and robomimic for RL training. The repo also consists of an heurestic agent that can simulate the action purely based on adjustments and control.

## Installation steps:
The installation heirarchy will follow from creating a new conda env and installing all required repos induvidually:
1. Set up conda environment:l
```sh
conda create -c conda-forge -n robot-skills-sim python=3.10
```
2. Activate conda environment:
```sh
conda activate robot-skills-sim
```
3. Clone and setup robosuite dependency:

```sh
cd robosuite
pip install -e .
pip install -r requirements.txt
pip install -r requirements-extra.txt
```
(optional: if running into issues with numba/numpy, run: conda install -c numba numba=0.56.4 -y)

4. Re-Install Numpy and Numba versions (If faced with error):
```sh
pip install numpy==1.23.3 numba==0.56.4
```
## Teleoperation Skill
Users can teleop the robot + gripper using the following scripts which saves the recording as Hdf5 file consisting of states and observations required for RL and also mimicgen. The base saving format is robomimic.

Before running teleop ,users need to compile any new task that's being aded to set of registered task . This can be done using:
```sh
 python -m py_compile cube_place.py 
```
Now,we can run teleop on CubePlace Task
```sh
python robosuite/scripts/collect_human_demonstrations.py --device keyboard --environment CubePlace --robots PandaRobotiq --directory XXXX/PegSlotSkill/robosuite_data_2"
```
where XXX can be the directory user wants to save the data in. Users can choose from a list of tasks .

The teleop-ed data can be played back using a playback script borrowed from Robocasa. 
```sh
python robosuite/scripts/playback_dataset.py --dataset XXXX/robosuite_data_2/demo_2025-11-XXXX.hdf5  --use-actions --n 1
```
here `use-actions` is used to step the ennvironments with the actions stored in the H5 file. 
![Video](./robosuite_data_2/use_action.gif)

