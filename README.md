# Controls Stack

## Setup
Download dependencies with the following code.
```
sudo apt install ros-humble-smach-ros
sudo apt install ros-humble-tf-transformations
pip3 install transforms3d
pip3 install dependency-injector
pip3 install simple_pid
```

## Pool Testing
1. Build and source.
```
colcon build --symlink-install
source install/setup.bash
```
2. Modify `/tasks/movement_test.py` as needed to configure state machine and associated movement tasks.
3. Launch cameras, sensors, etc.
4. Launch base nodes needed for initialising controls stack.
```
ros2 launch launch_files base.launch.py
```
**Note: To reset IMU drift, relaunch `base.launch.py`**
6. Launch movement test.
```
ros2 launch launch_files movement.launch.py
```
   
## Simulation
1. Build and source.
```
colcon build --symlink-install
source install/setup.bash
```
2. Modify `/simulation/movement_test_sim.py` as needed to configure state machine and associated movement tasks.
3. Launch simulation.
```
ros2 launch launch_files simulation.launch.py
```

## How it Works
### Structure
### Flow
### Packages and Scripts
