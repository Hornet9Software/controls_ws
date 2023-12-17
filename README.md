## Recommended file structure
```
ros2_ws/
└── src
    └── controls_ws
        ├── custom_msgs
        │   ├── CMakeLists.txt
        │   ├── msg
        │   │   ├── CVObject.msg
        │   │   └── ThrusterSpeeds.msg
        │   └── package.xml
        ├── README.md
        └── tasks
            ├── package.xml
            ├── resource
            ├── setup.cfg
            ├── setup.py
            ├── tasks
            │   ├── __init__.py
            │   ├── movement_tasks.py
            │   ├── task.py
            │   ├── task_state.py
            │   ├── test_publish_current_state.py
            │   ├── test_publish_global_pose_task.py
            │   └── utilities.py
            └── test
```
## Setup
Within ros2_ws/src, clone this repo to obtain the above file structure.
    
    colcon build
    source install/setup.bash
    ros2 run tasks test_publish_global_pose
    ros2 run tasks test_publish_current_state
## Program Flow
We first create a global movement task and continously publish our desired pose. We simulate that we have reached our desired pose by publishing our current state, terminating the program.

## Overall Architecture
The idea here is to use smach, a finite state machine library, to represent our tasks. We can string together multiple mini/subtasks, like moving to a specific location, to form an overarching task, like moving to the gate, represented by a state machine.

These states have corresponding outcomes associated with them. Based on the resulting outcome, the state machine can transition to another state. In simple terms, that means after a task is completed/failed, another task will be executed until we finish the final task.

HEAVILY inspired by https://github.com/tylerfeldman321/robosub-ros

### Notes
```spin_once()``` is required for the subscribers of Nodes to listen to topics. Publishers on the other hand work as usual without spinning, though it is not guaranteed we won't run into unexpected problems because of this. Use ```spin_once()``` over ```spin()``` because the latter is blocking, meaning nothing can continue or take place whilst the node spins.

# Installation
```
sudo apt install ros-humble-smach-ros
sudo apt install ros-humble-tf-transformations
sudo pip3 install transforms3d
sudo apt install ros-humble-dependency-injector
```