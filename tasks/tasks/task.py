from task_state import TaskState
from dependency_injector import providers
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry

# providers is used to create instances of a class/object

# smach stands for State Machine for Advanced Robots, provides us with state machines
# A state machine consists of states, state transitions and associated actions
import smach

from abc import abstractmethod

class Task(smach.State):
    # Use this to create a single TaskState instance
    task_state_provider = providers.Singleton(TaskState)

    def __init__(self, task_name : str, outcomes, input_keys=[], output_keys=[], io_keys=[]):
        super().__init__(outcomes, input_keys, output_keys, io_keys)
        
        self.task_state = self.task_state_provider(task_name=task_name)
        self.start_time = None
        self.initial_state = None
        self.output = {}

    # Use as task.state
    @property
    def state(self) -> Odometry:
        return self.task_state.state
    
    @property
    def cv_data(self) -> dict:
        return self.task_state.cv_data
    
    @abstractmethod
    def run(self, ud):
        # To be overwritten by a subclass 
        pass

    def execute(self, ud):
        # Sets initital_state as current task state
        self.initial_state = self.state
        return self.run(ud)
    
    # Invokes task_state's properties to publish stuff
    def publish_desired_pose(self, pose : Pose):
        self.task_state.desired_pose_publisher.publish(pose)

    def publish_desired_twist(self, twist : Twist):
        self.task_state.desired_twist_velocity_publisher.publish(twist)



"""
Notes for smach.State:
outcomes are label for the different possible states for the state machine,
returned at the end of the execute function

input_keys refers to data fed to the state
if input_keys=['foo_input'], then it is accessed via ud.foo_input in execute

output_keys are what the state can modify
if output_keys=['foo_output'], then it is accessed via ud.foo_output in execute
ud.foo_output = 3
"""