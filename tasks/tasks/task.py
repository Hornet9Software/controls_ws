from task_state import TaskState
from dependency_injector import providers
# providers is used to create instances of a class/object

# smach stands for State Machine for Advanced Robots, provides us with state machines
# A state machine consists of states, state transitions and associated actions
import smach

from abc import abstractmethod

class Task(smach.State):
    # Use this to create a TaskState instance
    task_state_provider = providers.Singleton(TaskState)
    def __init__(self, outcomes=..., input_keys=..., output_keys=..., io_keys=...):
        super().__init__(outcomes, input_keys, output_keys, io_keys)
        
        self.task_state = self.task_state_provider()
        self.start_time = None
        self.initial_state = None
        self.output = {}

    # Use as task.state
    @property
    def state(self):
        return self.task_state.state
    
    @property
    def cv_data(self):
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
    def publish_desired_pose(self, pose):
        self.task_state.desired_pose_publisher.publish(pose)

    def publish_desired_twist(self, twist):
        self.task_state.desired_twist_velocity_publisher.publish(twist)