from abc import abstractmethod

import smach
from controls_core.PIDManager import PIDManager
from dependency_injector import providers

# providers is used to create instances of a class/object
from tasks.task_state import TaskState


class Task(smach.State):
    # Use this to create a single TaskState instance
    task_state_provider = providers.Singleton(TaskState)
    pid_manager_provider = providers.Singleton(PIDManager)

    def __init__(
        self, task_name: str, outcomes, input_keys=[], output_keys=[], io_keys=[]
    ):
        super().__init__(outcomes, input_keys, output_keys, io_keys)

        self.name = task_name
        self.task_state = self.task_state_provider(task_name=task_name)
        self.pid_manager = self.pid_manager_provider()
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

    @property
    def depth(self):
        return self.task_state.depth

    @property
    def logger(self):
        return self.task_state.get_logger()

    @abstractmethod
    def run(self, ud):
        # To be overwritten by a subclass
        pass

    def execute(self, ud):
        # Sets initital_state as current task state
        self.initial_state = self.state
        return self.run(ud)

    def correctVehicle(self, currRPY, targetRPY, currXYZ, targetXYZ):
        return self.pid_manager.correctVehicle(currRPY, targetRPY, currXYZ, targetXYZ)

    def task_complete(self):
        self.targetRPY = [0.0, 0.0, 0.0]
        self.targetXYZ = [0.0, 0.0, 0.0]
        self.currRPY = [0.0, 0.0, 0.0]
        self.currentXYZ = [0.0, 0.0, 0.0]
        self.logger.info("{} COMPLETE, TURNING OFF THRUSTERS...".format(self.name))
        self.correctVehicle(self.currRPY, self.targetRPY, self.currXYZ, self.targetXYZ)

        return "done"


# class ObjectVisibleTask(Task):
#     def __init__(self, image_name, timeout):
#         super().__init__(
#             task_name=image_name,
#             outcomes=["undetected", "detected"],
#             input_keys=["image_name"],
#             output_keys=["image_name"],
#         )

#         self.image_name = image_name
#         self.timeout = timeout

#     def run(self, ud):
#         milli_secs = 10
#         rate = self.task_state.create_rate(milli_secs)
#         total = 0
#         while total < self.timeout * 1000:
#             if cv_object_position(self.cv_data[self.image_name]) is not None:
#                 return "detected"

#             total += milli_secs
#             rate.sleep()
#         return "undetected"


# class MutableTask(Task):
#     def __init__(self, mutablePose):
#         super().__init__(
#             task_name="mutable_task",
#             outcomes=["done"],
#             input_keys=["x", "y", "z", "roll", "pitch", "yaw"],
#         )
#         # super().__init__(task_name=name, outcomes=['done'], input_keys=['x','y','z','roll','pitch','yaw'])
#         self.mutablePose = mutablePose

#     def run(self, ud):
#         if ud.x != nan and ud.y != nan and ud.z != nan:
#             self.mutablePose.setPoseCoords(ud.x, ud.y, ud.z, ud.roll, ud.pitch, ud.yaw)

#         # Checks if preemption request has been made for current state
#         # Preemption refers to interuppting execution of current state in favor of another
#         if self.preempt_requested():
#             # Perform cleanup and transition to another state
#             self.service_preempt()

#         return "done"


# def cv_object_position(cv_obj_data):
#     pass


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
