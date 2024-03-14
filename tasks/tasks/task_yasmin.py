from yasmin import State

from abc import abstractmethod
from controls_core.PIDManager import PIDManager
from dependency_injector import providers

from tasks.task_state import TaskState

class Task(State):
    task_state_provider = providers.ThreadSafeSingleton(TaskState)
    pid_manager_provider = providers.Singleton(PIDManager)


    def __init__(self, task_name : str, outcomes):

        super().__init__(outcomes)

        self.name = task_name
        self.task_state = self.task_state_provider()
        self.pid_manager = self.pid_manager_provider()
        self.start_time = None
        self.initial_state = None
        self.output = {}

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
    def execute(self, blackboard):
        # To override
        pass

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