import threading
from abc import abstractmethod

import rclpy
from controls_core.PIDManager import PIDManager
from dependency_injector import providers
from tasks.task_state import TaskState
from yasmin import State


class Task(State):
    TASK_STATE_PROVIDER = providers.ThreadSafeSingleton(TaskState)
    PID_MANAGER_PROVIDER = providers.Singleton(PIDManager)

    def __init__(self, outcomes):

        super().__init__(outcomes)

        self.task_state = self.task_state_provider()
        self.pid_manager = self.pid_manager_provider()
        self.start_time = None
        self.initial_state = None
        self.output = {}

    @classmethod
    def init_task_state(cls):
        rclpy.init()
        task_state = cls.TASK_STATE_PROVIDER()
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(task_state)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

    @property
    def currRPY(self):
        return self.task_state.state

    @property
    def currXYZ(self):
        return [0.0, 0.0, self.task_state.depth]

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

    def correctVehicle(self, currRPY, targetRPY, currXYZ, targetXYZ, override_forward_acceleration=None):
        return self.pid_manager.correctVehicle(currRPY, targetRPY, currXYZ, targetXYZ, override_forward_acceleration=None)

    def task_complete(self):
        self.targetRPY = [0.0, 0.0, 0.0]
        self.targetXYZ = [0.0, 0.0, 0.0]
        self.currRPY = [0.0, 0.0, 0.0]
        self.currentXYZ = [0.0, 0.0, 0.0]
        self.logger.info("{} COMPLETE, TURNING OFF THRUSTERS...".format(self.name))
        self.correctVehicle(self.currRPY, self.targetRPY, self.currXYZ, self.targetXYZ)

        return "done"
