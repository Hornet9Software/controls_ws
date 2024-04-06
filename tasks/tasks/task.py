import threading
import time
from abc import abstractmethod

import rclpy
from controls_core.PIDManager import *
from dependency_injector import providers
from tasks.task_state import TaskState
from yasmin import State


class Task(State):
    TASK_STATE_PROVIDER = providers.ThreadSafeSingleton(TaskState)
    PID_MANAGER_PROVIDER = providers.Singleton(NormalPIDManager)
    GATE_PID_MANAGER_PROVIDER = providers.Singleton(GatePIDManager)
    FLARE_PID_MANAGER_PROVIDER = providers.Singleton(FlarePIDManager)
    RUN_TIME_CAP = 5

    def __init__(self, outcomes):

        super().__init__(outcomes)

        self.task_state = self.TASK_STATE_PROVIDER()
        self.pid_manager = self.PID_MANAGER_PROVIDER()
        self.gate_pid_manager = self.GATE_PID_MANAGER_PROVIDER()
        self.flare_pid_manager = self.FLARE_PID_MANAGER_PROVIDER()
        self.start_time = None
        self.initial_state = None
        self.output = {}

    @classmethod
    def init_task_state(cls):
        task_state = cls.TASK_STATE_PROVIDER()

        task_state.set_init_time()
        # executor = rclpy.executors.MultiThreadedExecutor()
        # executor.add_node(task_state)
        # executor_thread = threading.Thread(target=executor.spin, daemon=True)

        executor_thread = threading.Thread(
            target=rclpy.spin, args=(task_state,), daemon=True
        )
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
    def flare_order(self):
        return self.task_state.flare_order

    @property
    def logger(self):
        return self.task_state.get_logger()

    @abstractmethod
    def run(self, blackboard):
        pass

    def execute(self, blackboard):
        self.init_time = time.time()

        while True:
            self.curr_time = time.time()

            if (self.curr_time - self.task_state.get_init_time()) > (self.RUN_TIME_CAP * 60):
                return "end"

            if self.depth > 0:
                return "restart"

            out = self.run(blackboard)
            if out != "running":
                return out

    def clear_old_cv_data(self, object_name, refresh_time=5.0):
        if self.cv_data[object_name] is None:
            return

        msg_time = self.cv_data[object_name]["time"]
        curr_time = time.time()

        if (curr_time - msg_time) >= refresh_time:
            self.task_state.clear_cv_data(object_name)

    def clear_cv_data(self, object_name):
        self.task_state.clear_cv_data(object_name)

    def correctVehicle(
        self,
        currRPY,
        targetRPY,
        currXYZ,
        targetXYZ,
        override_forward_acceleration=None,
        pid_type="normal",
    ):
        if pid_type == "normal":
            return self.pid_manager.correctVehicle(
                currRPY, targetRPY, currXYZ, targetXYZ, override_forward_acceleration
            )
        elif pid_type == "gate":
            return self.gate_pid_manager.correctVehicle(
                currRPY, targetRPY, currXYZ, targetXYZ, override_forward_acceleration
            )
        elif pid_type == "flare":
            return self.flare_pid_manager.correctVehicle(
                currRPY, targetRPY, currXYZ, targetXYZ, override_forward_acceleration
            )
