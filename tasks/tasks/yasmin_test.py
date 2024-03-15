from tasks.task_yasmin import Task
from yasmin import StateMachine
import time

import rclpy
from rclpy.node import Node 

class SimpleTask(Task):
    def __init__(self):
        super().__init__(task_name="simple", outcomes=["done"])
    
    def execute(self, blackboard):
        print(f"Executing task {self.name}")
        time.sleep(3)

        return "done"
    
class TaskNode(Node):
    def __init__(self):
        super().__init__("task_node")

        sm = StateMachine(outcomes=["done"])

        sm.add_state("SIMPLE", SimpleTask(), transitions={"done":"done"})

        outcome = sm()
        print(outcome)

def main(args=None):

    print("yasmin_demo")
    rclpy.init(args=args)
    node = TaskNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

