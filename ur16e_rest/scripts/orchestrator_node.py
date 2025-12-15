import rclpy
from rclpy.node import Node
from custom_ros_messages.srv import UR16BehaviorTrigger, EthernetMotor
from ur16e_rest.scripts.core import RESTAPI
from std_srvs.srv import Trigger
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ur_manipulation.srv import PlanToPose, ExecutePlan, PlanToJoint
from moveit_msgs.srv import GetCartesianPath
import numpy as np
import time
class orchestrator_node(Node):
    def __init__(self):
        super().__init__('orchestrator_node')

        self.rest_client = self.create_client(UR16BehaviorTrigger, 'ur16e_rest/BehaviorTrigger')
        self.motor_client = self.create_client(EthernetMotor, 'motor_control')
        # self.ee_plan_client = self.create_client(PlanToPose, 'plan_to_pose')
        # self.execute_client = self.create_client(ExecutePlan, 'execute_plan')
        # self.joint_plan_client = self.create_client(PlanToJoint, 'plan_to_joint')

        self.get_logger().info('Waiting for ur16e_rest/BehaviorTrigger service...')
        self.rest_client.wait_for_service()
        self.get_logger().info('ur16e_rest/BehaviorTrigger service available')

        self.get_logger().info('Waiting for motor_control service...')
        self.motor_client.wait_for_service()
        self.get_logger().info('motor_control service available')

        # self.get_logger().info('Waiting for PlanToPose service...')
        # self.ee_plan_client.wait_for_service()
        # self.get_logger().info('PlanToPose service available')

        # self.get_logger().info('Waiting for PlanToJoint service...')
        # self.joint_plan_client.wait_for_service()
        # self.get_logger().info('PlanToJoint service available')

        # self.get_logger().info('Waiting for execute_plan service...')
        # self.execute_client.wait_for_service()
        # self.get_logger().info('execute_plan service available')

    # def plan_to_pose(self, x, y, z, qx, qy, qw, qz):
    #     # Plan to the desired pose
    #     plan_request = PlanToPose.Request()
    #     plan_request.target_pose.header.frame_id = "floor_link"
    #     plan_request.target_pose.pose.position.x = x
    #     plan_request.target_pose.pose.position.y = y
    #     plan_request.target_pose.pose.position.z = z
    #     plan_request.target_pose.pose.orientation.x = qx
    #     plan_request.target_pose.pose.orientation.y = qy
    #     plan_request.target_pose.pose.orientation.z = qz
    #     plan_request.target_pose.pose.orientation.w = qw

    #     future_plan = self.ee_plan_client.call_async(plan_request)
    #     rclpy.spin_until_future_complete(self, future_plan)
    #     return future_plan.result()

    # def plan_to_joint_positions(self, joint_array):
    #     # Plan to the desired joint positions
    #     plan_request = PlanToJoint.Request()
    #     plan_request.joint1 = joint_array[0]
    #     plan_request.joint2 = joint_array[1]
    #     plan_request.joint3 = joint_array[2]
    #     plan_request.joint4 = joint_array[3]
    #     plan_request.joint5 = joint_array[4]
    #     plan_request.joint6 = joint_array[5]
    #     future_plan = self.joint_plan_client.call_async(plan_request)
    #     rclpy.spin_until_future_complete(self, future_plan)
    #     return future_plan.result()

    # def execute_plan(self):
    #     # Execute the planned trajectory
    #     execute_request = ExecutePlan.Request()

    #     future_execute = self.execute_client.call_async(execute_request)
    #     rclpy.spin_until_future_complete(self, future_execute)
    #     return future_execute.result()

    def play_program(self):
        behavior_request = UR16BehaviorTrigger.Request()
        behavior_request.behavior = "play"
        future_behavior = self.rest_client.call_async(behavior_request)
        rclpy.spin_until_future_complete(self, future_behavior)
        return future_behavior.result()

    def motor_on(self):
        motor_request = EthernetMotor.Request()
        motor_request.enable = True
        motor_request.speed = 200

        future_motor = self.motor_client.call_async(motor_request)
        rclpy.spin_until_future_complete(self, future_motor)
        return future_motor.result()

    def motor_off(self):
        motor_request = EthernetMotor.Request()
        motor_request.enable = False
        motor_request.speed = 0

        future_motor = self.motor_client.call_async(motor_request)
        rclpy.spin_until_future_complete(self, future_motor)
        return future_motor.result()

    def contact(self):
        behavior_request = UR16BehaviorTrigger.Request()
        behavior_request.behavior = "contact"
        future_behavior = self.rest_client.call_async(behavior_request)
        rclpy.spin_until_future_complete(self, future_behavior)

        return future_behavior.result()

    def zforce(self):
        behavior_request = UR16BehaviorTrigger.Request()
        behavior_request.behavior = "zforce"
        future_behavior = self.rest_client.call_async(behavior_request)
        rclpy.spin_until_future_complete(self, future_behavior)

        return future_behavior.result()

    def retract(self):
        behavior_request = UR16BehaviorTrigger.Request()
        behavior_request.behavior = "retract"
        future_behavior = self.rest_client.call_async(behavior_request)
        rclpy.spin_until_future_complete(self, future_behavior)

        return future_behavior.result()

    def go_above_screw(self, screw_number):
        behavior_request = UR16BehaviorTrigger.Request()
        behavior_request.behavior = f"above_screw_{screw_number}"
        future_behavior = self.rest_client.call_async(behavior_request)
        rclpy.spin_until_future_complete(self, future_behavior)
        
        return future_behavior.result()

    # def ros2_control(self):
    #     behavior_request = UR16BehaviorTrigger.Request()
    #     behavior_request.behavior = "ROS2Control"

    #     future_behavior = self.rest_client.call_async(behavior_request)
    #     rclpy.spin_until_future_complete(self, future_behavior)
    #     return future_behavior.result()



def main(args=None):
    rclpy.init(args=args)
    node = orchestrator_node()
    print("Orchestrator node started")
    # # screw_1 = [0.313, -0.629, 1.038, -0.699, 0.715, 0.006, -0.000]
    # # screw_2 = [0.414, -0.630, 1.034, -0.692, 0.722, 0.005, -0.004]
    # # screw_3 = [0.513, -0.624, 1.031, -0.680, 0.733, -0.013, -0.004]
    # # screw_4 = [0.612, -0.630, 1.031, -0.668, 0.744, -0.001, 0.013]
    # # screw_5 = [0.714, -0.636, 1.030, -0.695, 0.719, -0.011, -0.008]
    # screw_1_pose = [0.306, -0.624, 1.033, -0.692, 0.722, 0.008, 0.007]
    # screw_2_pose = [0.405, -0.623, 1.033, -0.691, 0.723, 0.003, -0.007]
    # # screw_3_pose = [0.509, -0.631, 1.034, -0.703, 0.711, 0.006, -0.006]
    # # screw_4_pose = [0.607, -0.628, 1.032, -0.698, 0.716, -0.010, -0.011]
    # # screw_5_pose = [0.705, -0.638, 1.032, -0.699, 0.715, 0.008, 0.003]

    # screw_1_joint = [-1.8353263340392054, -1.4010802507400513, -1.4565215867808838, 4.704919815063477, -1.8898323217975062, -1.2103412787066858]
    # screw_2_joint = [-1.8280636272826136, -1.401463508605957, -1.4906918120435257, 4.700307369232178, -1.7259424368487757, -1.3708065191852015]
    # # screw_3_joint = [-1.8825613460936488, -1.331817388534546, -1.4981300842813035, 4.695348739624023, -1.5883172194110315, -1.5409138838397425]
    # # screw_4_joint = [-1.934702058831686, -1.241634488105774, -1.5648957605338474, 4.706243991851807, -1.423706356679098, -1.6918342749225062]
    # # screw_5_joint = [-2.0691048107542933, -1.0467400550842285, -1.5791546307005824, 4.710046291351318, -1.2891066710101526, -1.8309362570392054]

    # screw_bundles = [
    #     (screw_1_pose, screw_1_joint),
    #     (screw_2_pose, screw_2_joint),
    #     # (screw_3_pose, screw_3_joint),
    #     # (screw_4_pose, screw_4_joint),
    #     # (screw_5_pose, screw_5_joint)
    # ]
    # screws = []
    # for screw in screw_bundles:
    #     screw[0][2]+=0.045
    #     screws.append(screw)
    # print("begin for loop")
    # for pose, joint in screws:
    #     # print("\n\n\n")
    #     happy_with_plan = False
    #     # pose = [0.348, -0.328, 1.245, -0.691, 0.722, -0.006, -0.002]
    #     # pose[0] += 0.1
    #     while not happy_with_plan:
    #         print("begin planning")
    #         node.plan_to_pose(*pose)
    #         # node.plan_to_joint_positions(joint) 
    #         inp = input("pose planned press h if you're happy to execute press anything else to replan:")
    #         if inp == 'h':
    #             happy_with_plan = True

    #     node.execute_plan()
    #     input("pose executed press enter to continue")

    #     node.motor_on()
    #     input("motor on press enter to continue")

    #     node.contact()
    #     input("contact made press enter to continue")

    #     node.motor_off()
    #     input("motor off press enter to continue")

    #     node.retract()
    #     input("retracted press enter to continue")

    #     node.ros2_control()
    #     input("ros2 control press enter to continue")

    for screw_num in [1,2,3]:

        node.go_above_screw(screw_num)
        time.sleep(1)
        # input(f"Above screw {screw_num} loaded, press enter to play")
        node.play_program()
        time.sleep(2)

        node.motor_on()

        node.zforce()
        time.sleep(1)
        # input("zforce program loaded, press enter to play")
        node.play_program()
        if screw_num == 2:
            time.sleep(12)
        else:
            time.sleep(8)

        node.motor_off()

        node.retract()
        time.sleep(1)
        # input("retracted program loaded, press enter play")
        node.play_program()
        time.sleep(2)

        # input("press enter to proced to the next node")

if __name__ == '__main__':
    main()
