import rclpy
from rclpy.node import Node
from custom_ros_messages.srv import UR16BehaviorTrigger
from ur16e_rest.scripts.core import RESTAPI
from std_srvs.srv import Trigger
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor



class UR16RestNode(Node):
    def __init__(self):
        super().__init__('ur16_rest_node')
        self.robot_ip = "128.138.224.247"
        self.cb_group = ReentrantCallbackGroup()

        # Declare and read robot_ip parameter
        self.get_logger().info(f'Initializing RESTAPI with robot_ip={self.robot_ip}')
        self.api = RESTAPI(self.robot_ip)

        self.srv_move_contact = self.create_service(
            UR16BehaviorTrigger,
            'ur16e_rest/BehaviorTrigger',
            self.handle_behavior_service,
            callback_group = self.cb_group
        )

        self.robot_program_trigger_client = self.create_client(Trigger, '/io_and_status_controller/resend_robot_program',callback_group = self.cb_group)
        self.get_logger().info('Waiting for /io_and_status_controller/resend_robot_program service...')
        self.robot_program_trigger_client.wait_for_service()
        self.get_logger().info('resend_robot_program service available')




        self.get_logger().info('UR16ERestNode ready.')

    # ---------- Service Callbacks ----------
    def handle_behavior_service(self, request, response):
        self.get_logger().info(f"Received BehaviorTrigger request: {request}")
        status, msg = None, None
        behavior = request.behavior.lower().strip().replace(" ", "")
        if behavior in ("moveuntilcontact", "contact"):
            status, msg = self.api.MoveUntilContact()
            msg = "Robot moved until contact successfully" if msg is None and status == 0 else msg
        elif behavior in ("retract",):
            status, msg = self.api.Retract()
            msg = "Robot retracted successfully" if msg is None and status == 0 else msg
        elif behavior in ("unlockrobot", "unlock"):
            status, msg = self.api.unlock_robot()
            msg = "Robot unlocked successfully" if msg is None and status == 0 else msg
        elif behavior in ("poweronrobot", "poweron"):
            status, msg = self.api.power_on_robot()
            msg = "Robot Powered On Successfully" if msg is None and status == 0 else msg
        elif behavior in ("poweroffrobot", "poweroff"):
            status, msg = self.api.power_off_robot()
            msg = "Robot Powered Off Successfully" if msg is None and status == 0 else msg

        elif behavior in ("ros2control", "externalcontrol"):
            #CALLING ROS2 control is not needed, you just need to invoke the resend_robot_program service
            # status, msg = self.api.ros2_control()
            trig_request = Trigger.Request()
            try:
                trig_response = self.robot_program_trigger_client.call(trig_request)
                if trig_response.success:
                    status = 0
                    msg = f'Service success: {trig_response.message}'
                else:
                    status = -1
                    msg = f'Service returned false: {trig_response.message}'
            except Exception as e:
                status = -1
                msg = f"service call failed: {e}"

        elif behavior in ("play", ):
            status, msg = self.api.play_program()
        else:
            status = -1
            msg = f"Unknown behavior: {request.behavior}"
        self.get_logger().info(f"behavior {behavior} returned status: {status}, msg: {msg}")
        response.success = (status == 0)
        response.message = str(msg)
        self.get_logger().info(f"sending service response: {response}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = UR16RestNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
