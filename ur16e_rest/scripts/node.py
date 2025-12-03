import rclpy
from rclpy.node import Node
from custom_ros_messages.srv import UR16BehaviorTrigger
from ur16e_rest.scripts.core import RESTAPI

class UR16RestNode(Node):
    def __init__(self):
        super().__init__('ur16_rest_node')
        self.robot_ip = "128.138.224.247"

        # Declare and read robot_ip parameter
        self.get_logger().info(f'Initializing RESTAPI with robot_ip={self.robot_ip}')
        self.api = RESTAPI(self.robot_ip)

        self.srv_move_contact = self.create_service(
            UR16BehaviorTrigger,
            'ur16e_rest/BehaviorTrigger',
            self.handle_behavior_service
        )

        self.get_logger().info('UR16ERestNode ready.')

    # ---------- Service Callbacks ----------
    def handle_behavior_service(self, request, response):
        print(f"recived service request: {request}")
        self.get_logger().info('Received BehaviorTrigger request')
        status, msg = None, None
        behavior = request.behavior.lower().strip().replace(" ", "")
        if behavior == "moveuntilcontact":
            status, msg = self.api.MoveUntilContact()
            msg = "Robot moved until contact successfully" if msg is None and status == 0 else msg
        elif behavior == "retract":
            status, msg = self.api.Retract()
            msg = "Robot retracted successfully" if msg is None and status == 0 else msg
        elif behavior == "unlockrobot":
            status, msg = self.api.unlock_robot()
            msg = "Robot unlocked successfully" if msg is None and status == 0 else msg
        elif behavior == "lockrobot":
            status, msg = self.api.lock_robot()
            msg = "Robot locked successfully" if msg is None and status == 0 else msg
        elif behavior == "poweronrobot":
            status, msg = self.api.power_on_robot()
            msg = "Robot Powered On Successfully" if msg is None and status == 0 else msg
        elif behavior == "poweroffrobot":
            status, msg = self.api.power_off_robot()
            msg = "Robot Powered Off Successfully" if msg is None and status == 0 else msg
        else:
            status = -1
            msg = f"Unknown behavior: {request.behavior}"
        print(f"behavior {behavior} returned status: {status}, msg: {msg}")
        response.success = (status == 0)
        response.message = str(msg)
        print(f"sending service response: {response}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = UR16RestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
