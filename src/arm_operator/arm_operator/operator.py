import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import ServoCommandType

class Operator(Node):
    def __init__(self):
        super().__init__('operator_servo')
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.switch_command_type_client = self.create_client(ServoCommandType, '/servo_node/switch_command_type')
        self.switch_command_type_client.wait_for_service()

        self.joint_jog_publisher = self.create_publisher(JointJog, 'servo_node/delta_joint_cmds', 10)
        self.twist_command_publisher = self.create_publisher(TwistStamped, 'servo_node/delta_twist_cmds', 10)
        self.pose_command_publisher = self.create_publisher(PoseStamped, 'servo_node/pose_target_cmds', 10)

        request = ServoCommandType.Request()
        request.command_type = ServoCommandType.Request.TWIST
        future = self.switch_command_type_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info('OperatorServo node started')

    def joy_callback(self, msg: Joy):
        something_has_been_pressed = any(button == 1 for button in msg.buttons) or any(axis != 0 for axis in msg.axes)

        if not something_has_been_pressed:
            return
        
        twist_cmd = TwistStamped()
        twist_cmd.header.stamp = self.get_clock().now().to_msg()
        twist_cmd.header.frame_id = 'world'

        if (msg.buttons[3] == 1):  # y button
            twist_cmd.twist.linear.x = 0.1
        elif (msg.buttons[1] == 1):  # a button
            twist_cmd.twist.linear.x = -0.1

        if (msg.buttons[2] == 1):  # b button
            twist_cmd.twist.linear.y = 0.1
        elif (msg.buttons[0] == 1):  # x button
            twist_cmd.twist.linear.y = -0.1

        if (msg.axes[5] == 1):  # dpad up
            twist_cmd.twist.linear.z = 0.1
        elif (msg.axes[5] == -1):  # dpad down
            twist_cmd.twist.linear.z = -0.1

        self.twist_command_publisher.publish(twist_cmd)
        self.get_logger().info('Published Twist Command')


def main(args=None):
    rclpy.init(args=args)
    node = Operator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
