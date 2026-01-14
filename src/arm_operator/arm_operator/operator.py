import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from moveit_msgs.srv import ServoCommandType
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class Operator(Node):
    def __init__(self):
        super().__init__('operator')
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.switch_command_type_client = self.create_client(ServoCommandType, '/servo_node/switch_command_type')
        self.switch_command_type_client.wait_for_service()

        self.joint_jog_publisher = self.create_publisher(JointJog, 'servo_node/delta_joint_cmds', 10)
        self.twist_command_publisher = self.create_publisher(TwistStamped, 'servo_node/delta_twist_cmds', 10)
        self.pose_command_publisher = self.create_publisher(PoseStamped, 'servo_node/pose_target_cmds', 10)

        self.joint_trajectory_publisher = self.create_publisher(JointTrajectory, 'arm_controller/joint_trajectory', 10)

        self.current_command_type = None
        self.set_command_type(ServoCommandType.Request.JOINT_JOG)

        self.get_logger().info('OperatorServo node started')

    def joy_callback(self, msg: Joy):
        something_has_been_pressed = any(button == 1 for button in msg.buttons) or any(axis != 0 for axis in msg.axes)
        if not something_has_been_pressed:
            return
        
        if (msg.buttons[5]): # right bumper
            self.go_to_start_angles()
            return
        
        if (msg.buttons[6]): # left trigger
            self.set_command_type(ServoCommandType.Request.JOINT_JOG)
            return
        if (msg.buttons[7]): # right trigger
            self.set_command_type(ServoCommandType.Request.TWIST)
            return
        if (msg.buttons[4]): # left bumper
            self.set_command_type(ServoCommandType.Request.POSE)
            return
        
        if self.current_command_type == ServoCommandType.Request.JOINT_JOG:
            self.handle_joint_jog_controls(msg)
        elif self.current_command_type == ServoCommandType.Request.TWIST:
            self.handle_twist_controls(msg)
        elif self.current_command_type == ServoCommandType.Request.POSE:
            self.handle_pose_controls(msg)

    def handle_joint_jog_controls(self, msg: Joy):
        joint_jog = JointJog()
        joint_jog.header.stamp = self.get_clock().now().to_msg()
        joint_jog.header.frame_id = 'world'
        joint_jog.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        joint_jog.velocities = [0.0] * 6
        joint_jog.duration = 0.25  # seconds

        if (msg.axes[1] == 1):  # dpad up
            joint_jog.velocities[0] = 0.1
        elif (msg.axes[1] == -1):  # dpad down
            joint_jog.velocities[0] = -0.1

        if (msg.axes[0] == 1):  # dpad left
            joint_jog.velocities[1] = 0.1
        elif (msg.axes[0] == -1):  # dpad right
            joint_jog.velocities[1] = -0.1

        if (msg.buttons[3]):  # y button
            joint_jog.velocities[2] = 0.1
        elif (msg.buttons[1]):  # a button
            joint_jog.velocities[2] = -0.1

        if (msg.buttons[2]):  # b button
            joint_jog.velocities[3] = 0.1
        elif (msg.buttons[0]):  # x button
            joint_jog.velocities[3] = -0.1

        if (msg.axes[1] == 1):  # left joystick up
            joint_jog.velocities[4] = 0.1
        elif (msg.axes[1] == -1):  # left joystick down
            joint_jog.velocities[4] = -0.1

        if (msg.axes[3] == 1):  # right joystick up
            joint_jog.velocities[5] = 0.1
        elif (msg.axes[3] == -1):  # right joystick down
            joint_jog.velocities[5] = -0.1

        self.joint_jog_publisher.publish(joint_jog)
        self.get_logger().info('Published Joint Jog Command')
    
    def handle_twist_controls(self, msg: Joy):
        twist_cmd = TwistStamped()
        twist_cmd.header.stamp = self.get_clock().now().to_msg()
        twist_cmd.header.frame_id = 'world'

        if (msg.buttons[3] == 1):  # y button
            twist_cmd.twist.linear.x = 0.1
        elif (msg.buttons[1] == 1):  # a button
            twist_cmd.twist.linear.x = -0.1

        if (msg.buttons[0] == 1):  # x button
            twist_cmd.twist.linear.y = 0.1
        elif (msg.buttons[2] == 1):  # b button
            twist_cmd.twist.linear.y = -0.1

        if (msg.axes[5] == 1):  # dpad up
            twist_cmd.twist.linear.z = 0.1
        elif (msg.axes[5] == -1):  # dpad down
            twist_cmd.twist.linear.z = -0.1

        self.twist_command_publisher.publish(twist_cmd)
        self.get_logger().info('Published Twist Command')

    def handle_pose_controls(self, msg: Joy):
        pose_cmd = PoseStamped()
        pose_cmd.header.stamp = self.get_clock().now().to_msg()
        pose_cmd.header.frame_id = 'world'

        self.get_logger().warn('Pose Controls not yet implemented')

    def go_to_start_angles(self):
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        start_positions = [1.5708, 1.0, 0.2, 0.5, 0.5, 0.5]
        point = JointTrajectoryPoint()
        point.positions = start_positions
        point.time_from_start.sec = 2
        joint_trajectory.points = [point]

        self.joint_trajectory_publisher.publish(joint_trajectory)
        self.get_logger().info('Published Joint Trajectory to Start Angles')
    
    def set_command_type(self, command_type):
        if self.current_command_type != command_type:
            self.current_command_type = command_type
            request = ServoCommandType.Request()
            request.command_type = command_type
            future = self.switch_command_type_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            if (command_type == ServoCommandType.Request.JOINT_JOG):
                self.get_logger().info('Switched command type to JOINT_JOG')
            elif (command_type == ServoCommandType.Request.TWIST):
                self.get_logger().info('Switched command type to TWIST')
            elif (command_type == ServoCommandType.Request.POSE):
                self.get_logger().info('Switched command type to POSE')

def main(args=None):
    rclpy.init(args=args)
    node = Operator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
