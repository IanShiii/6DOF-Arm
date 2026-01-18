import rclpy
import random
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import ServoCommandType

X_CENTER = 0.4
X_HALF_RANGE = 0.2
Y_CENTER = 0.0
Y_HALF_RANGE = 0.2
Z_CENTER = 0.4
Z_HALF_RANGE = 0.1

class Operator(Node):
    def __init__(self):
        super().__init__('operator')
        self.wrist_pose_subscriber = self.create_subscription(Point, '/hand_tracking/wrist', self.wrist_callback, 10)

        self.pose_command_publisher = self.create_publisher(PoseStamped, '/target_pose', 1)

        self.switch_command_type_client = self.create_client(ServoCommandType, '/servo_node/switch_command_type')
        self.switch_command_type_client.wait_for_service()
        request = ServoCommandType.Request()
        request.command_type = ServoCommandType.Request.POSE
        future = self.switch_command_type_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        self.last_target_pose = PoseStamped()
        self.last_target_pose.header.frame_id = 'world'
        self.last_target_pose.pose.position.x = X_CENTER
        self.last_target_pose.pose.position.y = Y_CENTER
        self.last_target_pose.pose.position.z = Z_CENTER
        self.last_target_pose.pose.orientation.w = 0.5
        self.last_target_pose.pose.orientation.y = 0.5
        self.last_target_pose.pose.orientation.z = 0.5
        self.last_target_pose.pose.orientation.x = 0.5

        self.create_timer(1.5, self.publish_last_target_pose)
        # self.create_timer(0.2, self.create_random_pose)

        self.get_logger().info('OperatorServo node started')

    def wrist_callback(self, msg: Point):
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = 'world'

        target_pose.pose.position.x = X_CENTER + (msg.x * 2 - 1) * X_HALF_RANGE
        target_pose.pose.position.y = Y_CENTER + (msg.y * 2 - 1) * Y_HALF_RANGE
        target_pose.pose.position.z = Z_CENTER + (msg.z * 2 - 1) * Z_HALF_RANGE
        target_pose.pose.orientation.w = 0.5
        target_pose.pose.orientation.y = 0.5
        target_pose.pose.orientation.z = 0.5
        target_pose.pose.orientation.x = 0.5

        self.last_target_pose = target_pose

    def publish_last_target_pose(self):
        self.last_target_pose.header.stamp = self.get_clock().now().to_msg()
        self.pose_command_publisher.publish(self.last_target_pose)

    def create_random_pose(self):
        self.last_target_pose.pose.position.x = X_CENTER + (random.random() * 2 - 1) * X_HALF_RANGE
        self.last_target_pose.pose.position.y = Y_CENTER + (random.random() * 2 - 1) * Y_HALF_RANGE
        self.last_target_pose.pose.position.z = Z_CENTER + (random.random() * 2 - 1) * Z_HALF_RANGE
        self.last_target_pose.pose.orientation.w = 0.5
        self.last_target_pose.pose.orientation.y = 0.5
        self.last_target_pose.pose.orientation.z = 0.5
        self.last_target_pose.pose.orientation.x = 0.5

def main(args=None):
    rclpy.init(args=args)
    node = Operator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
