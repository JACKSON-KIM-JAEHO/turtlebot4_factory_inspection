import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
import yaml
import os
import math
import threading
import sys
import termios
import tty

def get_key():
    """터미널에서 키 하나를 비동기적으로 읽음"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def quaternion_to_yaw(x, y, z, w):
    """쿼터니언 → yaw (라디안)"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class BasePointSaver(Node):
    def __init__(self):
        super().__init__('base_point_saver')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile)

        self.latest_pose = None
        self.saved = False

        self.get_logger().info("스페이스 키를 누르면 현재 위치를 base_location.yaml에 저장합니다.")
        threading.Thread(target=self.keyboard_loop, daemon=True).start()

    def odom_callback(self, msg):
        self.latest_pose = msg.pose.pose

    def keyboard_loop(self):
        while True:
            key = get_key()
            if key == ' ':
                if self.saved:
                    self.get_logger().info("이미 저장됨. 새로 저장하려면 노드를 재시작하세요.")
                    continue

                if self.latest_pose is None:
                    self.get_logger().warn("아직 /odom 데이터를 수신하지 못했습니다.")
                    continue

                pose = self.latest_pose
                yaw = quaternion_to_yaw(
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w
                )

                # ROS 2에서 바로 사용할 수 있는 간단한 구조로 저장
                base_pose = {
                    'base_pose': {
                        'x': round(pose.position.x, 3),
                        'y': round(pose.position.y, 3),
                        'yaw': round(math.degrees(yaw), 2)
                    }
                }

                self.save_yaml(base_pose)
                self.saved = True
                self.get_logger().info(f"저장됨: {base_pose}")

    def save_yaml(self, data):
        home_dir = os.path.expanduser('~')
        path = os.path.join(
            home_dir,
            'sw_projects',
            'my_ros2_ws',
            'src',
            'turtlebot4_factory_inspection',
            'data',
            'base_location.yaml'
        )
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        self.get_logger().info(f"저장 완료: {path}")

def main(args=None):
    rclpy.init(args=args)
    node = BasePointSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
