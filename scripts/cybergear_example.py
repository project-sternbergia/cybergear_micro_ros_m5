#!/usr/env/bin python3

import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import JointState

class CybergearExampleNode(Node):
    def __init__(self):
        super().__init__("example")
        self._pub = self.create_publisher(JointState, "/cgc_node/joint_command", 1)
        self.create_timer(0.01, self.timer_callback)
        self._max_pos = 0.1
        self._cnt = 0

    def timer_callback(self):
        msg = JointState()
        msg.name.append("mot1");
        msg.name.append("mot2");
        pos = 0.4 * math.sin(self._cnt / 100 * 2.0 * math.pi)
        msg.position.append(pos);
        msg.position.append(pos);
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "cybergear"
        self._pub.publish(msg)
        self._cnt += 1
        self._cnt = self._cnt % 100


def main(args=None):
    try:
        rclpy.init(args=args)
        node = CybergearExampleNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 終了処理
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
