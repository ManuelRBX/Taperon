#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import ApplyLinkWrench
from builtin_interfaces.msg import Duration

class PushNode(Node):
    def __init__(self):
        super().__init__('taperon_push')
        self.cli = self.create_client(ApplyLinkWrench, '/world/taperon_world/apply_wrench')
        self.declare_parameter('link', 'taperon::base_link')
        self.declare_parameter('force_x', 200.0)
        self.declare_parameter('secs', 3)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for /apply_wrench...')
        req = ApplyLinkWrench.Request()
        req.link_name.name = self.get_parameter('link').get_parameter_value().string_value
        req.wrench.force.x = float(self.get_parameter('force_x').value)
        req.duration = Duration(sec=int(self.get_parameter('secs').value))
        self.cli.call_async(req).add_done_callback(lambda f: (self.get_logger().info('done'), rclpy.shutdown()))

def main():
    rclpy.init()
    PushNode()
    rclpy.spin(rclpy.create_node('spindummy'))  # keep alive until callback

if __name__ == '__main__':
    main()
