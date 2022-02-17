import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class RangeAhead(Node):
    def __init__(self):
        super(RangeAhead, self).__init__("range_ahead")
        self.subscription = self.create_subscription(
            LaserScan, "scan", self.scan_callback, 1
        )

    def scan_callback(self, msg):
        range_ahead = msg.ranges[int(len(msg.ranges) / 2)]
        self.get_logger().info(f"range ahead: {range_ahead}")


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = RangeAhead()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
