import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from geometry_msgs.msg import Twist


class CmdVelPub(Node):
    def __init__(self, publish_frequency=10, velocity=0.05):
        super(CmdVelPub, self).__init__("cmd_vel_pub")
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", qos_profile=1)

        self.red_light_twist = Twist()
        self.green_light_twist = Twist()
        self.green_light_twist.linear.x = velocity

        self.timer = self.create_timer(
            timer_period_sec=1./publish_frequency, callback=self.timer_callback
        )

        self.light_change_time = self.get_clock().now()
        self.driving_forward = False

    def timer_callback(self):
        if self.driving_forward:
            self.publisher_.publish(self.green_light_twist)
        else:
            self.publisher_.publish(self.red_light_twist)

        if self.light_change_time < self.get_clock().now():
            self.driving_forward = not self.driving_forward
            self.light_change_time = self.get_clock().now() + Duration(seconds=3)
            self.get_logger().info(f"{'Driving forward' if self.driving_forward else 'Stopped'}")


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CmdVelPub(velocity=-.05)

    rclpy.spin(minimal_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    import os
    print(os.environ["LD_LIBRARY_PATH"])
    print(os.environ["ROS_DOMAIN_ID"])
    main()
