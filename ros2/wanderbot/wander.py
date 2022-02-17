import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.time import Duration


class WanderBot(Node):
    def __init__(self, publish_frequency=10, linear_velocity=0.1, angular_velocity=1):
        """
        WanderBot node for turning regularly or when we're too close to something else
        Parameters
        ----------
        publish_frequency: float
            Publisher frequency in Hz
        linear_velocity:
            Forward velocity when wandering
        angular_velocity:
            Angular velocity when turning fo 30 seconds (rad/s)
        """
        super(WanderBot, self).__init__("wander_bot")
        self.range_ahead = 1  # anything to start

        self.subscription = self.create_subscription(
            LaserScan, "scan", self.scan_callback, qos_profile=1
        )
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", qos_profile=1)

        self.state_change_time = self.get_clock().now()
        self.driving_forward = False

        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity

        self.timer = self.create_timer(
            timer_period_sec=1.0 / publish_frequency, callback=self.timer_callback
        )

    def scan_callback(self, msg):
        """
        Report nearest distance in laser scan message

        Parameters
        ----------
        msg: LaserScan

        Returns
        -------

        """
        self.range_ahead = min(msg.ranges[int(len(msg.ranges) / 4):-int(len(msg.ranges) / 4)])

    def timer_callback(self):
        """Callback for publishing with given frequency"""
        if self.driving_forward:
            # BEGIN FORWARD
            if (
                self.range_ahead < 0.1
                or self.state_change_time < self.get_clock().now()
            ):
                self.driving_forward = False
                self.state_change_time = self.get_clock().now() + Duration(seconds=5)
            # END FORWARD
        else:  # we're not driving forward
            # BEGIN TURNING
            if self.get_clock().now() > self.state_change_time:
                self.driving_forward = True  # we're done spinning, time to go forwards!
                self.state_change_time = self.get_clock().now() + Duration(seconds=30)
            # END TURNING
        twist = Twist()
        if self.driving_forward:
            twist.linear.x = float(self.linear_velocity)
        else:
            twist.angular.z = float(self.angular_velocity)
        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    wander_node = WanderBot()

    rclpy.spin(wander_node)

    wander_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
