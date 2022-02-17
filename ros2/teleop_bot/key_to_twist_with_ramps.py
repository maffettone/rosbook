import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math

# BEGIN KEYMAP
key_mapping = {
    "w": [0.0, 1.0],
    "x": [0.0, -1.0],
    "a": [1.0, 0.0],
    "d": [-1.0, 0.0],
    "s": [0.0, 0.0],
}
# END KEYMAP


# BEGIN RAMP
def ramped_vel(v_prev, v_target, t_prev, t_now, ramp_rate):
    """

    Parameters
    ----------
    v_prev: float
    v_target: float
    t_prev: Time
    t_now: Time
    ramp_rate: float
        Acceleration

    Returns
    -------

    """
    # compute maximum velocity step
    step = ramp_rate * (t_now - t_prev).nanoseconds / 1e9
    sign = 1.0 if (v_target > v_prev) else -1.0
    error = math.fabs(v_target - v_prev)
    if error < step:  # we can get there within this timestep. we're done.
        return v_target
    else:
        return v_prev + sign * step  # take a step towards the target


# END RAMP


class KeyToTwist(Node):
    def __init__(self, frequency=20):
        super().__init__("key_to_twist")
        self.subscription_ = self.create_subscription(
            String, "keys", self.keys_callback, 10
        )
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 1)
        self.timer = self.create_timer(1.0 / frequency, self.send_twist)
        self.target_twist = Twist()
        self.last_twist = Twist()  # Most recent twist in progress to target
        self.last_twist_send_time = self.get_clock().now()
        self.declare_parameter("linear_scale", 0.5)
        self.declare_parameter("angular_scale", 1.0)
        self.declare_parameter("linear_accel", 1.0)
        self.declare_parameter("angular_accel", 1.0)

    def keys_callback(self, msg):
        """
        Converts message to twist from keys
        """
        if len(msg.data) == 0 or not msg.data[0] in key_mapping:
            return
        vels = key_mapping[msg.data[0]]
        t = Twist()
        t.angular.z = (
            vels[0]
            * self.get_parameter("angular_scale").get_parameter_value().double_value
        )
        t.linear.x = (
            vels[1]
            * self.get_parameter("linear_scale").get_parameter_value().double_value
        )
        self.target_twist = t

    def ramped_twist(self, prev, target, t_prev, t_now):
        """
        Ramp the twist based in the acceleration parameters

        Parameters
        ----------
        prev: Twist
        target: Twist
        t_prev: Time
        t_now: Time

        Returns
        -------

        """
        tw = Twist()
        tw.angular.z = ramped_vel(
            prev.angular.z,
            target.angular.z,
            t_prev,
            t_now,
            self.get_parameter("angular_accel").get_parameter_value().double_value,
        )
        tw.linear.x = ramped_vel(
            prev.linear.x,
            target.linear.x,
            t_prev,
            t_now,
            self.get_parameter("linear_accel").get_parameter_value().double_value,
        )
        return tw

    def send_twist(self):
        """
        Publish twist. Timer based callback.
        Returns
        -------

        """
        t_now = self.get_clock().now()
        self.last_twist = self.ramped_twist(
            self.last_twist,
            self.target_twist,
            self.last_twist_send_time,
            t_now,
        )
        self.last_twist_send_time = t_now
        self.publisher_.publish(self.last_twist)


def main(args=None):
    rclpy.init(args=args)
    key_to_twist = KeyToTwist()
    rclpy.spin(key_to_twist)
    key_to_twist.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
