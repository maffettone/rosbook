import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# BEGIN KEYMAP
key_mapping = {
    "w": [0.0, 1.0],
    "x": [0.0, -1.0],
    "a": [-1.0, 0.0],
    "d": [1.0, 0.0],
    "s": [0.0, 0.0],
}
# END KEYMAP


class KeyToTwist(Node):
    def __init__(self):
        super().__init__("key_to_twist")
        self.subscription_ = self.create_subscription(
            String, "keys", self.keys_callback, 10
        )
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 1)

    def keys_callback(self, msg):
        if len(msg.data) == 0 or not msg.data[0] in key_mapping:
            return
        vels = key_mapping[msg.data[0]]
        t = Twist()
        t.angular.z = vels[0]
        t.linear.x = vels[1]
        self.publisher_.publish(t)


def main(args=None):
    rclpy.init(args=args)
    key_to_twist = KeyToTwist()
    rclpy.spin(key_to_twist)
    key_to_twist.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
