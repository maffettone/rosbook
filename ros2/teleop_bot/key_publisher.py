import sys, select, tty, termios
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class KeystrokePublisher(Node):
    def __init__(self, frequency=100.0):
        """

        Parameters
        ----------
        frequency: float
            Frequency of observations and publications
        """
        super().__init__("keystroke_publisher")
        self.publisher_ = self.create_publisher(String, "keys", qos_profile=1)
        self.timer = self.create_timer(
            timer_period_sec=1.0 / frequency, callback=self.timer_callback
        )

    def timer_callback(self):
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            msg = String()
            msg.data = sys.stdin.read(1)
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    keystroke_publisher = KeystrokePublisher()

    # BEGIN TERMIOS
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    # END TERMIOS

    keystroke_publisher.get_logger().info(
        "Publishing keystrokes. Press Ctrl-C to exit..."
    )
    rclpy.spin(keystroke_publisher)
    keystroke_publisher.destroy_node()
    rclpy.shutdown()

    # BEGIN TERMIOS_END
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
    # END TERMIOS_END


if __name__ == "__main__":
    main()
