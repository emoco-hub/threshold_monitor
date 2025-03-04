import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import time

class ThresholdMonitorNode(Node):
    def __init__(self):
        super().__init__('threshold_monitor')
        
        self.declare_parameter('threshold', 50.0)
        self.declare_parameter('comparison_mode', 'above')  # 'above' or 'below'
        self.declare_parameter('stale_timeout', 5.0)  # Time in seconds

        self.threshold = self.get_parameter('threshold').value
        self.mode = self.get_parameter('comparison_mode').value
        self.stale_timeout = self.get_parameter('stale_timeout').value

        self.last_received_time = None

        self.subscription = self.create_subscription(
            Float32, '/threshold_monitor/input', self.listener_callback, 10)
        self.alert_pub = self.create_publisher(String, '/threshold_monitor/alert', 10)

        self.timer = self.create_timer(1.0, self.check_staleness)

    def listener_callback(self, msg):
        self.last_received_time = time.time()
        alert_msg = String()

        if self.mode == 'above' and msg.data > self.threshold:
            alert_msg.data = "ALERT_TRUE"
        elif self.mode == 'below' and msg.data < self.threshold:
            alert_msg.data = "ALERT_TRUE"
        else:
            alert_msg.data = "ALERT_FALSE"
        
        self.alert_pub.publish(alert_msg)

    def check_staleness(self):
        if self.last_received_time is None or (time.time() - self.last_received_time) > self.stale_timeout:
            alert_msg = String()
            alert_msg.data = "STALE"
            self.alert_pub.publish(alert_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ThresholdMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
