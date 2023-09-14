import os
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32






class WifiSignalPublisher(Node):

    def __init__(self):
        super().__init__('wifi_signal_publisher')
        self.declare_parameter('ssid', rclpy.Parameter.Type.STRING) 
        self.ssid = self.get_parameter('ssid').value
        self.get_logger().info(f"SSID = {self.ssid}")
        self.publisher_ = self.create_publisher(Float32, '/fv/signal_strength', 10)
        self.timer = self.create_timer(1, self.publish_wifi_signal)

    def get_wifi_signal_strength(self, ssid):
        try:
            # Use the 'iwlist' command to get the signal strength for the specified SSID
            #output = os.popen(f"iwlist wlan0 scanning | grep '{ssid}' -A 5 | grep 'Signal level='").read()
            output = os.popen(f"iwlist wlan0 scanning | grep 'Signal level='").read()
            signal_level = re.search(r"-[0-9]+", output).group()
            signal_strength = int(signal_level)
            return signal_strength
        except Exception as e:
            self.get_logger().error(f"Error retrieving signal strength for SSID {ssid}: {str(e)}")
            return None

    def publish_wifi_signal(self):
        signal_strength = self.get_wifi_signal_strength(self.ssid)
        if signal_strength is not None:
            self.get_logger().info(f"Signal strength from  {self.ssid}: {signal_strength} dBm")
            msg = Int32()
            msg.data = signal_strength
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WifiSignalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




