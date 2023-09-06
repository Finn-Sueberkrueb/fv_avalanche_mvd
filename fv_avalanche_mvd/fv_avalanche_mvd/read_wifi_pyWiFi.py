import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import pywifi
from pywifi import const

class WifiSignalPublisher(Node):

    def __init__(self):
        super().__init__('wifi_signal_publisher')
        self.declare_parameter('ssid', rclpy.Parameter.Type.STRING) 
        self.ssid = self.get_parameter('ssid').value
        self.get_logger().info(f"SSID = {self.ssid}")
        self.publisher_ = self.create_publisher(Int32, 'wifi_signal_strength', 10)
        self.timer = self.create_timer(1, self.publish_wifi_signal)

    def get_wifi_signal_strength(self, ssid):
        wifi = pywifi.PyWiFi()
        iface = wifi.interfaces()[0]

        iface.scan()
        rclpy.spin_once(self)
        scan_results = iface.scan_results()

        for result in scan_results:
            if result.ssid == ssid:
                signal_strength = result.signal
                return signal_strength

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




