import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker
from px4_msgs.msg import VehicleLocalPosition


import math
import random

class SignalStrengthNode(Node):
    def __init__(self):
        super().__init__('signal_simulation')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )


        self.declare_parameter('transponder_position_x', 0.0)
        self.declare_parameter('transponder_position_y', 0.0)
        self.declare_parameter('transponder_position_z', 0.0)
        self.declare_parameter('transponder_position_noise', 0.1)

        # get the params from the launch file
        transponder_position_x = self.get_parameter('transponder_position_x').value
        transponder_position_y = self.get_parameter('transponder_position_y').value
        transponder_position_z = self.get_parameter('transponder_position_z').value        
        transponder_position = (transponder_position_x, transponder_position_y, transponder_position_z)  # Replace with actual transponder position
        self.transponder_position = (transponder_position)

        self.noise = self.get_parameter('transponder_position_noise').value


        # subscribe to the drone local position
        #self.vehicle_local_position_subscriber = self.create_subscription(
        #    VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.drone_position_callback, qos_profile)
        self.subscription_position = self.create_subscription(PoseStamped, '/fv/drone_position', self.drone_position_callback, 10)


        # publisch the wifi signal strength
        self.signal_strength_publisher = self.create_publisher(
            Float32, '/fv/signal_strength', 10)

        self.true_transponder_publisher = self.create_publisher(
            Marker, '/fv/true_transponder', 10)

    

    def drone_position_callback(self, msg):
        """Callback function for vehicle_local_position topic subscriber."""

        drone_position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        signal_strength = self.simulate_signal_strength(self.transponder_position, drone_position, self.noise)
        signal_msg = Float32()
        signal_msg.data = signal_strength
        self.signal_strength_publisher.publish(signal_msg)

        # publisch true transponder position
        marker_msg = Marker()
        marker_msg.ns = "transponder"
        marker_msg.id = 0
        marker_msg.header.frame_id = 'base_link'  # Replace with appropriate frame ID
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.type = Marker.SPHERE
        marker_msg.pose.position.x = (self.transponder_position)[0]
        marker_msg.pose.position.y = (self.transponder_position)[1]
        marker_msg.pose.position.z = (self.transponder_position)[2]
        marker_msg.scale.x = 0.5
        marker_msg.scale.y = 0.5
        marker_msg.scale.z = 0.5
        marker_msg.color.r = 1.0
        marker_msg.color.g = 1.0
        marker_msg.color.b = 0.0
        marker_msg.color.a = 1.0
        self.true_transponder_publisher.publish(marker_msg)

        



    def simulate_signal_strength(self, transponder_position, drone_position, noise):
        distance = math.sqrt((transponder_position[0] - drone_position[0])**2 +
                             (transponder_position[1] - drone_position[1])**2 +
                             (transponder_position[2] - drone_position[2])**2)
        
        distance += random.uniform(-noise, noise)
        
        if distance > 0:
            signal_strength = 1.0 / (distance**2)
        else:
            signal_strength = -1.0
        
        return signal_strength

def main(args=None):
    rclpy.init(args=args)
    
    signal_strength_node = SignalStrengthNode()
    
    rclpy.spin(signal_strength_node)
    
    signal_strength_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
