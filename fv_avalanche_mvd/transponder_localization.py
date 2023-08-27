import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int16
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.optimize import minimize
from visualization_msgs.msg import Marker
import math



class TransponderLocalizationNode(Node):
    def __init__(self):
        super().__init__('transponder_localization')
        
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription_position = self.create_subscription(PoseStamped, '/fv/drone_position', self.position_callback, 10)
        self.subscription_signal_strength = self.create_subscription( Float32, '/fv/signal_strength', self.signal_strength_callback, 10)
        
        self.estimated_marker_publisher = self.create_publisher( Marker, '/fv/estimated_transponder', 10)
        self.publisher_next_search_position = self.create_publisher(PoseStamped, '/fv/next_search_position', 10)
        self.search_length_publisher = self.create_publisher(Int16, '/fv/search_length', 10)

        #TODO: global parameter
        self.takeoff_height = 5.0


        # TODO: syncronize measurments
        self.last_drone_position = np.array([0.0, 0.0, 0.0])
        self.drone_positions = []  # List to store received drone positions
        # TODO: do measurment as service???
        self.last_signal_strengt = 0.0
        self.signal_strength = []  # List to store received signal strengths

        # Initial guess for transponder position estimate
        self.estimated_transponder = np.array([0.0, 0.0, 0.0])

        self.next_drone_position = np.array([0.0, 0.0, self.takeoff_height])

        # Create a timer to callculate estimate and next search position
        self.timer = self.create_timer(1.0, self.timer_callback)

    def position_callback(self, msg):
        self.last_drone_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        #self.get_logger().info('Received current_position: %s' % self.last_drone_position)

    def signal_strength_callback(self, msg):
        self.last_signal_strengt = msg.data
        #self.get_logger().info('Received signal_strength: %s' % msg)
    


    def simulate_signal_strength(self, transponder_position, drone_position):
        """
        Calculate the signal strength received by a drone from an avalanche transponder.

        Args:
        transponder_position (tuple): The 3D position of the avalanche transponder (x, y, z).
        drone_position (tuple): The 3D position of the drone (x, y, z).

        Returns:
        float: Signal strength value.
        """

        # Calculate the Euclidean distance between the transponder and drone positions
        distance = math.sqrt((transponder_position[0] - drone_position[0])**2 +
                            (transponder_position[1] - drone_position[1])**2 +
                            (transponder_position[2] - drone_position[2])**2)
        
        # Calculate signal strength using inverse square law: signal_strength = 1 / distance^2
        if distance > 0:
            signal_strength = 1.0 / (distance**2)
        else:
            signal_strength = -1.0

        return signal_strength

    # Function for optimization
    # true_measure are the simulated or measured signals
    def error_function(self, transponder_position_estimate):
        
        estimated_measurements = []
        for i in range(len(self.signal_strength)):
            estimated_measurement = self.simulate_signal_strength(transponder_position_estimate, (self.drone_positions)[i])
            estimated_measurements.append(estimated_measurement)
            
        error = sum((true_measure - est_measure)**2 for true_measure, est_measure in zip(self.signal_strength, estimated_measurements))
        return error
    


    def timer_callback(self) -> None:
        
        # TODO: threshold for reaching the positon
        if (np.linalg.norm(self.next_drone_position - self.last_drone_position) <= 0.1):
            # next position reached

            self.drone_positions.append(self.last_drone_position)
            self.signal_strength.append(self.last_signal_strengt)
            self.get_logger().info(('SEARCHING'))

            if len(self.signal_strength) >= 3:
                # Calculate estimate of transponder
                # Perform optimization
                result = minimize(self.error_function, self.estimated_transponder, method='Nelder-Mead')

                # Estimated transponder position
                self.estimated_transponder = result.x


                # publisch true transponder position
                marker_msg = Marker()
                marker_msg.ns = "transponder"
                marker_msg.id = 1
                marker_msg.header.frame_id = 'base_link'  # Replace with appropriate frame ID
                marker_msg.header.stamp = self.get_clock().now().to_msg()
                marker_msg.type = Marker.SPHERE
                marker_msg.pose.position.x = (self.estimated_transponder)[0]
                marker_msg.pose.position.y = (self.estimated_transponder)[1]
                marker_msg.pose.position.z = (self.estimated_transponder)[2]
                marker_msg.scale.x = 0.5
                marker_msg.scale.y = 0.5
                marker_msg.scale.z = 0.5
                marker_msg.color.r = 0.0
                marker_msg.color.g = 1.0
                marker_msg.color.b = 0.0
                marker_msg.color.a = 1.0
                self.estimated_marker_publisher.publish(marker_msg)

            # TODO: Random next position
            self.next_drone_position = np.array([np.random.uniform(-5, 5), np.random.uniform(-5, 5), self.takeoff_height])



        data_length_msg = Int16()
        data_length_msg.data = len(self.signal_strength)
        self.search_length_publisher.publish(data_length_msg)
        

        # create next random position arround estimate
        pose_msg = PoseStamped()
        # TODO: get timestamp from PX4 massage
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'  # Adjust frame_id to your needs

        pose_msg.pose.position.x = self.next_drone_position[0]
        pose_msg.pose.position.y = self.next_drone_position[1]
        pose_msg.pose.position.z = self.next_drone_position[2]

        # TODO: integrate attitude
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        self.publisher_next_search_position.publish(pose_msg)





def main(args=None):
    rclpy.init(args=args)
    node = TransponderLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
