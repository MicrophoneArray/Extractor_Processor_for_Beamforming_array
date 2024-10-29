import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray  # Import the message type
import xml.etree.ElementTree as ET

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher_ = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/cae_micarray/audio_array',
            self.audio_array_callback,
            10
        )
        self.marker_array = MarkerArray()
        self.audio_values = None  # Initialize audio array

        # Load markers from XML
        self.load_markers_from_xml('/cae-microphone-array-containerized/src/Extractor_V2/processor/resource/Acoular_data/mic_on_the_car.xml')

    def load_markers_from_xml(self, xml_file):
        tree = ET.parse(xml_file)
        root = tree.getroot()
        
        for pos in root.findall('pos'):
            marker_id = int(pos.get('Name'))  # Use the "Name" attribute for the marker ID
            x = float(pos.get('x'))
            y = float(pos.get('y'))
            z = float(pos.get('z'))
            self.create_marker(marker_id, x, y, z)

    def create_marker(self, marker_id, x, y, z):
        marker = Marker()
        marker.header.frame_id = "map"  # Set to the appropriate frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "markers"
        marker.id = marker_id
        marker.type = Marker.SPHERE  # Marker type
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = z
        marker.pose.position.z = y
        marker.scale.x = 0.1  # Size of the marker
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0  # Alpha (opacity)

        # Default color is green
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        self.marker_array.markers.append(marker)

    def audio_array_callback(self, msg):
        audio_values = msg.data.reshape((32,-1))  # Update audio values
        self.get_logger().info(f'Received audio array: {self.audio_values}')
        
        # Set color based on audio values
        for i in range(32):
            intensity = self.audio_values[i]  # Get the corresponding audio value
            marker.color.r = intensity  # Map the audio value to color
            marker.color.g = 1.0 - intensity
            marker.color.b = 0.0  # Keep blue as 0

        self.publisher_.publish(self.marker_array)

def main(args=None):
    rclpy.init(args=args)
    marker_publisher = MarkerPublisher()
    rclpy.spin(marker_publisher)
    marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
