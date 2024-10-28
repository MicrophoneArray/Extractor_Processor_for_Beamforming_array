import cv2
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

# Function to draw grid with indices and RGB values
def draw_grid(image, rgb_values):
    num_squares = 56
    squares_per_row = int(np.sqrt(num_squares))
    squares_per_col = num_squares // squares_per_row

    square_width = 800 // squares_per_row
    square_height = 700 // squares_per_col

    index = 0
    for i in range(squares_per_col):
        for j in range(squares_per_row):
            top_left = (j * square_width, i * square_height)
            bottom_right = ((j + 1) * square_width, (i + 1) * square_height)
            color = rgb_values[index]
            cv2.rectangle(image, top_left, bottom_right, color, -1)
            text_position = (top_left[0] + 10, top_left[1] + 30)
            cv2.putText(image, str(index), text_position, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            index += 1

def update_image(msg, publisher, bridge):
    # Create a blank image with dimensions 800x700
    image = np.zeros((700, 800, 3), dtype=np.uint8)

    # Extract RGB values from MarkerArray
    rgb_values = []
    for marker in msg.markers:
        r = int(marker.color.r * 255)
        g = int(marker.color.g * 255)
        b = int(marker.color.b * 255)
        rgb_values.append((b, g, r))  # OpenCV uses BGR format

    # Ensure we have exactly 56 RGB values
    if len(rgb_values) < 56:
        rgb_values.extend([(0, 255, 0)] * (56 - len(rgb_values)))  # Fill remaining with green
    elif len(rgb_values) > 56:
        rgb_values = rgb_values[:56]  # Trim to 56

    # Draw the grid with indices and RGB values
    draw_grid(image, rgb_values)

    # Convert OpenCV image to ROS Image message
    image_msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")

    # Publish the image
    publisher.publish(image_msg)

if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('mic_tester')

    # Define QoS profile with best effort reliability and required history and depth settings
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )

    # Create a publisher for the image topic
    image_publisher = node.create_publisher(Image, 'grid_image', qos_profile)

    # Create a CvBridge object
    bridge = CvBridge()

    # Create a subscription to the MarkerArray topic
    subscriber = node.create_subscription(
        MarkerArray, 
        "/cae_micarray/markers", 
        lambda msg: update_image(msg, image_publisher, bridge), 
        qos_profile
    )

    rclpy.spin(node)
    rclpy.shutdown()