import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs_py import point_cloud2 as pc2
from sklearn.cluster import DBSCAN
import numpy as np

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('point_cloud_subscriber')

        lidar_qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        
        # Subscribers to the topics
        self.subscription_beam = self.create_subscription(
            PointCloud2,
            '/beam_point_cloud',
            self.beam_callback,
            1
        )
        self.subscription_beam  # Prevent unused variable warning

        self.subscription_lidar = self.create_subscription(
            PointCloud2,
            '/sensing/lidar/top/points',
            self.lidar_callback,
            lidar_qos_profile
        )
        self.subscription_lidar  # Prevent unused variable warning

        # Publishers to the point cloud without ground points
        self.publisher_no_ground = self.create_publisher(
            PointCloud2,
            '/point_cloud_no_ground',
            1
        )

        self.publisher_cluster = self.create_publisher(
            PointCloud2,
            '/point_cloud_cluster',
            1
        )

        self.detection_angle = 0

    def beam_callback(self, msg):
        # Extract x, y, z points from PointCloud2 message
        points = []
        for point in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x,y,z = point
            points.append([x,y,z])
        
        points = np.array(points)
        
        # now we calculate the DoA from the detection point
        x_average = np.mean(points[:,0])
        z_average = np.mean(points[:,2])
        angle = np.arctan2(z_average, x_average)/np.pi * 180
        self.detection_angle = angle
        print('the detection angle is ', self.detection_angle)


    def lidar_callback(self, msg):
        # Extract points and filter ground points
        filtered_points = self.filter_ground(msg)

        # Convert filtered points back to PointCloud2 format
        filtered_msg = self.create_pointcloud2_msg(filtered_points, msg.header)

        # Cluster the filtered points using DBSCAN
        clustered_points = self.cluster_points(filtered_points)

        # Convert clustered points back to PointCloud2 format
        clustered_msg = self.create_pointcloud2_msg(clustered_points, msg.header)

        # Publish the filtered point cloud
        self.publisher_no_ground.publish(filtered_msg)
        self.publisher_cluster.publish(clustered_msg)


    def filter_ground(self, msg, ground_threshold=-1.5, top_threshold=0):
        # Extract (x, y, z) points from PointCloud2
        points = np.array([
            [point[0], point[1], point[2]]
            for point in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        ])

        # Filter points based on z value (height threshold)
        filtered_points = points[points[:, 2] > ground_threshold]
        filtered_points = filtered_points[filtered_points[:, 2] < top_threshold]
        filtered_points = filtered_points[(filtered_points[:, 0]**2 + filtered_points[:, 1]**2 + filtered_points[:, 2]**2) > 4]

        # filter the point according to the detection angle
        filtered_points = filtered_points[np.arctan2(-filtered_points[:,0], -filtered_points[:,1])/np.pi * 180 > self.detection_angle - 15]
        filtered_points = filtered_points[np.arctan2(-filtered_points[:,0], -filtered_points[:,1])/np.pi * 180 < self.detection_angle + 15]

        return filtered_points
    
    def cluster_points(self, points, eps=0.2, min_samples=50):
        # Perform DBSCAN clustering
        dbscan = DBSCAN(eps=eps, min_samples=min_samples)
        labels = dbscan.fit_predict(points)
        print(labels.shape)

        # Extract points that are part of clusters (label != -1)
        clustered_points = points[labels != -1]
        print(clustered_points.shape)
        
        return clustered_points

    def create_pointcloud2_msg(self, points, header):
        # Convert numpy array of points back to PointCloud2 message
        point_cloud_msg = pc2.create_cloud_xyz32(header, points.tolist())
        return point_cloud_msg
    
def main(args=None):
    rclpy.init(args=args)
    point_cloud_subscriber = PointCloudSubscriber()
    
    try:
        rclpy.spin(point_cloud_subscriber)
    except KeyboardInterrupt:
        pass
    
    point_cloud_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
