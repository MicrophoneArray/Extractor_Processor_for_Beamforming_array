import rclpy
import io
import time
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
import os
from cv_bridge import CvBridge
import numpy as np
from extractor_node.msg import AvReader
import std_msgs.msg
import sensor_msgs_py.point_cloud2 as pc2
import matplotlib.pyplot as plt
from acoular import MicGeom, TimeSamples, PowerSpectra, RectGrid,\
SteeringVector, BeamformerBase, BeamformerMusic, BeamformerEig, L_p, Environment, ImportGrid
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas


class cameraAlign:

    def __init__(self):
        #default values are based on transformation from ueye/left to mic_array (Use set_attributes to change alignment information)
        self.position_x    = -0.47  #m
        self.position_y    = -0.11    #m
        self.position_z    = 0    #m

        self.alpha         = 0.0       
        self.beta          = 0.0
        self.gamma         = 0.0
        self.angle_of_view = 0.6        #rad  (half angle of view in horizontal direction)
        self.aspect_ratio  = 0.628

class BeamForming:
    def __init__(self) -> None:
        self.distance = 5
        self.mg = MicGeom(from_file='/cae-microphone-array-containerized/src/Extractor_V2/processor/resource/Acoular_data/32_mic_on_the_car.xml')
        self.alignment = cameraAlign()
        self.grid_increment = 0.4
        self.array_arrngmnt = None
        self.x_min_grid = None
        self.x_max_grid = None
        self.y_min_grid = None
        self.y_max_grid = None
        self.bridge = CvBridge()

        self.freq = 1200


        # create the point cloud for the microphone array
        self.cloud =  PointCloud2()
        self.header = std_msgs.msg.Header()
        self.header.stamp = rclpy.clock.Clock().now().to_msg()
        self.header.frame_id = 'mic_array'
        self.cloud.header = self.header

        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        self.fields_with_color = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        self.grid = ImportGrid(from_file='/cae-microphone-array-containerized/src/Extractor_V2/processor/resource/circular_grid.xml')
        self.st = SteeringVector(grid=self.grid, mics=self.mg)

        print("BeamForming initialized")

    def do_beamforming(self, mic_data):
        ts = TimeSamples(data=mic_data, sample_freq=44000)
        ps = PowerSpectra(time_data=ts, block_size=128, window='Hanning')
        bb = BeamformerBase(freq_data=ps, steer=self.st)
        pm = bb.synthetic(1200, 2)
        self.Lm = L_p(pm)
        return

    def draw_beam(self):
        # this function is used to save the beam outcome to an audio point cloud 
        plot_min = self.Lm.max() - 2
        plot_max = self.Lm.max()
        
        # show the point with potential sound source 
        output_point = self.Lm[self.Lm > plot_min]
        grid_output = self.grid.grid[:,self.Lm > plot_min]
        points = []

        for i in range(len(grid_output)):
            point = [grid_output[i,0], grid_output[i,1], grid_output[i,2]]
            points.append(point)

        # create the point cloud accroding to the output from the microphone array
        cloud= pc2.create_cloud(self.header, self.fields, points)
        return cloud

    

class Beamforming_node(Node):
    def __init__(self):
        super().__init__('beamforming_processor',namespace='beamforming')

        # subscribe to the audio and image topics
        self.audio_subscription = self.create_subscription(AvReader,'/extractor/av_message',self.av_callback,1)
        self.audio_cloud_publisher = self.create_publisher(PointCloud2, '/colored_point_cloud', 1)
        self.bridge = CvBridge()
        self.beam = BeamForming()

    def av_callback(self, msg):
        print("the message received")

        # car only has 32 channels
        audio_data = np.array(msg.audio).reshape(-1,32)

        # calculate the beam and publish the audio point cloud 
        self.beam.do_beamforming(audio_data)
        audio_cloud = self.beam.draw_beam()

        # publish the audio point cloud
        self.audio_cloud_publisher.publish(audio_cloud)



def main(args=None):
    rclpy.init(args=args)

    image_size_reader = Beamforming_node()
    rclpy.spin(image_size_reader)
    image_size_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
