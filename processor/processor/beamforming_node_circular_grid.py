import rclpy
import io
import time
import std_msgs.msg
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, PointCloud2, PointField, PointCloud
import os
from geometry_msgs.msg import Point32
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import numpy as np
from extractor_node.msg import AvReaderCom
import matplotlib.pyplot as plt
import acoular as ac
from acoular import MicGeom, TimeSamples, PowerSpectra, RectGrid,\
SteeringVector, BeamformerBase, BeamformerMusic, BeamformerEig, L_p, Environment
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

import struct


class cameraAlign:

    def __init__(self):
        #default values are based on transformation from ueye/left to mic_array (Use set_attributes to change alignment information)
        self.position_z    = 0.62826  #m
        self.position_y    = -0.2692   #m
        self.position_x    = -0.00404   #m

        self.alpha         = 0.0       
        self.beta          = 0.0
        self.gamma         = 0.0
        self.angle_of_view = 0.71       #rad  (half angle of view in horizontal direction)
        self.aspect_ratio  = 0.6458

class BeamForming:
    def __init__(self) -> None:
        self.distance = 3
        self.mg = MicGeom(from_file='/cae-microphone-array-containerized/src/Extractor_V2/processor/resource/Acoular_data/mic_on_the_car.xml')
        self.alignment = cameraAlign()
        self.grid_increment = 0.02
        self.array_arrngmnt = None
        self.x_min_grid = None
        self.x_max_grid = None
        self.y_min_grid = None
        self.y_max_grid = None
        self.bridge = CvBridge()
        # self.mg.mpos[[0, 2]] = self.mg.mpos[[2, 0]]


        # create the point cloud for microphone array detections 
        self.cloud =  PointCloud2()
        self.header = std_msgs.msg.Header()
        self.header.stamp = rclpy.clock.Clock().now().to_msg()
        self.header.frame_id = 'cae_micarray_link'
        self.cloud.header = self.header
        
        self.fields_point = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        self.grid = ac.ImportGrid(from_file='/cae-microphone-array-containerized/src/Extractor_V2/processor/resource/circular_grid_1.xml')
        self.st = SteeringVector(grid=self.grid, mics=self.mg)


        print("BeamForming initialized")

    def do_beamforming(self, mic_data,freq):

        ts = TimeSamples(data=mic_data, sample_freq=48000)
        ps = PowerSpectra(time_data=ts, block_size=128, window='Hanning')
        bb = BeamformerBase(freq_data=ps, steer=self.st)
        pm = bb.synthetic(freq, 2)
        self.Lm = L_p(pm)
        return

    def draw_beam(self):
        # this function is used to save the beam image as a Sensor_msgs/Image message 

        plot_min = self.Lm.max() - 3
        plot_max = self.Lm.max()
        normalized_matrix = np.clip((self.Lm - plot_min) / (plot_max - plot_min) * 255, 0, 255).astype(np.uint8)
        image_matrix = normalized_matrix.T 

        # show the point with potential sound source 
        Lm_flat = self.Lm.flatten()
        grid_output = self.grid.gpos[:,Lm_flat> plot_min]
        
        # normalize the outcome to every column
        Lm_line = self.Lm.reshape(10,180)
        Lm_line_sum = np.sum(Lm_line, axis=0)/10
        line_max = Lm_line_sum.max()
        line_min = Lm_line_sum.max()-1
        grid_output = self.grid.gpos.reshape(3,10,180)[:,:,Lm_line_sum>line_min].reshape(3,-1)

        points_mic = []
        points_beam = []


        for i in range(grid_output.shape[1]):
            points_beam.append([grid_output[0,i], grid_output[1,i], grid_output[2,i]])

        for i in range(32):
            point = [-self.mg.mpos[0,i], self.mg.mpos[1,i], self.mg.mpos[2,i]]
            points_mic.append(point)
        
        mic_cloud = pc2.create_cloud(self.header, self.fields_point, points_mic)
        beam_cloud = pc2.create_cloud(self.header, self.fields_point, points_beam)

        return mic_cloud, beam_cloud
    


    

class Beamforming_node(Node):
    def __init__(self):
        super().__init__('beamforming_car',namespace='beamforming')

        # subscribe to the audio and image topics
        self.audi_subscriber = self.create_subscription(AvReaderCom,'/extractor/av_message',self.av_callback,1)
        self.beam_image_publisher = self.create_publisher(Image, 'beam_image', 1)
        self.beam_overlay_image_publisher = self.create_publisher(CompressedImage,'beam_overlay_image',1)
        self.mic_cloud_publisher = self.create_publisher(PointCloud2, '/mic_point_cloud', 1)
        self.mic_cloud_cloud_publisher = self.create_publisher(PointCloud2, '/beam_point_cloud', 1)
        self.highest_frequency_subscriber = self.create_subscription(std_msgs.msg.Float32, '/spectrogram/max_frequency', self.highest_frequency_callback, 1)
        self.bridge = CvBridge()
        self.freq = 1200
        self.beam = BeamForming()

    def highest_frequency_callback(self, msg):
        self.freq= msg.data

    def av_callback(self, msg):
        print("the message received")
        audio_data = np.array(msg.audio).reshape(-1,56)
        audio_data = audio_data[:, list(range(0, 24)) + list(range(48, 56))]

        # calculate the beam image
        self.beam.do_beamforming(audio_data,self.freq)
        mic_cloud, beam_cloud = self.beam.draw_beam()


        # publish the beamforming point cloud 
        self.mic_cloud_publisher.publish(mic_cloud)
        self.mic_cloud_cloud_publisher.publish(beam_cloud)
        print("the beam cloud is published")


def main(args=None):
    rclpy.init(args=args)

    image_size_reader = Beamforming_node()
    rclpy.spin(image_size_reader)
    image_size_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
