import rclpy
import io
import time
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
import os
from cv_bridge import CvBridge
import numpy as np
import h5py
from extractor_node.msg import AvReader
import matplotlib.pyplot as plt
from acoular import MicGeom, TimeSamples, PowerSpectra, RectGrid,\
SteeringVector, BeamformerBase, BeamformerMusic, BeamformerEig, L_p, Environment
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
    def __init__(self,model) -> None:
        self.distance = 5
        self.mg = MicGeom(from_file='/cae-microphone-array-containerized/src/Extractor_V2/processor/resource/Acoular_data/ourmicarray_56.xml')
        self.alignment = cameraAlign()
        self.grid_increment = 0.4
        self.array_arrngmnt = None
        self.x_min_grid = None
        self.x_max_grid = None
        self.y_min_grid = None
        self.y_max_grid = None
        self.bridge = CvBridge()

        self.freq = 1200

        # create the grid for beamforming 
        width  = 2 * (self.distance - self.alignment.position_z) * np.tan(self.alignment.angle_of_view)
        height = width * self.alignment.aspect_ratio

        if model == "CAMERA":
            self.x_min_grid = -0.5*width - self.alignment.position_x 
            self.x_max_grid = 0.5*width - self.alignment.position_x
            self.y_min_grid = -0.5*height + self.alignment.position_y
            self.y_max_grid = 0.5*height + self.alignment.position_y

        elif model == "MICARRAY":

            self.x_min_grid = -0.5*width
            self.x_max_grid = 0.5*width
            self.y_min_grid = -0.5*height
            self.y_max_grid = 0.5*height 

        self.rg = RectGrid(x_min=self.x_min_grid, x_max=self.x_max_grid, y_min=self.y_min_grid, y_max=self.y_max_grid, z=self.distance, increment=self.grid_increment)
        self.st = SteeringVector(grid=self.rg, mics=self.mg)

        print("BeamForming initialized")

    def do_beamforming(self, mic_data):

        # write the mic_data in a h5 file
        # target_file = '/cae-microphone-array-containerized/src/Extractor_V2/processor/processor/dataset/audio.h5'
        # if os.path.exists(target_file):
        #     os.remove(target_file)
        # with h5py.File(target_file, 'w') as data_file:
        #     data_file.create_dataset('time_data', data=mic_data)
        #     data_file['time_data'].attrs.__setitem__('sample_freq', 44000)
        
        # calculate the beamfomring with the h5 file
        ts = TimeSamples(data=mic_data, sample_freq=44000)
        ps = PowerSpectra(time_data=ts, block_size=128, window='Hanning')
        bb = BeamformerBase(freq_data=ps, steer=self.st)
        pm = bb.synthetic(1200, 2)
        self.Lm = L_p(pm)
        return

    def draw_beam(self):
        # this function is used to save the beam image as a Sensor_msgs/Image message 

        plot_min = self.Lm.max() - 2
        plot_max = self.Lm.max()
        normalized_matrix = np.clip((self.Lm - plot_min) / (plot_max - plot_min) * 255, 0, 255).astype(np.uint8)
        image_matrix = normalized_matrix.T

        # transfer the gray scale image to a color image
        colormap = np.zeros((256, 4), dtype=np.uint8)
        colormap = np.zeros((256, 4), dtype=np.uint8)
        colormap[:, 1] = np.linspace(255, 0, 256) 
        colormap[:, 2] = 255
        colormap[:, 0] = np.linspace(255, 0, 256) 
        colormap[:, 3] = np.linspace(0, 255, 256)  
        color_image = colormap[image_matrix]
        img = np.flipud(color_image)

        ros_image = self.bridge.cv2_to_imgmsg(img, encoding="rgba8")
        return ros_image,img
    
    def draw_overlay(self, cam_pict, beam_pict):

        # add a transparent channel to the camera image
        b, g, r = cv2.split(cam_pict)
        alpha = np.ones_like(b) * 255 
        rgba_image = cv2.merge((b, g, r, alpha))

        # resize the beam image 
        beam_pict_resized = cv2.resize(beam_pict, (cam_pict.shape[1], cam_pict.shape[0]))

        # overlay the images
        overlay_pict = cv2.addWeighted(rgba_image, 1.0, beam_pict_resized, 0.3, 0)
        ros_overlay_pict = self.bridge.cv2_to_imgmsg(overlay_pict, encoding="rgba8")
        return ros_overlay_pict

    

class Beamforming_node(Node):
    def __init__(self):
        super().__init__('beamforming_processor',namespace='beamforming')

        # subscribe to the audio and image topics
        self.subscription1 = self.create_subscription(AvReader,'/extractor/av_message',self.av_callback,1)
        self.beam_image_publisher = self.create_publisher(Image, 'beam_image', 1)
        self.beam_overlay_image_publisher = self.create_publisher(Image,'beam_overlay_image',1)
        self.bridge = CvBridge()
        self.model = "CAMERA"
        self.beam = BeamForming(self.model)

    def av_callback(self, msg):
        print("the message received")
        audio_data = np.array(msg.audio).reshape(-1,56)

        # calculate the beam image
        self.beam.do_beamforming(audio_data)
        beam_image_ros, beam_image = self.beam.draw_beam()

        # overlay the beam image to the camera image
        cam_pict = np.asarray(self.bridge.imgmsg_to_cv2(msg.image,'bgr8'))
        overlay_pict = self.beam.draw_overlay(cam_pict,beam_image)

        # publish the beam and overlay image 
        self.beam_image_publisher.publish(beam_image_ros)
        self.beam_overlay_image_publisher.publish(overlay_pict)
        print("the beam image is published")


def main(args=None):
    rclpy.init(args=args)

    image_size_reader = Beamforming_node()
    rclpy.spin(image_size_reader)
    image_size_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
