import rclpy
import io
import time
import std_msgs.msg
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, PointCloud2, PointField
import os
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import numpy as np
from extractor_node.msg import AvReaderCom
import matplotlib.pyplot as plt
from acoular import MicGeom, TimeSamples, PowerSpectra, RectGrid,\
SteeringVector, BeamformerBase, BeamformerMusic, BeamformerEig, L_p, Environment
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas


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
    def __init__(self,model) -> None:
        self.distance = 3
        self.mg = MicGeom(from_file='/cae-microphone-array-containerized/src/Extractor_V2/processor/resource/Acoular_data/8_mic.xml')
        self.alignment = cameraAlign()
        self.grid_increment = 0.02
        self.array_arrngmnt = None
        self.x_min_grid = None
        self.x_max_grid = None
        self.y_min_grid = None
        self.y_max_grid = None
        self.bridge = CvBridge()
        self.mg.mpos[[0, 2]] = self.mg.mpos[[2, 0]]

        self.freq = 1200

        # create the point cloud for microphone array detections 
        self.cloud =  PointCloud2()
        self.header = std_msgs.msg.Header()
        self.header.stamp = rclpy.clock.Clock().now().to_msg()
        self.header.frame_id = 'cae_micarray_link'
        self.cloud.header = self.header

        self.fields_mic = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        self.fields_with_color = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=16, datatype=PointField.UINT32, count=1)
        ]

        # create the grid for beamforming 
        width  = 2 * (self.distance - self.alignment.position_z) * np.tan(self.alignment.angle_of_view)
        height = width * self.alignment.aspect_ratio
        print(height,width)

        width_new = 4
        height_new = 4

        if model == "CAMERA":
            self.x_min_grid = -0.5*width - self.alignment.position_x 
            self.x_max_grid = 0.5*width - self.alignment.position_x
            self.y_min_grid = -0.5*height + self.alignment.position_y
            self.y_max_grid = 0.5*height + self.alignment.position_y

        elif model == "MICARRAY":

            self.x_min_grid = -0.5*width_new
            self.x_max_grid = 0.5*width_new
            self.y_min_grid = -0.5*height_new
            self.y_max_grid = 0.5*height_new

        self.rg = RectGrid(x_min=self.x_min_grid, x_max=self.x_max_grid, y_min=self.y_min_grid, y_max=self.y_max_grid, z=self.distance, increment=self.grid_increment)
        self.st = SteeringVector(grid=self.rg, mics=self.mg)

        print("BeamForming initialized")

    def do_beamforming(self, mic_data):

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
        print(self.Lm.shape)
        line_image_matrix = image_matrix.reshape(-1)

        # show the point with potential sound source 
        Lm_flat = self.Lm.flatten()
        print(Lm_flat.shape)
        grid_output = self.rg.gpos[:,Lm_flat> plot_min]

        
        # beacause the current design cannot detect the source of sound with respect to the height dimension
        # we just focus on the horizontal dimension
        # row_sums = np.sum(image_matrix, axis=0)
        # same_line_matrix = np.tile(row_sums, (112,1))

        # line_max = same_line_matrix.max()
        # line_min = same_line_matrix.min()
        # normalized_same = np.clip((same_line_matrix - line_min) / (line_max - line_min) * 255, 0, 255).astype(np.uint8)

        # transfer the gray scale image to a color image
        colormap = np.zeros((256, 4), dtype=np.uint8)
        colormap = np.zeros((256, 4), dtype=np.uint8)
        colormap[:, 1] = np.linspace(255, 0, 256) 
        colormap[:, 2] = 255
        colormap[:, 0] = np.linspace(255, 0, 256) 
        colormap[:, 3] = np.linspace(0, 255, 256)  
        color_image = colormap[image_matrix]
        color_matrix = color_image.reshape((-1,4))
        img = np.flipud(color_image)

        ros_image = self.bridge.cv2_to_imgmsg(img, encoding="rgba8")
        points_mic = []
        points_beam = []

        # for i in range(self.rg.gpos.shape[1]):
        #     r = color_matrix[i,0]
        #     g = color_matrix[i,1]
        #     b = color_matrix[i,2]
        #     if line_image_matrix[i] != 0:
        #         rgb = self.rgb_to_uint32(r,g,b)
        #         point = [self.rg.gpos[2,i], self.rg.gpos[0,i], self.rg.gpos[1,i],rgb]
        #         points_beam.append(point)

        for i in range(grid_output.shape[1]):
            point = [grid_output[2,i], grid_output[0,i], grid_output[1,i],255]
            points_beam.append(point)


        for i in range(8):
            point = [self.mg.mpos[2,i], self.mg.mpos[0,i], self.mg.mpos[1,i]]
            points_mic.append(point)
        
        mic_cloud = pc2.create_cloud(self.header, self.fields_mic, points_mic)
        beam_cloud = pc2.create_cloud(self.header, self.fields_with_color, points_beam)

        return ros_image,img, mic_cloud, beam_cloud
    
    def rgb_to_uint32(self, r, g, b, a=255):
        return (r << 24) | (a << 16) | (b << 8) | g

    
    # def draw_overlay(self, cam_pict, beam_pict):

    #     # add a transparent channel to the camera image
    #     b, g, r = cv2.split(cam_pict)
    #     alpha = np.ones_like(b) * 255 
    #     rgba_image = cv2.merge((b, g, r, alpha))

    #     # resize the beam image 
    #     beam_pict_resized = cv2.resize(beam_pict, (cam_pict.shape[1], cam_pict.shape[0]))

    #     # overlay the images
    #     overlay_pict = cv2.addWeighted(rgba_image, 1.0, beam_pict_resized, 0.3, 0)
    #     ros_overlay_pict = self.bridge.cv2_to_imgmsg(overlay_pict, encoding="rgba8")
    #     return ros_overlay_pict

    

class Beamforming_node(Node):
    def __init__(self):
        super().__init__('beamforming_processor',namespace='beamforming')

        # subscribe to the audio and image topics
        self.subscription1 = self.create_subscription(AvReaderCom,'/extractor/av_message',self.av_callback,1)
        self.beam_image_publisher = self.create_publisher(Image, 'beam_image', 1)
        self.beam_overlay_image_publisher = self.create_publisher(CompressedImage,'beam_overlay_image',1)
        self.mic_cloud_publisher = self.create_publisher(PointCloud2, '/mic_point_cloud', 1)
        self.beamforming_cloud_publisher = self.create_publisher(PointCloud2, '/beam_point_cloud', 1)
        self.bridge = CvBridge()
        self.model = "CAMERA"
        self.beam = BeamForming(self.model)

    def av_callback(self, msg):
        print("the message received")
        audio_data = np.array(msg.audio).reshape(-1,8)

        # calculate the beam image
        self.beam.do_beamforming(audio_data)
        beam_image_ros, beam_image, mic_cloud, beam_cloud = self.beam.draw_beam()

        # overlay the beam image to the camera image
        # cam_pict = np.asarray(self.bridge.imgmsg_to_cv2(msg.image,'bgr8'))
        # overlay_pict = self.beam.draw_overlay(cam_pict,beam_image)

        # publish the beam and overlay image 
        self.beam_image_publisher.publish(beam_image_ros)
        self.mic_cloud_publisher.publish(mic_cloud)
        self.beamforming_cloud_publisher.publish(beam_cloud)
        # overlay_pict = msg.compressed_image
        # self.beam_overlay_image_publisher.publish(overlay_pict)
        print("the beam image is published")


def main(args=None):
    rclpy.init(args=args)

    image_size_reader = Beamforming_node()
    rclpy.spin(image_size_reader)
    image_size_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
