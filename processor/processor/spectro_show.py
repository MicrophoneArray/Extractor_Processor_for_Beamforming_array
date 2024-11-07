import rclpy
from rclpy.node import Node
import numpy as np
from extractor_node.msg import AvReaderCom
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from scipy.fft import fft
from cv_bridge import CvBridge
import cv2
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

class AudioSpectrumNode(Node):
    def __init__(self):
        super().__init__('audio_spectrum_node')
        
        # Subscribe to the audio message topic
        self.subscription = self.create_subscription(
            AvReaderCom,
            '/extractor/av_message',
            self.audio_callback,
            10
        )
        
        # Publisher for the spectrum image topic
        self.image_pub = self.create_publisher(Image, '/audio_spectrum_image', 10)
        
        # Publisher for the max frequency topic
        self.max_freq_pub = self.create_publisher(Float32, '/spectrogram/max_frequency', 10)
        
        # Bridge to convert OpenCV images to ROS 2 Image messages
        self.bridge = CvBridge()
        
        self.get_logger().info("Audio Spectrum Node initialized.")

    def audio_callback(self, msg):
        # Convert the audio data to a numpy array and select the first channel
        audio_data = np.array(msg.audio, dtype=np.float32).reshape(-1, 56)
        audio_data_list = audio_data[:,[0,8,16,48]]
        print(audio_data.shape)


        # FFT calculation for each side then get the best outcome 

        for i in range(4):
            audio = audio_data_list[:,i]
            N = len(audio)
            yf = fft(audio)
            if i == 0:
                print(np.fft.fftfreq(N, 1 / 48000).shape)
                xf = np.fft.fftfreq(N, 1 / 48000)
                print(xf.shape)
            else:
                print(xf.shape)
                print(np.fft.fftfreq(N, 1 / 48000).shape)
                xf = xf + np.fft.fftfreq(N, 1 / 48000)
                
            idx = np.arange(0, N // 2)  # Only take the positive frequency part
        
        xf, yf = xf[idx], 2.0 / N * np.abs(yf[idx])
        xf = xf/ 4
        
        print(xf.shape,yf.shape)

        # Find the frequency with the highest amplitude
        max_freq_index = np.argmax(yf)
        max_freq = xf[max_freq_index]
        max_amplitude = yf[max_freq_index]

        # Publish the max frequency as a Float32 message
        max_freq_msg = Float32()
        max_freq_msg.data = max_freq
        self.max_freq_pub.publish(max_freq_msg)

        # Log the max frequency and amplitude
        self.get_logger().info(f"Max Frequency: {max_freq:.2f} Hz, Amplitude: {max_amplitude:.2f}")

        # Create a matplotlib figure
        fig, ax = plt.subplots()
        ax.plot(xf, yf)
        ax.set_xlabel("Frequency (Hz)")
        ax.set_xlim(0, 5000)
        ax.set_ylabel("Amplitude")
        ax.set_title("Audio Spectrum")
        ax.grid()

        # Convert the plot to an image (no saving to file)
        canvas = FigureCanvas(fig)
        canvas.draw()
        spectrum_image = np.frombuffer(canvas.tostring_rgb(), dtype='uint8')
        spectrum_image = spectrum_image.reshape(fig.canvas.get_width_height()[::-1] + (3,))

        # Convert the numpy image to ROS 2 Image message
        image_msg = self.bridge.cv2_to_imgmsg(spectrum_image, encoding="rgb8")
        self.image_pub.publish(image_msg)
        self.get_logger().info("Published audio spectrum image.")

        # Close the figure to avoid memory leak
        plt.close(fig)

def main(args=None):
    rclpy.init(args=args)
    audio_spectrum_node = AudioSpectrumNode()
    
    # Keep the node active
    rclpy.spin(audio_spectrum_node)
    
    # Shutdown the node when done
    audio_spectrum_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
