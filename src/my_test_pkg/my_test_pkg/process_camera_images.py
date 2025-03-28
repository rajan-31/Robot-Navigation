import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2  # v4.6
import numpy as np
import threading

import cv2.aruco as aruco

# Define dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
aruco_params = aruco.DetectorParameters_create()

class ProcessCameraImages(Node):
    def __init__(self):
        super().__init__("process_camera_images")
 
    # Create a subscriber with a queue size of 1 to only keep the last frame
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            1  # Queue size of 1
        )

        # self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Variable to store the latest frame
        self.latest_frame = None
        self.frame_lock = threading.Lock()  # Lock to ensure thread safety

        # Flag to control the display loop
        self.running = True

        # Start a separate thread for spinning (to ensure image_callback keeps receiving new frames)
        self.spin_thread = threading.Thread(target=self.spin_thread_func)
        self.spin_thread.start()

    def spin_thread_func(self):
        """Separate thread function for rclpy spinning."""
        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0.05)

    def image_callback(self, msg):
        """Callback function to receive and store the latest frame."""
        # Convert ROS Image message to OpenCV format and store it
        with self.frame_lock:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def display_image(self):
        """Main loop to process and display the latest frame."""
        # Create a single OpenCV window
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("frame", 800,600)

        while rclpy.ok():
            # Check if there is a new frame available
            if self.latest_frame is not None:

                # Process the current image
                # self.process_image(self.latest_frame)

                # ===========================
                gray = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2GRAY)
                corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

                if ids is not None:
                    aruco.drawDetectedMarkers(self.latest_frame, corners, ids)
                # ===========================

                # Show the latest frame
                cv2.imshow("frame", self.latest_frame)
                self.latest_frame = None  # Clear the frame after displaying

            # Check for quit key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break

        # Close OpenCV window after quitting
        cv2.destroyAllWindows()
        self.running = False

    def process_image(self, img):
        """Image processing task."""
        
        return
    
    def stop(self):
        """Stop the node and the spin thread."""
        self.running = False
        self.spin_thread.join()
 
def main(args=None):
    print("OpenCV version: %s" % cv2.__version__)
    
    rclpy.init(args=args)
    node = ProcessCameraImages()
    
    try:
        node.display_image()  # Run the display loop
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()  # Ensure the spin thread and node stop properly
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()