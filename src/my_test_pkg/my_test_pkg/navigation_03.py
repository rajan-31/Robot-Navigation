import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2  # v4.6
import numpy as np
import threading

import cv2.aruco as aruco
import time
import math
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Rotation



# Define dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
aruco_params = aruco.DetectorParameters_create()

# next action: turn_cw, turn_ccw, move, idle
aruco_data = {
    -1: { "next_action": "move", "next_id": 0},         # move until you get close to 0
    0: { "next_action": "turn_cw", "next_id": 1},       # turn_cw until you see 1, move until you get close to 1
    1: { "next_action": "turn_ccw", "next_id": 2},      # turn_ccw until you see 2, move until you get close to 2
    2: { "next_action": "turn_ccw", "next_id": 3},      
    3: { "next_action": "turn_ccw", "next_id": 4},      
    4: { "next_action": "idle", "next_id": math.inf},   # idle
}


class Navigation_03(Node):
    def __init__(self):
        super().__init__("navigation_03")
 
    # Create a subscriber with a queue size of 1 to only keep the last frame
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            1  # Queue size of 1
        )
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize CvBridge
        self.bridge = CvBridge()

        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_length = 2
        
        # Variable to store the latest frame
        self.latest_frame = None
        self.frame_lock = threading.Lock()  # Lock to ensure thread safety

        # Flag to control the display loop
        self.running = True

        # Start a separate thread for spinning (to ensure image_callback keeps receiving new frames)
        self.spin_thread = threading.Thread(target=self.spin_thread_func)
        self.spin_thread.start()


        self.target_id = -1
        self.target_action = aruco_data[self.target_id]["next_action"]

    def spin_thread_func(self):
        """Separate thread function for rclpy spinning."""
        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0.05)

    def image_callback(self, msg):
        """Callback function to receive and store the latest frame."""
        # Convert ROS Image message to OpenCV format and store it
        with self.frame_lock:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def camera_info_callback(self, msg):
        """Extract camera parameters from /camera/camera_info."""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def display_image(self):
        """Main loop to process and display the latest frame."""
        # Create a single OpenCV window
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("frame", 800,600)

        while rclpy.ok():
            # Check if there is a new frame available
            if self.latest_frame is not None:

                # Process the current image
                self.process_image(self.latest_frame)

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

    
    def get_marker_center_on_image(self, rvec, tvec):
        # Get origin coordinates in image space
        origin_3d = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)  # Marker's origin in 3D
        image_points, _ = cv2.projectPoints(origin_3d, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        x, y = image_points[0][0]  # Extract pixel coordinates
        
        return x, y

    def process_image(self, img):
        """Image processing task."""
        gray = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        
        if ids is not None:
            aruco.drawDetectedMarkers(self.latest_frame, corners, ids)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            
            for i in range(len(ids)):
                cv2.drawFrameAxes(self.latest_frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], self.marker_length * 0.5)

                

        
        if ids is not None:
            ids_flattened = ids.flatten().tolist()
            requred_id = aruco_data[self.target_id]["next_id"]
            marker_center = [0, 0]
            if(requred_id in ids_flattened):
                index = ids_flattened.index(requred_id)
                print(f"{round(tvecs[index][0][2], 3):<6} {aruco_data[self.target_id]["next_id"]:>2} {self.target_action:>10}")

                marker_center = self.get_marker_center_on_image(rvecs[index][0], tvecs[index][0])

                if self.target_action in ["turn_cw", "turn_ccw"]:
                    if marker_center[0] > 310 and marker_center[0] < 330:
                        self.target_action = "move"


                elif self.target_action == "move":
                    if tvecs[index][0][2] <= 4:
                        self.target_id = aruco_data[self.target_id]["next_id"]
                        self.target_action = aruco_data[self.target_id]["next_action"]
        else:
            print(f"{'':<6} {aruco_data[self.target_id]["next_id"]:>2} {self.target_action:>10}")

        
        twist = Twist()
        match self.target_action:
            case "move":
                twist.linear.x = 0.5
            case "turn_cw":
                twist.angular.z = -0.3
            case "turn_ccw":
                twist.angular.z = 0.3
            case "idle":
                twist.linear.x = 0.0
                twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

        return
    
    def stop(self):
        """Stop the node and the spin thread."""
        self.running = False
        self.spin_thread.join()
 
def main(args=None):
    print("OpenCV version: %s" % cv2.__version__)
    
    rclpy.init(args=args)
    node = Navigation_03()
    
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