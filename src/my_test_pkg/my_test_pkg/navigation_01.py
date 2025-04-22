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


# Define dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
aruco_params = aruco.DetectorParameters_create()

aruco_data = {
    # x, y, z, roll, pitch, yaw (in degrees)
    0: {"x":0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 0},
    1: {"x":9, "y": 5, "z": 0, "roll": 0, "pitch": 0, "yaw": 1.57},
    2: {"x":-2, "y": 10, "z": 0, "roll": 0, "pitch": 0, "yaw": 0.7},
    3: {"x":-5, "y": -8, "z": 0, "roll": 0, "pitch": 0, "yaw": -2.79},
}

class Navigation_01(Node):
    def __init__(self):
        super().__init__("navigation_01")
 
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


        self.detected_id = None
        self.tvecs = None
        self.rvecs = None

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

    def process_image(self, img):
        """Image processing task."""
        gray = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        
        if ids is not None:
            aruco.drawDetectedMarkers(self.latest_frame, corners, ids)
            self.rvecs, self.tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            # print("# ==================================================")
            # print([180-np.degrees(round(vec, 2)) for vec in self.rvecs[0][0]][1], self.tvecs[0][0])
            # print(ids[0][0], [f"{vec:.4f}" for vec in self.rvecs[0][0]], [f"{vec:.4f}" for vec in self.tvecs[0][0]])
            print(f"{ids[0][0]:>5} {'|':>3} {[f'{v:.4f}' for v in self.rvecs[0][0]][0]:>10} {[f'{v:.4f}' for v in self.rvecs[0][0]][1]:>10} {[f'{v:.4f}' for v in self.rvecs[0][0]][2]:>10} {'|':>3} {[f'{v:.4f}' for v in self.tvecs[0][0]][0]:>10} {[f'{v:.4f}' for v in self.tvecs[0][0]][1]:>10} {[f'{v:.4f}' for v in self.tvecs[0][0]][2]:>10}")


            # result_yaw = aruco_data[0]["yaw"] - (math.pi - rvecs[0][0][1])
            # x_ = aruco_data[0]["x"] + tvecs[0][0][2] * math.cos(result_yaw - math.pi/2)
            # y_ = aruco_data[0]["y"] + tvecs[0][0][2] * math.sin(result_yaw - math.pi/2)
            # print(f"({round(x_, 3)}, {round(y_, 3)}) {round(result_yaw, 3)}")

            # take first detected marker only
            self.detected_id = ids[0][0]
            
            for i in range(len(ids)):
                cv2.drawFrameAxes(self.latest_frame, self.camera_matrix, self.dist_coeffs, self.rvecs[i], self.tvecs[i], self.marker_length * 0.5)


        # if(self.rvecs is not None and self.tvecs is not None):
        #     print(f"Marker Id: {self.detected_id}, rvec: {self.rvecs[0][0][2]}, tvec: {self.tvecs[0][0][2]}")

        MIN_DISTANCE = 4

        if(self.detected_id == 0):
            angle = -0.1
            twist = Twist()
            twist.angular.z = float(angle)
            self.cmd_vel_publisher.publish(twist)




        if self.rvecs is not None and self.detected_id == 1 and self.rvecs[0][0][2] > -0.01 and self.rvecs[0][0][2] < 0.01:
            angle = 0
            translation = 0.5
            twist = Twist()
            twist.angular.z = float(angle)
            twist.linear.x = float(translation)
            self.cmd_vel_publisher.publish(twist)

        if self.tvecs is not None and self.detected_id == 1 and self.tvecs[0][0][2] <= MIN_DISTANCE:
            angle = 0.1
            translation = 0
            twist = Twist()
            twist.angular.z = float(angle)
            twist.linear.x = float(translation)
            self.cmd_vel_publisher.publish(twist)

        


        # compared to above, below two variants will continuosly do course correction
        if self.rvecs is not None and self.tvecs is not None and self.detected_id == 2 and self.rvecs[0][0][1] < 3.14 and self.tvecs[0][0][2] > MIN_DISTANCE:
            angle = 0.1
            translation = 0
            twist = Twist()
            twist.angular.z = float(angle)
            twist.linear.x = float(translation)
            self.cmd_vel_publisher.publish(twist)

        if self.rvecs is not None and self.tvecs is not None and self.detected_id == 2 and self.rvecs[0][0][1] >= 3 and self.tvecs[0][0][2] > MIN_DISTANCE:
            angle = 0
            translation = 0.5
            twist = Twist()
            twist.angular.z = float(angle)
            twist.linear.x = float(translation)
            self.cmd_vel_publisher.publish(twist)

        if self.tvecs is not None and self.detected_id == 2 and self.tvecs[0][0][2] <= MIN_DISTANCE:
            angle = 0.1
            translation = 0
            twist = Twist()
            twist.angular.z = float(angle)
            twist.linear.x = float(translation)
            self.cmd_vel_publisher.publish(twist)





        if self.rvecs is not None and self.tvecs is not None and self.detected_id == 3 and self.rvecs[0][0][1] < 3.14 and self.tvecs[0][0][2] > MIN_DISTANCE:
            angle = 0.1
            translation = 0
            twist = Twist()
            twist.angular.z = float(angle)
            twist.linear.x = float(translation)
            self.cmd_vel_publisher.publish(twist)

        if self.rvecs is not None and self.tvecs is not None and self.detected_id == 3 and self.rvecs[0][0][1] >= 3 and self.tvecs[0][0][2] > MIN_DISTANCE:
            angle = 0
            translation = 0.5
            twist = Twist()
            twist.angular.z = float(angle)
            twist.linear.x = float(translation)
            self.cmd_vel_publisher.publish(twist)

        if self.tvecs is not None and self.detected_id == 3 and self.tvecs[0][0][2] <= MIN_DISTANCE:
            angle = 0.1
            translation = 0
            twist = Twist()
            twist.angular.z = float(angle)
            twist.linear.x = float(translation)
            self.cmd_vel_publisher.publish(twist)





        if self.rvecs is not None and self.tvecs is not None and self.detected_id == 4 and self.rvecs[0][0][2] > -0.01 and self.rvecs[0][0][2] < 0.01:
            angle = 0
            translation = 0.5
            twist = Twist()
            twist.angular.z = float(angle)
            twist.linear.x = float(translation)
            self.cmd_vel_publisher.publish(twist)


        return
    
    def get_camera_pose_in_world(self, rvec, tvec, marker_pose_world):
        """Compute the camera's pose in the world frame given the ArUco marker's world pose.
        
        Args:
            rvec: Rotation vector from ArUco detection (marker-to-camera).
            tvec: Translation vector from ArUco detection (marker-to-camera).
            marker_pose_world: (x, y, z, roll, pitch, yaw) of marker in world frame.
        
        Returns:
            camera_pose_world: (x, y, z, roll, pitch, yaw) of camera in world frame.
        """
        # Convert rotation vector to rotation matrix
        R_marker_to_camera, _ = cv2.Rodrigues(rvec)
        
        # Convert marker world rotation (roll, pitch, yaw) to rotation matrix
        R_marker_to_world = R.from_euler('xyz', marker_pose_world[3:], degrees=True).as_matrix()
        
        # Compute world-to-camera rotation
        R_camera_to_world = R_marker_to_world @ R_marker_to_camera.T
        
        # Compute camera position in world frame
        camera_position_world = R_marker_to_world @ (-R_marker_to_camera.T @ tvec) + np.array(marker_pose_world[:3]).reshape(3, 1)
        
        # Compute camera orientation in world frame
        r = R.from_matrix(R_camera_to_world)
        roll, pitch, yaw = r.as_euler('xyz', degrees=True)
        
        return tuple(camera_position_world.flatten()[:3]), (roll, pitch, yaw)
    
    def stop(self):
        """Stop the node and the spin thread."""
        self.running = False
        self.spin_thread.join()
 
def main(args=None):
    print("OpenCV version: %s" % cv2.__version__)
    
    rclpy.init(args=args)
    node = Navigation_01()
    
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