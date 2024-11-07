import rclpy
import math
import time
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from geometry_msgs.msg import PoseStamped
from pyquaternion import Quaternion
import cv2
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

t_inicial = time.time()

class ArucoPublisher(Node):
    def __init__(self):
        super().__init__('aruco_publisher')
        self.publisher_ = self.create_publisher(PoseStamped,'aruco_pose', 10)
        timer_period = 1.0 / 30.0  # seconds (1/Hz)
        self.timer = self.create_timer(timer_period, self.camera_callback)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict,self.parameters)

        #Matrix intrinsics and distorsion from 640x480 resolution
        #self.intrinsics = np.array([[502.7067405466173,0,324.0841768359094],[0,502.2196089520196,244.0365732591922],[0,0,1]])
        #self.distortion_params = np.array([0.104265320307726,0.278719406667176,0,0])

        #Matrix intrinsics and distorsion from 1920x1080 resolution
        self.intrinsics = np.array([[1492.044121321748,0,980.4590896715763],[0,1489.826211867192,561.5354742023020],[0,0,1]])
        self.distortion_params = np.array([0.119776955261496,0.145125649383559,0,0])

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def euler_from_quaternion(self, q):
        x = q.x
        y = q.y
        z = q.z
        w = q.w

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return (roll, pitch, yaw)
    
    def quaternion2theta(self,q):
        rpy=self.euler_from_quaternion(q)
        return rpy[0]

    def camera_callback(self):
        ret, frame = self.cap.read()

        markerCorners, markerIds, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)

        #try: 
        #    t_base = self.tf_buffer.lookup_transform('tb3_1/camera_rgb_optical_frame','tb3_1/base_link',rclpy.time.Time())
        #    x_cam2bl = t_base.transform.translation.x
        #    y_cam2bl = t_base.transform.translation.y
        #    theta_cam2bl = self.quaternion2theta(t_base.transform.rotation)
        #    self.get_logger().info('x_cam "%s"' % x_cam2bl)
        #    self.get_logger().info('y_cam "%s"' % y_cam2bl)
        #    self.get_logger().info('theta_cam "%s"' % theta_cam2bl)
        #
        #except TransformException as ex:
        #    self.get_logger().info('Esperando para obtener los frames')
        #    pass

        
       
        if markerIds is not None:
            #cv_image = cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Stima posa del marker: rvec/tvec definiscono il sistema di coordinate destrorse del marker.
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 0.122, self.intrinsics, self.distortion_params)

            if rvecs.size==3: 

                rotation_matrix, _ = cv2.Rodrigues(rvecs)
                q = Quaternion(matrix=rotation_matrix)

            
            #transformation_matrix = np.array([  
            #[0.0, 0.0, 1.0, 0.076],
            #[-1.0, 0.0, 0.0, 0.0,],
            #[0.0, -1.0, 0.0, 0.093],
            #[0.0, 0.0, 0.0, 1.0]
            #])


            #transformation_matrix = np.array([  
            #[0.0, 0.0, 1.0, 0.0],
            #[-1.0, 0.0, 0.0, 0.0,],
            #[0.0, -1.0, 0.0, 0.0],
            #[0.0, 0.0, 0.0, 1.0]
            #])

            #tvec = np.array ([tvecs[0, 0, 0], tvecs[0, 0, 1], tvecs[0, 0, 2],1])

            #new_pose = np.dot(transformation_matrix, tvec)
    
            # Create a PoseStamped message
                self.pose = PoseStamped()
                self.pose.header.frame_id = "camera_info"
                self.pose.header.stamp = self.get_clock().now().to_msg()
                self.pose.pose.position.x = tvecs[0, 0, 0]
                self.pose.pose.position.y = tvecs[0, 0, 1]
                self.pose.pose.position.z = tvecs[0, 0, 2]
                #self.pose.pose.position.x = new_pose[0]
                #self.pose.pose.position.y = new_pose[1]
                #self.pose.pose.position.z = new_pose[2]
                self.pose.pose.orientation.x = q[1]
                self.pose.pose.orientation.y = q[2]
                self.pose.pose.orientation.z = q[3]
                self.pose.pose.orientation.w = q[0]

                self.publish_pose()

            #rpy=self.euler_from_quaternion(self.pose.pose.orientation)

            #theta_rob2_atan = math.atan2(self.pose.pose.position.y, self.pose.pose.position.x)
            #theta_rob2_q2e = self.quaternion2theta(self.pose.pose.orientation)
            
            #self.get_logger().info('x_rob2 "%s"' % self.pose.pose.position.x)
            #self.get_logger().info('y_rob2 "%s"' % self.pose.pose.position.y)
            #self.get_logger().info('z_rob2 "%s"' % self.pose.pose.position.z)

            #theta_rob2_q2e_transf = theta_rob2_q2e 

            

            #self.get_logger().info('Theta_rob2_atan "%s"' % theta_rob2_atan)
            #self.get_logger().info('Theta_rob2_q2e "%s"' % theta_rob2_q2e)
            #self.get_logger().info('Theta_rob2_q2e_transf "%s"' % theta_rob2_q2e_transf)

            #self.get_logger().info('alfa "%s"' % alfa)
            #self.get_logger().info('pitch "%s"' % rpy[1])
            #self.get_logger().info('yaw "%s"' % rpy[2])

        else:
            self.get_logger().info('No Markers Detected Yet')
        

    


    def publish_pose(self):

        if hasattr(self, 'pose') and self.pose is not None:
            self.publisher_.publish(self.pose)
            self.get_logger().info('Pose: ' + str(self.pose))

def main(args=None):
    rclpy.init(args=args)
    aruco_publisher = ArucoPublisher()
    rclpy.spin(aruco_publisher)
    aruco_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()