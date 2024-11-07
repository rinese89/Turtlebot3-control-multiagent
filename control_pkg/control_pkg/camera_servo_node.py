import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from pyquaternion import Quaternion
import pigpio
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.servo_pin = 18
        self.freq = 50
        self.theta_Cam = 0.0
        self.duty = 0.0
        #self.init_servo = False
        self.curr_pos = 0.0
        self.pi = pigpio.pi() #conexión con el daemon pigpio
        self.msg=PoseStamped
        self.aruco_flag = False
        
        self.setup_gpio()

        self.subscription = self.create_subscription(PoseStamped,'aruco_pose', self.aruco_pose_callback, qos_profile)

        self.servo_publisher_ = self.create_publisher(PoseStamped,'servo_pose', 10)

        self.timer = self.create_timer(1, self.move_servo)
    
    def setup_gpio(self):
        self.pi.set_PWM_frequency(self.servo_pin, self.freq)
        self.pi.set_PWM_range(self.servo_pin, 20000)  # Rango de 20ms (50Hz)

        self.pi.set_servo_pulsewidth(self.servo_pin, 1500)
        #self.pi.set_servo_pulsewidth(self.servo_pin, self.curr_pos)
        #self.get_logger().info('Posicion Actual: "%s"' % self.curr_pos)
        self.get_logger().info('Servo Initialize')
    
    def aruco_pose_callback(self, PoseStamped):
        
        self.msg = PoseStamped


        #angle = self.quaternion2theta(msg.pose.orientation)
        x = self.msg.pose.position.x
        z = self.msg.pose.position.z

        self.theta_Cam = math.atan2(-x,z)

        #if angle < 0:
        #        self.theta_Cam = -(math.pi + angle)
        #else:
        #        self.theta_Cam = -(angle - math.pi)
        
        self.get_logger().info('x: "%s"' % x)
        self.get_logger().info('z: "%s"' % z)
        self.get_logger().info('Theta_Cam: "%s"' % self.theta_Cam)
        
        #self.move_servo()

        self.aruco_flag = True

    def getTransform(self):
        R=self.quaternion2rot()
        T_cam_aruco = np.array([  
            [R[0][0], R[0][1], R[0][2], self.msg.pose.position.x],
            [R[1][0], R[1][1], R[1][2], self.msg.pose.position.y,],
            [R[2][0], R[2][1], R[2][2], self.msg.pose.position.z],
            [0.0, 0.0, 0.0, 1.0]
            ])

        return T_cam_aruco

    def rot2quaternion(self, MT):

        q=[[0],[0],[0],[0]]

        R = np.array([  
            [MT[0][0], MT[0][1], MT[0][2]],
            [MT[1][0], MT[1][1], MT[1][2]],
            [MT[2][0], MT[2][1], MT[2][2]],
            ])

        traza = np.trace(R)

        if traza > 0:
            
            q[0] = math.sqrt(1+traza)/4
            q[1] = (R[2][1] - R[1][2])/2*math.sqrt(1+traza)
            q[2] = (R[0][2] - R[2][0])/2*math.sqrt(1+traza)
            q[3] = (R[1][0] - R[0][1])/2*math.sqrt(1+traza)

        elif (traza == 0) and (R[0][0] > R[1][1]) and (R[0][0] > R[2][2]):

            q[0] = (R[2][1] - R[1][2])/2*math.sqrt(1+traza)
            q[1] = math.sqrt(1+traza)/4
            q[2] = (R[1][0] + R[0][1])/2*math.sqrt(1+traza)
            q[3] = (R[0][2] + R[2][0])/2*math.sqrt(1+traza)

        elif (traza == 0) and (R[1][1] > R[2][2]):

            q[0] = (R[0][2] - R[2][0])/2*math.sqrt(1+traza)
            q[1] = (R[1][0] + R[0][1])/2*math.sqrt(1+traza)
            q[2] = math.sqrt(1+traza)/4
            q[3] = (R[2][1] + R[1][2])/2*math.sqrt(1+traza)

        else:

            q[0] = (R[1][0] - R[0][1])/2*math.sqrt(1+traza)
            q[1] = (R[0][2] + R[2][0])/2*math.sqrt(1+traza)
            q[2] = (R[2][1] + R[1][2])/2*math.sqrt(1+traza)
            q[3] = math.sqrt(1+traza)/4

        q=[q[0],q[1],q[2],q[3]]
        
        return q 
        
    
    def quaternion2rot(self):

        q=[[0],[0],[0],[0]]
        
        q[1] = self.msg.pose.orientation.x
        q[2] = self.msg.pose.orientation.y
        q[3] = self.msg.pose.orientation.z
        q[0] = self.msg.pose.orientation.w
        

        R = np.array([  
            [1-2*q[2]*q[2]-2*q[3]*q[3], 2*q[1]*q[2]-2*q[3]*q[0], 2*q[1]*q[3]+2*q[2]*q[0]],
            [2*q[1]*q[2]+2*q[3]*q[0], 1-2*q[1]*q[1]-2*q[3]*q[3], 2*q[2]*q[2]-2*q[1]*q[0]],
            [2*q[1]*q[3]-2*q[2]*q[0], 2*q[2]*q[2]+2*q[1]*q[0], 1-2*q[1]*q[1]-2*q[3]*q[3]]
            ])

        return R

        
    def move_servo(self):
        
        if (self.aruco_flag):
            T_cam_aruco=self.getTransform()

            T_servo_cam = np.array([  
                [math.cos(self.curr_pos), 0.0, -math.sin(self.curr_pos), 0.0],
                [0.0, 1.0, 0.0, 0.0,],
                [math.sin(self.curr_pos), 0.0, math.cos(self.curr_pos), 0.04],
                [0.0, 0.0, 0.0, 1.0]
                ])

            T_servo_aruco = np.dot(T_servo_cam, T_cam_aruco)

            quaternion = self.rot2quaternion(T_servo_aruco)

            #self.get_logger().info('Quaternion: "%s"' % quaternion)

            #Publicamos T_servo_aruco
            msg_1 = PoseStamped()
            msg_1.pose.position.x=T_servo_aruco[0][3]
            msg_1.pose.position.y=T_servo_aruco[1][3]
            msg_1.pose.position.z=T_servo_aruco[2][3]

            #msg_1.pose.orientation.x = float(quaternion[1])
            #msg_1.pose.orientation.y = float(quaternion[2])
            #msg_1.pose.orientation.z = float(quaternion[3])
            #msg_1.pose.orientation.w = float(quaternion[0])

            msg_1.pose.orientation.x = quaternion[1]
            msg_1.pose.orientation.y = quaternion[2]
            msg_1.pose.orientation.z = quaternion[3]
            msg_1.pose.orientation.w = quaternion[0]

            self.servo_publisher_.publish(msg_1)


            position = self.curr_pos + self.theta_Cam


            self.get_logger().info('Current Position: "%s"' % self.curr_pos)
            self.get_logger().info('Position: "%s"' % position)
            #self.get_logger().info('Error: "%s"' % error)

            self.curr_pos = position

            command_servo = position*1000/(math.pi/2)+1500
            self.get_logger().info('Comando al servo: "%s"' % command_servo)


            self.pi.set_servo_pulsewidth(self.servo_pin, position*1000/(math.pi/2)+1500)

            #error = self.theta_Cam*1000/(math.pi/2)
        else:
            self.get_logger().info('Esperando recibir señal del arUco')
            self.pi.set_servo_pulsewidth(self.servo_pin, 1500)
            

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
       

def main():
    rclpy.init()
    node = ServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
