import math
import numpy as np
import time
import os

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

import rclpy
from rclpy.node import Node

import json

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TwistWithCovariance
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from geometry_msgs.msg import Quaternion
from action_tutorials_interfaces.action import Trajectory
from rcl_interfaces.msg import ParameterDescriptor 
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory

#Parametros de la generacion de la trayectoria
max_delta_t=0.125
vel = 0.22
b=(0.287/2)
e=0.2

class FrameListener(Node):

    def __init__(self):
        super().__init__('controller_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        #Usar la odometría del Laser Scan Matcher
        lsm_parameter_descriptor = ParameterDescriptor(description='This parameter determines to use the odometry topic from LSM instead of the odom topic from diff_drive_controller')
        self.declare_parameter('use_lsm',False, lsm_parameter_descriptor)
        self.use_lsm=self.get_parameter('use_lsm').get_parameter_value().bool_value
        self.get_logger().info('Laser Scan Matcher: "%s"' % self.use_lsm)

        #Usar el AMCL
        amcl_parameter_descriptor = ParameterDescriptor(description='This parameter determines to use AMCL')
        self.declare_parameter('use_amcl',False, amcl_parameter_descriptor)
        self.use_amcl=self.get_parameter('use_amcl').get_parameter_value().bool_value
        self.get_logger().info('AMCL: "%s"' % self.use_amcl)
        
        #Usar odometría simulada
        odom_sim_parameter_descriptor = ParameterDescriptor(description='This parameter determines to use the odometry simulation')
        self.declare_parameter('use_odom_sim',False, odom_sim_parameter_descriptor)
        self.use_odom_sim=self.get_parameter('use_odom_sim').get_parameter_value().bool_value
        self.get_logger().info('Odometria simulada: "%s"' % self.use_odom_sim)

        #Transmitir la referencia
        broadcast_ref_parameter_descriptor = ParameterDescriptor(description='This parameter determines to broadcast the reference')
        self.declare_parameter('use_broadcast_ref',False, broadcast_ref_parameter_descriptor)
        self.use_broadcast_ref=self.get_parameter('use_broadcast_ref').get_parameter_value().bool_value
        self.get_logger().info('Transmision de la referencia: "%s"' % self.use_broadcast_ref)

        #Usar fusión de AMCL y Odom
        fusion_amcl_odom_parameter_descriptor = ParameterDescriptor(description='This parameter determines to use the fusion from AMCL and Odometry')
        self.declare_parameter('use_fusion_amcl_odom',True, fusion_amcl_odom_parameter_descriptor)
        self.use_fusion_amcl_odom=self.get_parameter('use_fusion_amcl_odom').get_parameter_value().bool_value
        self.get_logger().info('Fusion de AMCL y Odometria: "%s"' % self.use_fusion_amcl_odom)

        #Usar parametro phi para velocidad constante o variable
        phi_parameter_descriptor = ParameterDescriptor(description='This parameter determines to use AMCL')
        self.declare_parameter('use_phi',True, phi_parameter_descriptor)
        self.use_phi=self.get_parameter('use_phi').get_parameter_value().bool_value
        self.get_logger().info('Phi trajectory: "%s"' % self.use_phi)

        #Seleccionar el tipo de controlador
        matrix_file_parameter_descriptor = ParameterDescriptor(description='This parameter determines to use the matrix file')
        self.declare_parameter('matrix_file','Kp.json', matrix_file_parameter_descriptor)
        self.matrix_file=self.get_parameter('matrix_file').get_parameter_value().string_value
        self.get_logger().info('Matrix_file: "%s"' % self.matrix_file)

        #Concatenamos para obtener la ruta completa al archivo en el directorio config
        pkg_dir = get_package_share_directory('control_pkg')
        self.matrix_file_path = os.path.join(pkg_dir, 'config', self.matrix_file)

        if self.use_lsm:
            self.subscriber = self.create_subscription(Odometry, 'scan_odom', self.odom_callback, qos_profile)
        else:
            self.subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, qos_profile)

        if self.use_amcl:
            self.subscriber_amcl = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.amcl_callback, qos_profile)
        
        #Open json file
        with open(self.matrix_file_path, 'r') as file:
            params = json.load(file)
        
        self.A=np.asarray(params["A"])
        self.B=np.asarray(params["B"])
        self.C=np.asarray(params["C"])
        self.D=np.asarray(params["D"])
        max_delta_t=params["Ts"]

        # Control Action Publisher
        self.publish_control_action = self.create_publisher(Twist, 'cmd_vel', 1)
        # Control Data Publisher
        self.publish_control_data = self.create_publisher(Float32MultiArray, 'controller', 1)
        
        #To broadcast the reference
        if self.use_broadcast_ref:
            self.publish_reference = self.create_publisher(Float32MultiArray, 'reference', 1)
        
        #On timer method
        self.timer = self.create_timer(max_delta_t, self.on_timer)
        self.delta_t=max_delta_t
        self.t=0.0
        self.initialized_trajectory=False
        self.initialized_odom=False
        self.init_sim_flag = False
        
        #Odometry initialization
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.last_odom_theta = 0.0

        #--Odometry_simulation--
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.theta_pos = 0.0
        self.x_vel = 0.0
        self.y_vel = 0.0
        self.theta_vel = 0.0
        self.t_ini = 0.0
        self.t_fin = 0.0

        self.t_odom = 0.0

        # Phi trajectory
        self.phi = 0.0

        ##################################################################################################################################################

        self.u_fb_x = np.zeros([self.D.shape[1],1])
        self.x_c_x = np.zeros([self.A.shape[0],1])
        self.e1_x = np.zeros([self.B.shape[1],1])

        self.u_fb_y = np.zeros([self.D.shape[1],1])
        self.x_c_y = np.zeros([self.A.shape[0],1])
        self.e1_y = np.zeros([self.B.shape[1],1])

        self.x_rob=0.0
        self.y_rob=0.0
        self.theta_rob=0.0
        self.x_rob_odom=0.0
        self.y_rob_odom=0.0
        self.theta_rob_odom=0.0
        self.x_rob_AMCL=0.0
        self.y_rob_AMCL=0.0
        self.theta_rob_AMCL=0.0

        self.y_rob_lsm_0 = 0.0
        self.x_rob_lsm_0 = 0.0
        self.theta_rob_lsm_0 = 0.0

        self.x_rob_lsm = 0.0
        self.y_rob_lsm = 0.0
        self.theta_rob_lsm = 0.0


        self._action_server = ActionServer(self,Trajectory,'Start_controller_node',self.execute_callback)
    
    def execute_callback(self, goal_handle):

        self.u_fb_x = np.zeros([self.D.shape[1],1])
        self.x_c_x = np.zeros([self.A.shape[0],1])
        self.e1_x = np.zeros([self.B.shape[1],1])

        self.u_fb_y = np.zeros([self.D.shape[1],1])
        self.x_c_y = np.zeros([self.A.shape[0],1])
        self.e1_y = np.zeros([self.B.shape[1],1])
        self.t = 0.0
        
        self.phi = 0.0

        self.initialized_trajectory= True
        self.initialized_odom = True

        goal_handle.succeed()

        result = Trajectory.Result()
        return result

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
        return rpy[2]

    def quaternion_from_euler(self,roll, pitch, yaw):
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr

        return q
    def theta2quaternion(self, theta):
        return self.quaternion_from_euler(0.0,0.0,theta)

    def on_timer(self):
           
        msg = Twist()
            
        if self.initialized_trajectory:
            x_ref, y_ref, theta_ref, x_ref_vel, y_ref_vel, theta_ref_vel, t = self.reference_trajectory()

            if (self.init_sim_flag == False):
                self.theta_pos = theta_ref
                self.init_sim_flag = True

            self.x_ff = x_ref_vel - e*theta_ref_vel*math.sin(theta_ref)
            self.y_ff = y_ref_vel + e*theta_ref_vel*math.cos(theta_ref)

            xe = x_ref + e*math.cos(theta_ref)
            ye = y_ref + e*math.sin(theta_ref)

            rex = self.x_rob + e*math.cos(self.theta_rob)
            rey = self.y_rob + e*math.sin(self.theta_rob)

            self.e1_x = (x_ref + e*math.cos(theta_ref)) - (self.x_rob + e*math.cos(self.theta_rob))
            self.e1_y = (y_ref + e*math.sin(theta_ref)) - (self.y_rob + e*math.sin(self.theta_rob))

            self.controller()

            x_vel = self.u_fb_x + self.x_ff
            y_vel = self.u_fb_y + self.y_ff

            #-----------------------------------------------------------------------------------------

            v_i = (1/e) * ((e*math.cos(self.theta_rob) + b*math.sin(self.theta_rob))*x_vel + (e*math.sin(self.theta_rob) - b*math.cos(self.theta_rob))*y_vel)
            v_d = (1/e) * ((e*math.cos(self.theta_rob) - b*math.sin(self.theta_rob))*x_vel + (e*math.sin(self.theta_rob) + b*math.cos(self.theta_rob))*y_vel) 

            msg.linear.x = float((v_i + v_d)/2)
            msg.angular.z = float((v_d - v_i)/(2*b))

            #---To simulate Odometry----
            if self.use_odom_sim:
                self.get_logger().info('Using Odom Simulation')
                self.odometry_simulation(t,msg.linear.x,msg.angular.z)
            #----------------------------

            self.publish_control_action.publish(msg)

            msg1 = Float32MultiArray()
            msg1.data = [float(t), float(self.e1_x), float(self.e1_y), float(x_vel), float(y_vel), float(self.x_ff), float(self.y_ff), float(self.u_fb_x), float(self.u_fb_y), float(xe), float(ye), float(rex), float(rey),float(self.x_rob),float(self.y_rob),float(x_ref),float(y_ref),float(self.theta_rob),float(theta_ref),float(self.x_pos),float(self.y_pos),float(self.theta_pos),float(self.x_vel),float(self.y_vel),float(self.theta_vel),float(self.x_rob_odom),float(self.y_rob_odom),float(self.theta_rob_odom),float(self.x_rob_AMCL),float(self.y_rob_AMCL),float(self.theta_rob_AMCL)]

            self.publish_control_data.publish(msg1)
            
    def controller(self):

        self.u_fb_x = np.dot(self.C,self.x_c_x) + np.dot(self.D,self.e1_x)
        self.x_c_x = np.dot(self.A,self.x_c_x) + np.dot(self.B,self.e1_x)

        self.u_fb_y = np.dot(self.C,self.x_c_y) + np.dot(self.D,self.e1_y)
        self.x_c_y = np.dot(self.A,self.x_c_y) + np.dot(self.B,self.e1_y)

    def odometry_simulation(self, t, vel_linear, vel_angular):

        self.t_fin = t

        self.x_vel = math.cos(self.theta_pos) * vel_linear
        self.y_vel = math.sin(self.theta_pos) * vel_linear
        self.theta_vel = vel_angular

        self.x_pos = self.x_pos + self.x_vel * (self.t_fin - self.t_ini) 
        self.y_pos = self.y_pos + self.y_vel * (self.t_fin - self.t_ini) 
        self.theta_pos = self.theta_pos + self.theta_vel * (self.t_fin - self.t_ini)

        self.t_ini = self.t_fin

###------Suscripcion al topic odom o odom_scan, según convenga

    def odom_callback(self, Odometry):
        
        msg = Odometry        
        
        if self.initialized_odom:
            
            self.get_logger().info('Initialized Odometry')

            odom_x= msg.pose.pose.position.x - self.x_rob_0
            odom_y= msg.pose.pose.position.y - self.y_rob_0
            odom_theta= self.quaternion2theta(msg.pose.pose.orientation) - self.theta_rob_0

            inc_x_odom = math.cos(self.theta_rob_0) * (odom_x - self.last_odom_x) + math.sin(self.theta_rob_0) * (odom_y - self.last_odom_y)
            inc_y_odom = -math.sin(self.theta_rob_0) * (odom_x - self.last_odom_x) + math.cos(self.theta_rob_0) * (odom_y - self.last_odom_y)
            inc_theta_odom = odom_theta - self.last_odom_theta

            self.x_rob = self.x_rob + (math.cos(inc_theta_odom) * (inc_x_odom) - math.sin(inc_theta_odom) * (inc_y_odom))
            self.y_rob = self.y_rob + (math.sin(inc_theta_odom) * (inc_x_odom) + math.cos(inc_theta_odom) * (inc_y_odom))
            self.theta_rob =self.theta_rob + inc_theta_odom 

            self.x_rob_odom = self.x_rob_odom + (math.cos(inc_theta_odom) * (inc_x_odom) - math.sin(inc_theta_odom) * (inc_y_odom))
            self.y_rob_odom = self.y_rob_odom + (math.sin(inc_theta_odom) * (inc_x_odom) + math.cos(inc_theta_odom) * (inc_y_odom))
            self.theta_rob_odom =self.theta_rob_odom + inc_theta_odom 
            
            self.last_odom_x = odom_x
            self.last_odom_y = odom_y
            self.last_odom_theta = odom_theta

        else:
            self.get_logger().info('Wainting to initialize Odometry from action')
            self.x_rob_0 = msg.pose.pose.position.x
            self.y_rob_0 = msg.pose.pose.position.y
            self.theta_rob_0 = self.quaternion2theta(msg.pose.pose.orientation)

    def amcl_callback(self, PoseWithCovarianceStamped):
        
        self.get_logger().info('AMCL activate')
        msg = PoseWithCovarianceStamped  
        
        self.x_rob_AMCL= msg.pose.pose.position.x
        self.y_rob_AMCL = msg.pose.pose.position.y
        self.theta_rob_AMCL = self.quaternion2theta(msg.pose.pose.orientation)
        
        if self.use_fusion_amcl_odom:
            self.get_logger().info('Using data fusion.')
            self.x_rob = msg.pose.pose.position.x
            self.y_rob = msg.pose.pose.position.y
            self.theta_rob = self.quaternion2theta(msg.pose.pose.orientation)
        
###-------Generacion de la referencia. Se pasará como feedforward al controlador:

    def reference_trajectory(self):
        
        if self.initialized_trajectory:
            
            self.t=self.t+self.delta_t
            t=self.t
            
            Ax = 1.5
            Ay = 0.6
            x0 = 0.0
            y0 = 0.0
            w = 2 * math.pi/90
            A = 1.0
            
            theta0_comp = -math.atan2(2*Ay,Ax)
        ######################################### Constant Velocity ########################################################
            if self.use_phi:
                dx_phi=Ax * math.cos(self.phi)
                dy_phi=Ay * math.cos(2*self.phi) * 2
                dphi_t=vel/(math.sqrt(dx_phi**2+dy_phi**2))
                self.phi=self.phi+dphi_t*self.delta_t
                x = Ax * math.sin(self.phi)
                y = Ay * math.sin(2 * self.phi)
                xp = dx_phi*dphi_t
                yp = dy_phi*dphi_t
                dx_phi2 = -Ax * math.sin(self.phi)
                dy_phi2 =  -Ay * math.sin(2 * self.phi) * 4 
                xpp = dx_phi2*(dphi_t**2)
                ypp = dy_phi2*(dphi_t**2)
            
        ######################################## Variable Velocity #################################################
            else:
                x = Ax * math.sin(w * t)
                y = Ay * math.sin(2 * w * t)
                xp = Ax * math.cos(w * t) * w
                yp = Ay * math.cos(2 * w * t) * 2 * w
                xpp = -Ax * math.sin(w * t) * w * w
                ypp = -Ay * math.sin(2 * w * t) * 4 * w * w


        # Transform to initialize in the same orientation that the robot
            c = math.cos(theta0_comp)
            s = math.sin(theta0_comp)
            x1 = c*x-s*y +x0
            y1 = s*x+c*y + y0
            xp1= c*xp-s*yp
            yp1= s*xp+c*yp
            xpp1=c*xpp-s*ypp
            ypp1=s*xpp+c*ypp

            theta1 = math.atan2(yp1,xp1)
            thetap1 = (ypp1*xp1-yp1*xpp1)/(yp1*yp1+xp1*xp1)
       
       ################################################################################################################

            msg = Float32MultiArray()

            msg.data = [float(t), float(x1), float(y1), float(theta1), float(xp1), float(yp1), float(thetap1)]
            
            if self.use_broadcast_ref:
                self.publish_reference.publish(msg)

            return(x1,y1,theta1,xp1,yp1,thetap1,t)
        else:
            self.get_logger().info('Trajectory not initialize')

def main():
    # Inicializa rclpy
    rclpy.init()

    # Crea la instancia del nodo
    controller_node = FrameListener()

    try:
        # Ejecuta el nodo hasta que ocurra una interrupción
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        # Publica el mensaje de log antes de apagar el contexto de ROS
        if rclpy.ok():  # Asegurarse de que el contexto sigue válido antes de loguear
            controller_node.get_logger().info('Interrupción detectada, apagando el nodo...')
    finally:
        # Asegúrate de destruir el nodo y apagar rclpy correctamente
        if rclpy.ok():
            controller_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()