import math
import numpy as np
import os

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor 
from ament_index_python.packages import get_package_share_directory

from std_srvs.srv import SetBool


import json

# Global variables
#max_delta_t=0.125
namespace1 = 'tb3_0' # namespace define internally
namespace2 = 'tb3_1' # namespace define internally

b=(0.287/2) #distance between wheels
e=0.2       #off center distance

#Parametros de diseño de la trayectoria
#vel = 0.1
#Ax = 2.5
#Ay = 0.81
#distance = 0.8
##Phi0 depende de estos tres parametros (si se cambian hay que recalcularla en: compute_critic_angle.m )
#phi0 = -0.275

class CentralizedController(Node):
    def __init__(self):
        super().__init__('Centralized_Controller')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.initialized=False
        self.initialized_odom_1=False
        self.initialized_odom_2=False

        ########################### Parametros de valores utilizados para el control #######################
        
        #Velocidad del robot cuando usamos el tipo de trayectoria phi con velocidad constante
        vel_parameter_descriptor = ParameterDescriptor(description='This parameter determines the robot velocity')
        self.declare_parameter('velocity',0.1, vel_parameter_descriptor)
        self.vel=self.get_parameter('velocity').get_parameter_value().double_value
        self.get_logger().info('Velocity: "%s"' % self.vel)

        #Amplitud de la trayectoria en el eje x
        Ax_parameter_descriptor = ParameterDescriptor(description='This parameter determines trajectory amplitude on x axis')
        self.declare_parameter('Ax',2.5, Ax_parameter_descriptor)
        self.Ax=self.get_parameter('Ax').get_parameter_value().double_value
        self.get_logger().info('Ax: "%s"' % self.Ax)

        #Amplitud de la trayectoria en el eje y
        Ay_parameter_descriptor = ParameterDescriptor(description='This parameter determines trajectory amplitude on y axis')
        self.declare_parameter('Ay',0.81, Ay_parameter_descriptor)
        self.Ay=self.get_parameter('Ay').get_parameter_value().double_value
        self.get_logger().info('Ay: "%s"' % self.Ay)

        #Distance between the robots
        distance_parameter_descriptor = ParameterDescriptor(description='This parameter determines the distance between the robots')
        self.declare_parameter('distance',-0.8, distance_parameter_descriptor)
        self.distance=self.get_parameter('distance').get_parameter_value().double_value
        self.get_logger().info('Distance between the robots: "%s"' % self.distance)

        #Phi0 value to go throught the trajectory
        phi0_parameter_descriptor = ParameterDescriptor(description='This parameter determines phi value to go throught the trajectory')
        self.declare_parameter('phi0',-0.275, phi0_parameter_descriptor)
        self.phi0=self.get_parameter('phi0').get_parameter_value().double_value
        self.get_logger().info('phi0: "%s"' % self.phi0)

        #Signal period to set the frecuency control
        max_delta_t_parameter_descriptor = ParameterDescriptor(description='This parameter determines the signal period to set the frecuency control')
        self.declare_parameter('period',0.125, max_delta_t_parameter_descriptor)
        self.max_delta_t=self.get_parameter('period').get_parameter_value().double_value
        self.get_logger().info('Periodo de la señal de control: "%s"' % self.max_delta_t)


        ########################### Parametros modificables desde argumentos del launch ####################
        ########################## para el uso o no de sensores, tipo de controlador... ####################
       
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

        #Usar el Aruco
        aruco_parameter_descriptor = ParameterDescriptor(description='This parameter determines to use Aruco')
        self.declare_parameter('use_aruco',False, aruco_parameter_descriptor)
        self.use_aruco=self.get_parameter('use_aruco').get_parameter_value().bool_value
        self.get_logger().info('Aruco: "%s"' % self.use_aruco)
        
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

        #Usar fusión de Aruco y Odom
        fusion_aruco_odom_parameter_descriptor = ParameterDescriptor(description='This parameter determines to use the fusion from aruco and Odometry')
        self.declare_parameter('use_fusion_aruco_odom',True, fusion_aruco_odom_parameter_descriptor)
        self.use_fusion_aruco_odom=self.get_parameter('use_fusion_aruco_odom').get_parameter_value().bool_value
        self.get_logger().info('Fusion de Aruco y Odometria: "%s"' % self.use_fusion_aruco_odom)

        #Usar parametro phi para velocidad constante o variable
        phi_parameter_descriptor = ParameterDescriptor(description='This parameter determines to use AMCL')
        self.declare_parameter('use_phi',True, phi_parameter_descriptor)
        self.use_phi=self.get_parameter('use_phi').get_parameter_value().bool_value
        self.get_logger().info('Phi trajectory: "%s"' % self.use_phi)

        #Usar parametro close_loop para velocidad constante o variable
        close_loop_parameter_descriptor = ParameterDescriptor(description='This parameter determines to use close or open loop')
        self.declare_parameter('use_close_loop',True, close_loop_parameter_descriptor)
        self.use_close_loop=self.get_parameter('use_close_loop').get_parameter_value().bool_value
        self.get_logger().info('Close Loop: "%s"' % self.use_close_loop)
        
        #Seleccionar el tipo de controlador
        controller_type_parameter_descriptor = ParameterDescriptor(description='This parameter determines the controller_type')
        self.declare_parameter('controller_type','Kp', controller_type_parameter_descriptor)
        self.controller_type=self.get_parameter('controller_type').get_parameter_value().string_value
        self.get_logger().info('Control_type: "%s"' % self.controller_type)

        #Concatenamos para obtener la ruta completa al archivo en el directorio config
        pkg_dir = get_package_share_directory('central_pkg')
        self.controller_type_file_path = os.path.join(pkg_dir, 'config', self.controller_type + '.json')

        #Open json file
        with open(self.controller_type_file_path, 'r') as file:
            params = json.load(file)
        
        self.A=np.asarray(params["A"])
        self.B=np.asarray(params["B"])
        self.C=np.asarray(params["C"])
        self.D=np.asarray(params["D"])
        self.max_delta_t=params["Ts"]


#################################### Publicaciones y Suscripciones #################################################

        # Publishers
        self.publisher1 = self.create_publisher(Twist, f'{namespace1}/cmd_vel', 1) # must be under namespace somehow
        self.publisher2 = self.create_publisher(Twist, f'{namespace2}/cmd_vel', 1) # must be under namespace somehow

        if self.use_broadcast_ref:
            self.publish_reference_1 = self.create_publisher(Float32MultiArray, f'{namespace1}/reference', 1)
            self.publish_reference_2 = self.create_publisher(Float32MultiArray, f'{namespace2}/reference', 1)

            self.publish_reference_broad_1 = self.create_publisher(Odometry, f'{namespace1}/reference_broad', 1)
            self.publish_reference_broad_2 = self.create_publisher(Odometry, f'{namespace2}/reference_broad', 1)


        if self.use_odom_sim:
            self.publish_odom_1 = self.create_publisher(Float32MultiArray, f'{namespace1}/odom_1_control', 1)
            self.publish_odom_2 = self.create_publisher(Float32MultiArray, f'{namespace2}/odom_2_control', 1)


        self.publish_controller_data_1 = self.create_publisher(Float32MultiArray, f'{namespace1}/controller_data', 1)
        self.publish_controller_data_2 = self.create_publisher(Float32MultiArray, f'{namespace2}/controller_data', 1)

        
        #Subscription to odom_topic
        
        if self.use_lsm:
            self.subscriber = self.create_subscription(Odometry, f'{namespace1}/scan_odom', self.odom_callback_1, qos_profile)
            self.subscriber = self.create_subscription(Odometry, f'{namespace2}/scan_odom', self.odom_callback_2, qos_profile)

        else:
            self.subscriber = self.create_subscription(Odometry, f'{namespace1}/odom', self.odom_callback_1, qos_profile)
            self.subscriber = self.create_subscription(Odometry, f'{namespace2}/odom', self.odom_callback_2, qos_profile)
        
        if(self.use_aruco):
            self.aruco_subscriber = self.create_subscription(PoseStamped, f'{namespace2}/servo_pose', self.aruco_callback, qos_profile)
        
        if(self.use_amcl):
            self.amcl_subscriber = self.create_subscription(PoseWithCovarianceStamped, f'{namespace1}/amcl_pose', self.amcl_callback, qos_profile)



        # Parameters
        self.target_distance = self.distance # The target distance between the robots [m]
        self.delta_t = self.max_delta_t # time step [s]
        self.phi1 = 0.0
        self.phi2 = self.phi0
        self.initialized = False
        self.initialized_low_pass_filter = False


        # Reference for relative postion
        self.ref12x=self.distance; #Constant relative position between agents in x
        self.ref12y=0; #Constant relative position between agents in y
    
        # Starting points ODOM
        self.x_rob1 = 0.0 
        self.y_rob1 = 0.0
        self.theta_rob1 = 0.0

        self.x_rob_0_1 = 0.0
        self.y_rob_0_1 = 0.0
        self.theta_rob_0_1 = 0.0

        self.x_rob_0_2 = 0.0
        self.y_rob_0_2 = 0.0
        self.theta_rob_0_2 = 0.0

        self.x_rob_odom1=0.0
        self.y_rob_odom1=0.0
        self.theta_rob_odom1=0.0

        self.x_rob_odom2=0.0
        self.y_rob_odom2=0.0
        self.theta_rob_odom2=0.0

        self.last_odom_x1 = 0.0
        self.last_odom_y1 = 0.0
        self.last_odom_theta1 = 0.0

        self.last_odom_x2 = 0.0
        self.last_odom_y2 = 0.0
        self.last_odom_theta2 = 0.0

        #AMCL from the First robot
        self.x_rob_AMCL_1= 0.0
        self.y_rob_AMCL_1 = 0.0
        self.theta_rob_AMCL_1 = 0.0

        #Aruco from Second robot
        self.x_rob2A = 0.0
        self.y_rob2A = 0.0
        self.theta_rob2A = 0.0

        #Run controller
        self.timer = self.create_timer(self.max_delta_t, self.on_timer)

        self.srv = self.create_service(SetBool, 'Controller_Initialization', self.handle_service_request)

    def handle_service_request(self, request, response):
        self.get_logger().info(f'Solicitud recibida: {request.data}')
        # Pocesamiento de la solicitud
        response.success = True
        response.message = "Solicitud procesada correctamente"
        self.handle_response(response.success)
        return response
    
    def handle_response(self,initialization_flag):
        
        if (initialization_flag):
            self.t = 0.0

            self.u_fb_x = np.zeros([self.D.shape[1],1])
            self.x_c_x = np.zeros([self.A.shape[0],1])
            self.e1_x = np.zeros([self.B.shape[1],1])

            self.u_fb_y = np.zeros([self.D.shape[1],1])
            self.x_c_y = np.zeros([self.A.shape[0],1])
            self.e1_y = np.zeros([self.B.shape[1],1])


            # Starting points
            self.x_rob1 = 0.0 
            self.y_rob1 = 0.0
            self.theta_rob1 = 0.0

            self.x_rob2 = self.ref12x 
            self.y_rob2 = self.ref12y
            self.theta_rob2 = 0.0

            self.phi1 = 0.0
            self.phi2 = self.phi0

            # Boolean initialization
            self.initialized = True
            self.initialized_odom_1 = True
            self.initialized_odom_2 = True


    def reference_trajectory(self):
        # Define a reference trajectory
        if self.initialized:
            self.t=self.t+self.delta_t
            t=self.t

            Ax = self.Ax
            Ay = self.Ay

            x0 = 0.0
            y0 = 0.0
            w = 2 * math.pi/90
            
            theta0_comp = -math.atan2(2*Ay,Ax) # angle to rotate the trajectory

            #######################################################First agent############################################################

            if (self.use_phi):
                dx1_phi=Ax * math.cos(self.phi1)
                dy1_phi=Ay * math.cos(2*self.phi1) * 2
                dphi1_t=self.vel/(math.sqrt(dx1_phi**2+dy1_phi**2))
                self.phi1=self.phi1+dphi1_t*self.delta_t
                x1 = Ax * math.sin(self.phi1)
                y1 = Ay * math.sin(2 * self.phi1)
                xp1 = dx1_phi*dphi1_t
                yp1 = dy1_phi*dphi1_t
                dx1_phi2 = -Ax * math.sin(self.phi1)
                dy1_phi2 =  -Ay * math.sin(2 * self.phi1) * 4 
                xpp1 = dx1_phi2*(dphi1_t**2)
                ypp1 = dy1_phi2*(dphi1_t**2)
            else:
                x1= Ax*math.sin(w*t)
                y1= Ay*math.sin(2*w*t)
                xp1= Ax*math.cos(w*t)*w
                yp1= Ay*math.cos(2*w*t)*2*w
                xpp1= -Ax*math.sin(w*t)*w*w
                ypp1= -Ay*math.sin(2*w*t)*4*w*w
                xppp1= -Ax*math.cos(w*t)*w*w*w
                yppp1= -Ay*math.cos(2*w*t)*8*w*w*w
               

            # Transformed
            c = math.cos(theta0_comp)
            s = math.sin(theta0_comp)
            x1ref = c*x1-s*y1 +x0
            y1ref = s*x1+c*y1 + y0
            x1refp= c*xp1-s*yp1
            y1refp= s*xp1+c*yp1
            x1refpp=c*xpp1-s*ypp1
            y1refpp=s*xpp1+c*ypp1
            #x1refppp=c*xppp1-s*yppp1
            #y1refppp=s*xppp1+c*yppp1
            theta1 = math.atan2(y1refp,x1refp)
            thetap1 = (y1refpp * x1refp - y1refp * x1refpp) / (y1refp * y1refp + x1refp * x1refp)
            #thetapp1 = ((y1refppp * x1refp - x1refppp * y1refp) * (x1refp**2 + y1refp**2) - 2 * (y1refpp * x1refp - x1refpp * y1refp) * (x1refp * x1refpp + y1refp * y1refpp)) / ((x1refp**2 + y1refp**2)**2)

            if(self.use_broadcast_ref):
                # Publish reference trajectory of first agent
                msg_reference_1 = Float32MultiArray()
                msg_reference_1.data = [float(self.t), float(x1refp), float(y1ref), float(theta1), float(x1refp), float(y1refp), float(thetap1)]
                self.publish_reference_1.publish(msg_reference_1)

                #Reference First Agent visualization
                msg_reference_broad_1 = Odometry()
                msg_reference_broad_1.pose.pose.position.x = x1ref
                msg_reference_broad_1.pose.pose.position.y = y1ref
                msg_reference_broad_1.pose.pose.orientation.z = theta1

                msg_reference_broad_1.twist.twist.linear.x = x1refp
                msg_reference_broad_1.twist.twist.linear.y = y1refp
                msg_reference_broad_1.twist.twist.angular.z = thetap1
                self.publish_reference_broad_1.publish(msg_reference_broad_1)

            # Linearization for the first agent
            # Remember the off-center model that we are using
            x1eref= x1ref+e*math.cos(theta1)
            y1eref= y1ref+e*math.sin(theta1)
            x1erefp= x1refp-e*thetap1*math.sin(theta1)
            y1erefp= y1refp+e*thetap1*math.cos(theta1)


            ############################################################################################################################

            #######################################################Second agent###########################################################

            if(self.use_phi):
                dx2_phi=Ax * math.cos(self.phi2)
                dy2_phi=Ay * math.cos(2 * self.phi2) * 2
                dphi2_t=self.vel/(math.sqrt(dx2_phi**2+dy2_phi**2))
                self.phi2=self.phi2+dphi2_t*self.delta_t
                x2 = Ax * math.sin(self.phi2)
                y2 = Ay * math.sin(2 * self.phi2)
                xp2 = dx2_phi*dphi2_t
                yp2 = dy2_phi*dphi2_t
                dx2_phi2 = -Ax * math.sin(self.phi2)
                dy2_phi2 =  -Ay * math.sin(2 * self.phi2) * 4
                xpp2 = dx2_phi2*(dphi2_t**2)
                ypp2 = dy2_phi2*(dphi2_t**2)

                c = math.cos(theta0_comp)
                s = math.sin(theta0_comp)
                x2ref = c*x2-s*y2 +x0
                y2ref = s*x2+c*y2 + y0
                x2refp= c*xp2-s*yp2
                y2refp= s*xp2+c*yp2
                x2refpp=c*xpp2-s*ypp2
                y2refpp=s*xpp2+c*ypp2

            else:

                x2ref= x1ref + math.cos(theta1) * self.ref12x - math.sin(theta1) * self.ref12y
                y2ref= y1ref + math.sin(theta1) * self.ref12x + math.cos(theta1) * self.ref12y
                x2refp= x1refp - thetap1 * (math.sin(theta1) * self.ref12x + math.cos(theta1) * self.ref12y)
                y2refp= y1refp - thetap1 * (-math.cos(theta1) * self.ref12x + math.sin(theta1) * self.ref12y)
                #x2refpp = x1refpp - (thetapp1 * math.sin(theta1) + (thetap1**2) * math.cos(theta1)) * self.ref12x - (thetapp1 * math.cos(theta1) - (thetap1**2) * math.sin(theta1)) * self.ref12y
                #y2refpp = y1refpp + (thetapp1 * math.cos(theta1) - (thetap1**2) * math.sin(theta1)) * self.ref12x - (thetapp1 * math.sin(theta1) + (thetap1**2) * math.cos(theta1)) * self.ref12y
                

            theta2 = math.atan2(y2refp, x2refp)
            thetap2 = (y2refpp * x2refp - x2refpp * y2refp) / (x2refp**2 + y2refp**2)

            if(self.use_broadcast_ref):
                # Publish reference trajectory of first agent -  TO ANALYZE DATA - NOT FUNCTIONAL
                msg_reference_2 = Float32MultiArray()
                msg_reference_2.data = [float(self.t), float(x2refp), float(y2ref), float(theta2), float(x2refp), float(y2refp), float(thetap2)]
                self.publish_reference_2.publish(msg_reference_2)

                #Reference Second Agent visualization
                msg_reference_broad_2 = Odometry()
                msg_reference_broad_2.pose.pose.position.x = x2ref
                msg_reference_broad_2.pose.pose.position.y = y2ref
                msg_reference_broad_2.pose.pose.orientation.z = theta2

                msg_reference_broad_2.twist.twist.linear.x = x2refp
                msg_reference_broad_2.twist.twist.linear.y = y2refp
                msg_reference_broad_2.twist.twist.angular.z = thetap2
                self.publish_reference_broad_2.publish(msg_reference_broad_2)

            # Linearization for the second agent
            # Remember the off-center model that we are using
            x2eref = x2ref + e * math.cos(theta2)
            y2eref = y2ref + e * math.sin(theta2)
            x2erefp = x2refp - e * thetap2 * math.sin(theta2)
            y2erefp = y2refp + e * thetap2 * math.cos(theta2)
            ############################################################################################################################

            ######################################################## Relative Position 12 ##################################################
            self.x12ref= x1ref - x2ref # Relative position between the two base_link
            self.y12ref = y1ref - y2ref
            self.x12eref = x1eref - x2eref # Relative position between off center points
            self.y12eref = y1eref - y2eref

            #############################################################################################################################



            return x1ref, y1ref, x1eref, y1eref, theta1, x1erefp, y1erefp, thetap1, x2ref, y2ref, x2eref, y2eref, theta2, x2erefp, y2erefp, thetap2, self.t

        else:
            self.get_logger().info('Reference trajectory not initialized yet!')

    
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
    
    
    def on_timer(self):
        msg_1 = Twist()
        msg_2 = Twist()
        

        if self.initialized:
            self.get_logger().info('Control initialized!')
           # WATCH OUT: x1, y1 (as well x2, y2) are the reference position without the off-center distance
            x1, y1, x_ref1, y_ref1, theta_ref1, x_ref_vel1, y_ref_vel1, theta_ref_vel1, x2, y2, x_ref2, y_ref2, theta_ref2, x_ref_vel2, y_ref_vel2, theta_ref_vel2, t = self.reference_trajectory()
            

            # -------------- for agent1 ----------------------
            self.x_ff_1 = x_ref_vel1 #velocity off center point reference
            self.y_ff_1 = y_ref_vel1

            #AMCL and Odom for the second robot:
            r1ex = self.x_rob1 + e*math.cos(self.theta_rob1)
            r1ey = self.y_rob1 + e*math.sin(self.theta_rob1)

            #error between off center point reference and off center point base_link
            self.e1_x = x_ref1 - r1ex 
            self.e1_y = y_ref1 - r1ey
            

            # -------------- for agent2 -----------------------
            self.x_ff_2 = x_ref_vel2
            self.y_ff_2 = y_ref_vel2

            #Aruco and Odom for the second robot:
            r2ex = self.x_rob2 + e*math.cos(self.theta_rob2)
            r2ey = self.y_rob2 + e*math.sin(self.theta_rob2)


            # now I need the relative position because the error for the second is build on the relative position 1 and 2
            x12 = r1ex - r2ex
            y12 = r1ey - r2ey
            self.e12_x = -(self.x12eref - x12)
            self.e12_y = -(self.y12eref - y12)

            self.controller()

            # for agent1 and agent2 (close loop)
            if(self.use_close_loop):
                x_vel1 = self.x_ff_1 + self.u_fb_x_1
                y_vel1 = self.y_ff_1 + self.u_fb_y_1
                x_vel2 = self.x_ff_2 + self.u_fb_x_2
                y_vel2 = self.y_ff_2 + self.u_fb_y_2

            # for agent1 and agent2 (open loop)
            else:    
                x_vel1 = self.x_ff_1 
                y_vel1 = self.y_ff_1
                x_vel2 = self.x_ff_2
                y_vel2 = self.y_ff_2

            v_i1 = (1/e) * ((e*math.cos(self.theta_rob1) + b*math.sin(self.theta_rob1))*x_vel1 + (e*math.sin(self.theta_rob1) - b*math.cos(self.theta_rob1))*y_vel1)
            v_d1 = (1/e) * ((e*math.cos(self.theta_rob1) - b*math.sin(self.theta_rob1))*x_vel1 + (e*math.sin(self.theta_rob1) + b*math.cos(self.theta_rob1))*y_vel1)
            v_i2 = (1/e) * ((e*math.cos(self.theta_rob2) + b*math.sin(self.theta_rob2))*x_vel2 + (e*math.sin(self.theta_rob2) - b*math.cos(self.theta_rob2))*y_vel2)
            v_d2 = (1/e) * ((e*math.cos(self.theta_rob2) - b*math.sin(self.theta_rob2))*x_vel2 + (e*math.sin(self.theta_rob2) + b*math.cos(self.theta_rob2))*y_vel2)
            
            msg_1.linear.x = float((v_i1 + v_d1)/2.0)
            msg_2.linear.x = float((v_i2 + v_d2)/2.0)
            msg_1.angular.z = float((v_d1 - v_i1)/(2.0*b))
            msg_2.angular.z = float((v_d2 - v_i2)/(2.0*b))
            
            self.publisher1.publish(msg_1)
            self.publisher2.publish(msg_2)
            
            # Publish controller data of first agent -  TO ANALYZE DATA - NOT FUNCTIONAL
            msg_controller_1 = Float32MultiArray()
            msg_controller_2 = Float32MultiArray()
            msg_controller_1.data = [float(t), float(self.e1_x), float(self.e1_y), float(x_vel1), float(y_vel1), float(self.x_ff_1), float(self.y_ff_1), float(self.u_fb_x_1), float(self.u_fb_y_1), float(x_ref1), float(y_ref1),
                                    float(r1ex), float(r1ey), float(self.x_rob1), float(self.y_rob1), float(self.theta_rob1), float(x1), float(y1), float(theta_ref1),float(self.x_rob_odom1),float(self.y_rob_odom1),float(self.theta_rob_odom1),float(self.x_rob_AMCL_1),float(self.y_rob_AMCL_1),float(self.theta_rob_AMCL_1)]
            msg_controller_2.data = [float(t), float(self.e12_x), float(self.e12_y), float(x_vel2), float(y_vel2), float(self.x_ff_2), float(self.y_ff_2), float(self.u_fb_x_2), float(self.u_fb_y_2), float(x_ref2), float(y_ref2),
                                    float(r2ex), float(r2ey), float(self.x_rob2), float(self.y_rob2), float(self.theta_rob2), float(x2), float(y2), float(theta_ref2),float(self.x_rob_odom2),float(self.y_rob_odom2),float(self.theta_rob_odom2),float(self.x_rob2A),float(self.y_rob2A),float(self.theta_rob2A)]
            self.publish_controller_data_1.publish(msg_controller_1)
            self.publish_controller_data_2.publish(msg_controller_2)
    
    
    def controller(self):

        ux=[[self.e1_x],[self.e12_x]]
        u_fb_x = np.dot(self.C,self.x_c_x)+np.dot(self.D,ux)
        self.x_c_x = np.dot(self.A,self.x_c_x)+np.dot(self.B,ux)

        uy=[[self.e1_y],[self.e12_y]]
        u_fb_y = np.dot(self.C,self.x_c_y)+np.dot(self.D,uy)
        self.x_c_y = np.dot(self.A,self.x_c_y)+np.dot(self.B,uy)

        self.u_fb_x_1=u_fb_x[0][0]
        self.u_fb_x_2=u_fb_x[1][0]
        self.u_fb_y_1=u_fb_y[0][0]
        self.u_fb_y_2=u_fb_y[1][0]

    def odom_callback_1(self, Odometry):
        
        msg = Odometry        
        
        if self.initialized_odom_1:
            
            odom_x= msg.pose.pose.position.x - self.x_rob_0_1
            odom_y= msg.pose.pose.position.y - self.y_rob_0_1
            odom_theta= self.quaternion2theta(msg.pose.pose.orientation) - self.theta_rob_0_1


            inc_x_odom = math.cos(self.theta_rob_0_1) * (odom_x - self.last_odom_x1) + math.sin(self.theta_rob_0_1) * (odom_y - self.last_odom_y1)
            inc_y_odom = -math.sin(self.theta_rob_0_1) * (odom_x - self.last_odom_x1) + math.cos(self.theta_rob_0_1) * (odom_y - self.last_odom_y1)
            inc_theta_odom = odom_theta - self.last_odom_theta1

            self.x_rob1 = self.x_rob1 + (math.cos(inc_theta_odom) * (inc_x_odom) - math.sin(inc_theta_odom) * (inc_y_odom))
            self.y_rob1 = self.y_rob1 + (math.sin(inc_theta_odom) * (inc_x_odom) + math.cos(inc_theta_odom) * (inc_y_odom))
            self.theta_rob1 =self.theta_rob1 + inc_theta_odom 

            self.x_rob_odom1 = self.x_rob_odom1 + (math.cos(inc_theta_odom) * (inc_x_odom) - math.sin(inc_theta_odom) * (inc_y_odom))
            self.y_rob_odom1 = self.y_rob_odom1 + (math.sin(inc_theta_odom) * (inc_x_odom) + math.cos(inc_theta_odom) * (inc_y_odom))
            self.theta_rob_odom1 =self.theta_rob_odom1 + inc_theta_odom 

            self.last_odom_x1 = odom_x
            self.last_odom_y1 = odom_y
            self.last_odom_theta1 = odom_theta

        else:
            self.get_logger().info('Wainting to initialize Odometry_1 from action')
            self.x_rob_0_1 = msg.pose.pose.position.x
            self.y_rob_0_1 = msg.pose.pose.position.y
            self.theta_rob_0_1 = self.quaternion2theta(msg.pose.pose.orientation)


    def amcl_callback(self, PoseWithCovarianceStamped):
        
        msg = PoseWithCovarianceStamped  
        
        self.x_rob_AMCL_1= msg.pose.pose.position.x
        self.y_rob_AMCL_1 = msg.pose.pose.position.y
        self.theta_rob_AMCL_1 = self.quaternion2theta(msg.pose.pose.orientation)

        if(self.use_fusion_amcl_odom):
            self.x_rob1 = msg.pose.pose.position.x
            self.y_rob1 = msg.pose.pose.position.y
            self.theta_rob1 = self.quaternion2theta(msg.pose.pose.orientation)
        

    def odom_callback_2(self, Odometry):

        msg = Odometry        
        
        if self.initialized_odom_2:
            
            odom_x= msg.pose.pose.position.x - self.x_rob_0_2
            odom_y= msg.pose.pose.position.y - self.y_rob_0_2
            odom_theta= self.quaternion2theta(msg.pose.pose.orientation) - self.theta_rob_0_2


            inc_x_odom = math.cos(self.theta_rob_0_2) * (odom_x - self.last_odom_x2) + math.sin(self.theta_rob_0_2) * (odom_y - self.last_odom_y2)
            inc_y_odom = -math.sin(self.theta_rob_0_2) * (odom_x - self.last_odom_x2) + math.cos(self.theta_rob_0_2) * (odom_y - self.last_odom_y2)
            inc_theta_odom = odom_theta - self.last_odom_theta2

            self.x_rob2 = self.x_rob2 + (math.cos(inc_theta_odom) * (inc_x_odom) - math.sin(inc_theta_odom) * (inc_y_odom))
            self.y_rob2 = self.y_rob2 + (math.sin(inc_theta_odom) * (inc_x_odom) + math.cos(inc_theta_odom) * (inc_y_odom))
            self.theta_rob2 =self.theta_rob2 + inc_theta_odom 

            self.x_rob_odom2 = self.x_rob_odom2 + (math.cos(inc_theta_odom) * (inc_x_odom) - math.sin(inc_theta_odom) * (inc_y_odom))
            self.y_rob_odom2 = self.y_rob_odom2 + (math.sin(inc_theta_odom) * (inc_x_odom) + math.cos(inc_theta_odom) * (inc_y_odom))
            self.theta_rob_odom2 =self.theta_rob_odom2 + inc_theta_odom 
            

            self.last_odom_x2 = odom_x
            self.last_odom_y2 = odom_y
            self.last_odom_theta2 = odom_theta


        else:
            self.get_logger().info('Wainting to initialize Odometry_2 from action')
            self.x_rob_0_2 = msg.pose.pose.position.x
            self.y_rob_0_2 = msg.pose.pose.position.y
            self.theta_rob_0_2 = self.quaternion2theta(msg.pose.pose.orientation)
        

    def aruco_callback(self, PoseStamped):
        
        msg2 = PoseStamped

        transformation_matrix = np.array([  
            [0.0, 0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0, 0.0,],
            [0.0, -1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
            ])
        
        tvec = np.array ([msg2.pose.position.x, msg2.pose.position.y, msg2.pose.position.z,1.0])
        new_pose = np.dot(transformation_matrix, tvec)


        rpy=self.euler_from_quaternion(msg2.pose.orientation)

        if rpy[0] < 0:
                thetaA = -(math.pi + rpy[0])
        else:
                thetaA = -(rpy[0] - math.pi)


        xA = new_pose[0]
        yA = new_pose[1]
       

        theta = self.theta_rob1 - thetaA
        lc = 0.21 #distance camera-base_link_rob2
        la = 0.08 #distance aruco_marker-base_link_rob1

        self.x_rob2A = self.x_rob1 - (math.cos(theta)*(lc+xA) - math.sin(theta)*yA) - (math.cos(self.theta_rob1)*la)
        self.y_rob2A = self.y_rob1 - (math.sin(theta)*(lc+xA) + math.cos(theta)*yA) - (math.sin(self.theta_rob1)*la)
        self.theta_rob2A = theta

        if(self.use_fusion_aruco_odom):
            self.x_rob2 = self.x_rob1 - (math.cos(theta)*(lc+xA) - math.sin(theta)*yA) - (math.cos(self.theta_rob1)*la)
            self.y_rob2 = self.y_rob1 - (math.sin(theta)*(lc+xA) + math.cos(theta)*yA) - (math.sin(self.theta_rob1)*la)
            self.theta_rob2 = theta


def main():
    # Inicializa rclpy
    rclpy.init()

    # Crea la instancia del nodo
    central_control_node = CentralizedController()

    try:
        # Ejecuta el nodo hasta que ocurra una interrupción
        rclpy.spin(central_control_node)
    except KeyboardInterrupt:
        # Publica el mensaje de log antes de apagar el contexto de ROS
        if rclpy.ok():  # Asegurarse de que el contexto sigue válido antes de loguear
            central_control_node.get_logger().info('Interrupción detectada, apagando el nodo...')
    finally:
        # Destruir el nodo y apagar rclpy correctamente
        if rclpy.ok():
            central_control_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
