import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import String
#from irobot_create_msgs.msg import WheelVels

class ArucoPoseSubscriber(Node):

    def __init__(self):
        super().__init__('aruco_pose_subscriber')

        #Sottoscrizione al topic delle pose ArUco
        self.aruco_pose_subscription = self.create_subscription(PoseStamped, 'aruco_pose', self.aruco_pose_callback, 3)

        # Pubblica sul cmd_vel del turtlebot
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Creazione del publisher per l'errore della distanza tra i due turtlebot'
        self.distance_publisher = self.create_publisher(String, 'distanza_turtlebot', 10)

     



        # Parametri per PID velocità lineare
        self.target = 0.4 # target distance to respect
        self.integral_prev = 0.0
        self.dt = 0.2 # must be the same of the frequency of aruco marker
        self.e_prev = 0.0

        self.Kp = 0.1
        self.Ki = 0.0
        self.kd = 0.0


        # Parametri per PID velocità angolare
        self.e_prev2 = 0.0
        self.integral2_prev = 0.0

        self.Kp2 = 0.1
        self.Ki2 = 0.0
        self.kd2 = 0.0



   



    # Callback chiamato quando arriva un messaggio sul topic '/aruco_pose'
    def aruco_pose_callback(self, msg):

        transformation_matrix = np.array([  
            [0.0, 0.0, 1.0, 0.076],
            [-1.0, 0.0, 0.0, 0.0,],
            [0.0, -1.0, 0.0, 0.093],
            [0.0, 0.0, 0.0, 1.0]
        ])

        tvec = np.array ([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 1 ])

        new_pose = np.dot(transformation_matrix, tvec) # to mpve in the base_link reference system
        
        #self.get_logger().info('New_Pose: ' + str(new_pose))

        # Chiama il metodo per muovere il turtlebot in base alla nuova posa calcolata
        self.move_turtlebot(new_pose)
        
      

    def move_turtlebot(self, new_pose):

        
        # Creazione di un messaggio Twist per inviarlo al topic cmd_vel   
        msg = Twist()
        
       
        # Calcolo l'errore tra le direzioni di avanzamento dei due turtlebot
        #self.error2= math.atan2(new_pose[1],new_pose[0])
        self.distance = math.sqrt(new_pose[0]** 2 +new_pose[1]** 2)
        self.distance = self.distance * math.cos(math.atan2(
                    new_pose[1],
                    new_pose[0]))
        self.error =  self.distance - self.target

        if self.error > 0:
            scale_rotation_rate = 0.5
            msg.angular.z = scale_rotation_rate * math.atan2(
                    new_pose[1],
                    new_pose[0])
        
            scale_forward_speed = 0.15
            msg.linear.x = -scale_forward_speed * math.sqrt(
                    new_pose[0] ** 2 +
                    new_pose[1] ** 2)

        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        
        self.cmd_vel_publisher.publish(msg)
        
        self.get_logger().info('Error: ' + str(self.error))
        self.get_logger().info('Distance: ' + str(self.distance))
        
        # PID rotation calculations
        #P2 = self.Kp2*self.error2
        #self.integral2 = self.integral2_prev + self.error2*(self.dt)
        #I2 = self.Ki2*self.integral2
        #D2 = self.kd2*(self.error2 - self.e_prev2)/(self.dt)

        #MV2 = P2 + I2 + D2
        #


        #if MV2 > 0.01 or MV2 <-0.01:
        #    msg.angular.z = MV2
        #else:
        #    msg.angular.z = 0.0

        ## update stored data for next iteration
        #self.e_prev2 = self.error2
        #self.integral2_prev = self.integral2

        #Calcolo la distanza tra i due turtlebot 
        #self.distance = math.sqrt(new_pose[0]** 2 +new_pose[1]** 2)
#
     #
#
        ## Calcolo l'errore
        #if self.distance !=0:
        #    self.error =  self.distance - self.target
        #else:
        #    self.error = 0.0
#
#
        ## Pubblico l'errore per postprocessor
        #msg_error = String()
        #msg_error.data = str(self.error)
        #self.distance_publisher.publish(msg_error)
#
    #
        ## PID calculations
        #P = self.Kp*self.error
#
#
        #self.integral = self.integral_prev + self.error*self.dt
        #
        #self.get_logger().info('Error: "%s"' self.error)
        #
        #
        #I = self.Ki*self.integral
        #D = self.kd*(self.error - self.e_prev)/(self.dt)
#
        ##self.get_logger().info('P: ' + str(P))
        ##self.get_logger().info('I: ' + str(self.integral))
        #    
        #MV = P + I + D
#
        #if MV > 0.05 :
        #    msg.linear.x = MV    
        #else:
        #    msg.linear.x = 0.0
#
        #self.cmd_vel_publisher.publish(msg)
        ## update stored data for next iteration
        #self.e_prev = self.error
        #self.integral_prev = self.integral

        # Pubblicazione del messaggio Twist
        #self.get_logger().info(f"cmd_vel: {str(MV)}")
        #self.cmd_vel_publisher.publish(msg)




    
        
def main(args=None):
    rclpy.init(args=args)
    node2 = ArucoPoseSubscriber()

    
    rclpy.spin(node2)
   

    node2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
