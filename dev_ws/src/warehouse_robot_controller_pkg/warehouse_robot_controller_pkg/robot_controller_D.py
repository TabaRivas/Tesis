# Author: Addison Sears-Collins
# Date: March 19, 2021
# ROS Version: ROS 2 Foxy Fitzroy
 
############## IMPORT LIBRARIES #################
# Python math library
import math 
 
# ROS client library for Python
import rclpy 
 
# Enables pauses in the execution of code
from time import sleep 
 
# Used to create nodes
from rclpy.node import Node
 
# Enables the use of the string message type
from std_msgs.msg import String 
 
# Twist is linear and angular velocity
from geometry_msgs.msg import Twist     
                     
# Handles LaserScan messages to sense distance to obstacles (i.e. walls)        
from sensor_msgs.msg import LaserScan    
 
# Handle Pose messages
from geometry_msgs.msg import Pose 
 
# Handle float64 arrays
from std_msgs.msg import Float64MultiArray
                     
# Handles quality of service for LaserScan data
from rclpy.qos import qos_profile_sensor_data 
 
# Scientific computing library
import numpy as np 
 
class Controller(Node):
  """
  Create a Controller class, which is a subclass of the Node 
  class for ROS2.
  """
  def __init__(self_RA):
    """
    Class constructor to set up the node
    """
    ##################### ROS SETUP ####################################################
    # Initiate the Node class's constructor and give it a name
    super().__init__('Controller')
 
    # Create a subscriber
    # This node subscribes to messages of type Float64MultiArray  
    # over a topic named: /demo/state_est
    # The message represents the current estimated state:
    #   [x, y, yaw]
    # The callback function is called as soon as a message 
    # is received.
    # The maximum number of queued messages is 10.
    self_RA.subscription = self_RA.create_subscription(
                        Float64MultiArray,
                        '/robot_D/state_est',
                        self_RA.state_estimate_callback,
                        10)
    self_RA.subscription  # prevent unused variable warning
 
    # Create a subscriber
    # This node subscribes to messages of type 
    # sensor_msgs/LaserScan     
    self_RA.scan_subscriber = self_RA.create_subscription(
                           LaserScan,
                           '/robot_D/laser/out',
                           self_RA.scan_callback,
                           qos_profile=qos_profile_sensor_data)
                            
    # Create a publisher
    # This node publishes the desired linear and angular velocity of the robot (in the
    # robot chassis coordinate frame) to the /demo/cmd_vel topic. Using the diff_drive
    # plugin enables the robot model to read this /demo/cmd_vel topic and execute
    # the motion accordingly.
    self_RA.publisher_ = self_RA.create_publisher(
                      Twist, 
                      '/robot_D/cmd_vel', 
                      10)
 
    # Initialize the LaserScan sensor readings to some large value
    # Values are in meters.
    self_RA.left_dist = 999999.9 # Left
    self_RA.leftfront_dist = 999999.9 # Left-front
    self_RA.front_dist = 999999.9 # Front
    self_RA.rightfront_dist = 999999.9 # Right-front
    self_RA.right_dist = 999999.9 # Right
 
    ################### ROBOT CONTROL PARAMETERS ##################
    # Maximum forward speed of the robot in meters per second
    # Any faster than this and the robot risks falling over.
    self_RA.forward_speed = 0.05
 
    # Current position and orientation of the robot in the global 
    # reference frame
    self_RA.current_x = 0.0
    self_RA.current_y = 0.0
    self_RA.current_yaw = 0.0
    self_RA.entreCuarto = 0
    self_RA.mover_robot = 0
 
    ############# WALL FOLLOWING PARAMETERS #######################     
    # Finite states for the wall following mode
    #  "turn left": Robot turns towards the left
    #  "search for wall": Robot tries to locate the wall        
    #  "follow wall": Robot moves parallel to the wall
    self_RA.wall_following_state = "turn left"
         
    # Set turning speeds (to the left) in rad/s 
    # These values were determined by trial and error.
    self_RA.turning_speed_wf_fast = 3.0  # Fast turn
    self_RA.turning_speed_wf_slow = 0.05 # Slow turn
 
    # Wall following distance threshold.
    # We want to try to keep within this distance from the wall.
    self_RA.dist_thresh_wf = 0.50 # in meters  
 
    # We don't want to get too close to the wall though.
    self_RA.dist_too_close_to_wall = 0.19 # in meters
 
  def state_estimate_callback(self_RA, msg):
    """
    Extract the position and orientation data. 
    This callback is called each time
    a new message is received on the '/demo/state_est' topic
    """
    # Update the current estimated state in the global reference frame
    curr_state = msg.data
    self_RA.current_x = curr_state[0]
    self_RA.current_y = curr_state[1]
    self_RA.current_yaw = curr_state[2]     
    
    self_RA.follow_wall()      
    
 
  def scan_callback(self_RA, msg):
    """
    This method gets called every time a LaserScan message is 
    received on the '/demo/laser/out' topic 
    """
    # Read the laser scan data that indicates distances
    # to obstacles (e.g. wall) in meters and extract
    # 5 distinct laser readings to work with.
    # Each reading is separated by 45 degrees.
    # Assumes 181 laser readings, separated by 1 degree. 
    # (e.g. -90 degrees to 90 degrees....0 to 180 degrees)
 
    #number_of_laser_beams = str(len(msg.ranges))       
    self_RA.left_dist = msg.ranges[180]
    self_RA.leftfront_dist = msg.ranges[135]
    self_RA.front_dist = msg.ranges[90]
    self_RA.rightfront_dist = msg.ranges[45]
    self_RA.right_dist = msg.ranges[0]
             
  def follow_wall(self_RA):
    """
    This method causes the robot to follow the boundary of a wall.
    """
    # Create a geometry_msgs/Twist message
    msg = Twist()
    
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0
    #cuando entra al cuarto empieza con la logica de recorrer el cuarto 
    

    if self_RA.current_x > 3:
      # Logic for following the wall
      # >d means no wall detected by that laser beam
      # <d means an wall was detected by that laser beam
      d = self_RA.dist_thresh_wf
      
      if self_RA.leftfront_dist > d and self_RA.front_dist > d and self_RA.rightfront_dist > d:
        self_RA.wall_following_state = "search for wall"
        msg.linear.x = self_RA.forward_speed  
        if self_RA.entreCuarto == 0:         
          msg.angular.z = -self_RA.turning_speed_wf_slow      

      elif self_RA.leftfront_dist > d and self_RA.front_dist < d and self_RA.rightfront_dist > d:
            self_RA.wall_following_state = "turn left"
            msg.angular.z = self_RA.turning_speed_wf_fast

      elif (self_RA.leftfront_dist > d and self_RA.front_dist > d and self_RA.rightfront_dist < d):
        if (self_RA.rightfront_dist < self_RA.dist_too_close_to_wall):
          # Getting too close to the wall
          self_RA.wall_following_state = "turn left"
          msg.linear.x = self_RA.forward_speed
          msg.angular.z = self_RA.turning_speed_wf_fast      
        else:             
          # Go straight ahead
          self_RA.wall_following_state = "follow wall" 
          msg.linear.x = self_RA.forward_speed   
  
      elif self_RA.leftfront_dist < d and self_RA.front_dist > d and self_RA.rightfront_dist > d:
        self_RA.wall_following_state = "search for wall"
        msg.linear.x = self_RA.forward_speed
        msg.angular.z = -self_RA.turning_speed_wf_slow # turn right to find wall
  
      elif self_RA.leftfront_dist > d and self_RA.front_dist < d and self_RA.rightfront_dist < d:
        self_RA.wall_following_state = "turn left"
        msg.angular.z = self_RA.turning_speed_wf_fast
  
      elif self_RA.leftfront_dist < d and self_RA.front_dist < d and self_RA.rightfront_dist > d:
        self_RA.wall_following_state = "turn left"
        msg.angular.z = self_RA.turning_speed_wf_fast
  
      elif self_RA.leftfront_dist < d and self_RA.front_dist < d and self_RA.rightfront_dist < d:
        self_RA.wall_following_state = "turn left"
        msg.angular.z = self_RA.turning_speed_wf_fast
              
      elif self_RA.leftfront_dist < d and self_RA.front_dist > d and self_RA.rightfront_dist < d:
        self_RA.wall_following_state = "search for wall"
        msg.linear.x = self_RA.forward_speed
        msg.angular.z = -self_RA.turning_speed_wf_slow # turn right to find wall

      else:
        pass
    else:
      self_RA.entreCuarto = 0
      if self_RA.mover_robot < 20:
        self_RA.wall_following_state = "turn left"
        msg.angular.z = -self_RA.turning_speed_wf_fast
        self_RA.mover_robot = self_RA.mover_robot +1
      
      else:
        msg.linear.x = self_RA.forward_speed
    # Send velocity command to the robot
    self_RA.publisher_.publish(msg)    
 
def main(args=None):
 
    # Initialize rclpy library
    rclpy.init(args=args)
     
    # Create the node
    controller = Controller()
 
    # Spin the node so the callback function is called
    # Pull messages from any topics this node is subscribed to
    # Publish any pending messages to the topics
    rclpy.spin(controller)
 
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
     
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
