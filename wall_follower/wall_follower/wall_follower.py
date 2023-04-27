"""
    Minimal code for wall follower 
"""

from math import pi
import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan


class Follow(Node):
    def __init__(self):
        super().__init__('follow')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        self.zero = 0.0 # index of 0-degree LIDAR ray
        self.ninty_range = 0.0 # range of 90-degree LIDAR ray
        self.zero_range = np.Inf # range of 0-degree LIDAR ray
        self.prev_ninty_ranges = [0]*14 # previous 90-degree ranges
        self.angle_increment = 1.0
        self.range_max  = 0.0
        self.wall_distance = 0.0 # 0-degree range before rotate so parallel state
        self.state = 'find wall' # current state of robot


        self.i = 0

        #sams vars
        self.index_min_dist = 0.0
        self.angle_min_dist = 0.0
        
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',  ## Read
            self.laser_listener_callback,
            qos_profile,
        )

        self.timer_period = 0.01 # update time in seconds
        #self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.timer = self.create_timer(self.timer_period, self.sams_timer_callback)

        self.msg = Twist()
        
    def getch(self, timeout=0.1):
        import sys, tty, termios, select
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                ch = sys.stdin.read(1)
            else:
                ch = None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def angle(self, angle_):  # returns the index of a given angle
        ninty = self.zero + int((pi*angle_/180)/ self.angle_increment)
        return int(ninty)
    
    def sams_timer_callback(self):
        pass
        
        #msg = Twist()        

        #PARAMETERS TO CHANGE
        self.cornering_linvel = 0.05
        self.cornering_angvel = 0.3

        self.drive_linvel = 0.1

        self.wall_dist_hi = 0.4
        self.wall_dist_lo = 0.2
        #self.drive_angvel (unused) - controller in the code below

        #if the min dist angle << 90, the robot is facing towards the wall, should turn away
        if (self.angle_min_dist < 80):
            self.msg.linear.x = self.cornering_linvel
            self.msg.angular.z = -1*self.cornering_angvel
        #if the min dist angel >> 90, the robot is facing away from the wall or is too far, should turn closer
        elif (self.angle_min_dist > 100):
            self.msg.linear.x =self.cornering_linvel
            self.msg.angular.z = self.cornering_angvel
        #if the minimum distance angle is ~= 90, the robot is travelling parallel to a wall
        else:
            self.msg.linear.x = self.drive_linvel
            self.msg.angular.z = 0.0
            
        
        #tests if too far away from wall, steers it towards it
        if (self.wall_distance > self.wall_dist_hi):
            self.msg.angular.z = self.msg.angular.z + self.wall_distance/4
        #tests if too close to wall, steers it away
        elif (self.wall_distance < self.wall_dist_lo):
            self.msg.angular.z = self.msg.angular.z - 0.1
        
        
        key = self.getch()
        if key == 'w':
            self.msg.linear.x = 1.0  
        elif key == 'x':
            self.msg.linear.x = -1.0  
        elif key == ' ':
            self.msg.linear.x = 0.0
            self.msg.angular.z = 0.0
        
        self.get_logger().info("xvel:\t" + str(self.msg.linear.x) + "\tzvel:\t" + str(self.msg.angular.z))

        self.publisher_.publish(self.msg)


    def timer_callback(self):

        if (self.state == 'follow wall'): # every 10 samples do correction.

            if (self.i != 5):
                self.msg.linear.x = 0.1 # an arbitrary speed
                self.i = self.i + 1
            
            elif (self.i == 5):
                self.msg.linear.x = 0.0
                self.i = 0
                self.state = 'rotate so parallel'

        elif (self.state == 'rotate so parallel'): # reorients bot so it is parallel to wall

            if self.wall_distance != 0.0: # tries to maintain the 90-degree LIDAR range to be constant

                if np.abs(self.ninty_range - self.wall_distance) > self.wall_distance*0.05: # reorient if ninty degree LIDAR range has changes
                    self.msg.angular.z = (self.ninty_range - self.wall_distance) * 0.5 
                
                elif np.abs(self.ninty_range - self.wall_distance) < self.wall_distance*0.05:
                    self.msg.angular.z = 0.0
                    self.wall_distance = self.ninty_range
                    self.state = 'follow wall'
                
                self.get_logger().info("%r, %f, %f" % (self.state, self.wall_distance, self.ninty_range))

            elif self.wall_distance == 0.0: # if there is no known wall distance to reorient

                first_half_median = np.median(self.prev_ninty_ranges[:int(len(self.prev_ninty_ranges)/2)])
                second_half_median = np.median(self.prev_ninty_ranges[int(len(self.prev_ninty_ranges)/2):])   

                if self.range_max == second_half_median: # if wall hasn't show up yet on 90-degree LIDAR rotate slowly
                    self.msg.angular.z = -0.05

                elif np.abs(first_half_median - second_half_median) > 0.02*second_half_median:

                    # if difference between first half previous ranges and second half
                    # is more than 2% do a proportional controller to reorient itself
                    # finds minimum 90 degree range this way

                    self.msg.angular.z = (first_half_median - second_half_median) * -0.2


                elif np.abs(first_half_median - second_half_median) < 0.02*second_half_median:

                    # when the 90 degree LIDAR stabilizes exit out of rotate so parallel 

                    self.msg.angular.z = 0.0
                    self.wall_distance = self.ninty_range
                    self.state = 'follow wall'
            
            #self.get_logger().info("%r, %f, %f, %f" % (self.state, self.zero_range, first_half_median, second_half_median))
            
        elif (self.state == 'find wall'): # go to wall until about 0.35 meters away
            if (self.zero_range > 0.50):
                self.msg.linear.x = (self.zero_range - 0.50) * 0.2 # proportional controller
                self.msg.angular.z = 0.0
            else:
                self.msg.linear.x = 0.0
                self.state = 'rotate so parallel'
                #self.msg.angular.z = 1.7



        # msg = Twist()
        key = self.getch()
        if key == 'w':
            self.msg.linear.x = 0.1  
        elif key == 'x':
            self.msg.linear.x = -0.1
        elif key == 'd':
            self.msg.angular.z = 0.1
        elif key == 'a':
            self.msg.angular.z = -0.1
        elif key == ' ':
            self.msg.linear.x = 0.0
            self.msg.angular.z = 0.0
            self.state = ''
        
        
        
        self.publisher_.publish(self.msg)

    def laser_listener_callback(self, msg):

        self.angle_increment = msg.angle_increment
        self.range_max = msg.range_max
        self.zero = int((0.0 - msg.angle_min) / self.angle_increment) # finds index of 0 degree LIDAR ray

        self.zero_range = msg.ranges[self.zero]
        self.prev_ninty_ranges = self.prev_ninty_ranges[1:] + [self.ninty_range] # update array of previous 90 degree LIDAR ranges

        self.ninty_range = msg.ranges[self.angle(90.0)]

        # find the angle of the closest distance to the wall.  
        # if parallel is 90.  if > 90, should rotate CW.  if < 90, should rotate CCW

        self.wall_distance = min(msg.ranges)
        self.index_min_dist = msg.ranges.index(self.wall_distance)
        self.angle_min_dist = self.zero + self.index_min_dist

        if self.ninty_range == np.Inf: # saturate current 90 degree LIDAR range to avoid uncontrollable behavior
            self.ninty_range = msg.range_max

        #debugging outputs
        #self.get_logger().info(str(self.zero_range) + '\t' + str(self.ninty_range))
        #self.get_logger().info(str(self.angle_min_dist) + '\t' + str(self.index_min_dist) + '\t' + str(msg.ranges[self.index_min_dist]) + '\t' + str(self.wall_distance))
        self.get_logger().info(str(self.angle_min_dist) + '\t' + str(self.index_min_dist) + '\t' + str(self.wall_distance))



def main(args=None):
    rclpy.init(args=args)
    my_follower = Follow()
    rclpy.spin(my_follower)
    my_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()