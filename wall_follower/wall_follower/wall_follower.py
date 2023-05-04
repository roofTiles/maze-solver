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
        self.zero_range = np.Inf # range of 0-degree LIDAR ray
        self.twoseventy_range = np.Inf # range of 270-degree LIDAR ray
        self.ninty_range = 0.0 # range of 90-degree LIDAR ray
        self.angle_increment = 1.0
        self.desired_wall_distance = 0.35
        self.wall_distance = 0.0 # 0-degree range before rotate so parallel state
        self.state = 'follow wall' # current state of robot

        self.goal_angle = 90 # goal angle for minimum distance from wall

        self.index_min_dist = 0.0
        self.angle_min_dist = 0.0

        self.turn = 0.0 # angle has turned so far (approx)
        self.drive = 0.0 # distance have traveled so far (approx)
        
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',  ## Read
            self.laser_listener_callback,
            qos_profile,
        )

        self.timer_period = 0.05 # update time in seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

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
    
    def indexToAngle(self, index):
        angle = ((index - self.zero)) *  self.angle_increment * 180/pi

        if angle < 0:
            angle = (angle + 360) % 360

        return angle
    
    
    def timer_callback(self):

        if (self.state == 'dead end'): # in a complete dead end where it is entirely surrounded with the walls very close
            angular_vel = 0.1
            if self.turn < 5*np.pi/6:  # rotate approximately pi radians
                self.msg.angular.z = angular_vel
                self.msg.linear.x = 0.0
                self.turn = self.turn + angular_vel*self.timer_period
            
            else:
                self.msg.angular.z = 0.0
                self.msg.linear.x = 0.0
                self.turn = 0.0
                self.drive = 0.0
                self.state = 'follow wall'


        if (self.state == 'outer corner'): # situation where turning left (we are a leftwall follower)
            angular_vel = 0.1
            linear_vel = 0.1

            if self.drive < 0.15: # drive approx 15 cm forward so it can make turn w/o bumping to wall
                self.msg.linear.x = linear_vel
                self.drive = self.drive + linear_vel*self.timer_period

            elif self.turn < np.pi/3:  # rotate approximately pi/3 radians
                self.msg.angular.z = angular_vel
                self.msg.linear.x = 0.0
                self.turn = self.turn + angular_vel*self.timer_period
            
            else:

                # go straight until near wall or about to crash into one >:)
                if self.zero_range < 0.2 or self.ninty_range < (self.desired_wall_distance + 0.2):

                    self.msg.angular.z = 0.0
                    self.msg.linear.x = 0.0
                    self.turn = 0.0
                    self.drive = 0.0
                    self.state = 'follow wall'

                else:
                    self.msg.linear.x = 0.15
                    self.msg.angular.z = 0.0


        if (self.state == 'inner corner'): # situtation where turning right (leftwall follower)
            angular_vel = -0.1 # rotate approximately pi/3 radians
            if self.turn > -np.pi/3:
                self.msg.angular.z = angular_vel
                self.msg.linear.x = 0.0
                self.turn = self.turn + angular_vel*self.timer_period
            
            else:
                self.msg.angular.z = 0.0
                self.msg.linear.x = 0.0
                self.turn = 0.0
                self.state = 'follow wall'

        if (self.state == 'follow wall'):

            self.drive_linvel = 0.15

            if self.zero_range < 0.35 and self.ninty_range < 0.35 and self.twoseventy_range < 0.35: # if surrounded prob a dead end
                self.state = 'dead end'
                self.msg.linear.x = 0.0
                self.msg.angular.z = 0.0

            elif self.zero_range < 0.35 and self.ninty_range < (self.desired_wall_distance + 0.2): # if front is closed off, inner corner
                self.state = 'inner corner'
                self.msg.linear.x  = 0.0
                self.msg.angular.z = 0.0

            elif self.ninty_range > (self.desired_wall_distance + 0.5): # if left wall disappears, outer corner
                self.state = 'outer corner'
                self.msg.linear.x  = 0.0
                self.msg.angular.z = 0.0


            #if the min dist angle << 90, the robot is facing towards the wall, should turn away
            elif (self.angle_min_dist < 85):
                self.msg.angular.z = -0.01*(90 - self.angle_min_dist)

                if (self.msg.angular.z < -0.5):
                    self.msg.angular.z = -0.5 # prevents from bot from spinning too fast

            #if the min dist angel >> 90, the robot is facing away from the wall or is too far, should turn closer
            elif (self.angle_min_dist > 95):
                self.msg.angular.z = 0.01*(self.angle_min_dist - 90)

                if (self.msg.angular.z > 0.5):
                    self.msg.angular.z = 0.5 # prevents from bot from spinning too fast
            
            #if too close or far from the wall orient towards it
            elif (np.abs(self.desired_wall_distance - self.wall_distance) > 0.1):
                self.msg.angular.z = 1 * (self.wall_distance - self.desired_wall_distance)
            
            else:
                self.msg.angular.z = 0.0 # if bot wants to chug forward, dont rotate

            #if the minimum distance angle is ~= 90, the robot is travelling parallel to a wall

            # go close to wall, slow down if getting close
            if self.zero_range > 0.65:
                self.msg.linear.x = self.drive_linvel

            if self.zero_range < 0.65 and self.zero_range > 0.35:
                self.msg.linear.x = self.drive_linvel * (self.zero_range - 0.25) # slows down the bot when close to wall

        elif (self.state == 'find wall'): # go to wall until about 0.35 meters away
            stop_range = self.desired_wall_distance
            if (self.zero_range > stop_range):
                self.msg.linear.x = ((self.zero_range - stop_range) * 0.5) + 0.05 # proportional controller
                self.msg.angular.z = 0.0
            else:
                self.msg.linear.x = 0.0
                self.state = 'follow wall'



        # msg = Twist()
        key = self.getch()
        if key == 'w':
            self.msg.linear.x = 0.2  
        elif key == 'x':
            self.msg.linear.x = -0.2
        elif key == 'd':
            self.msg.angular.z = 0.2
        elif key == 'a':
            self.msg.angular.z = -0.2
        elif key == ' ':
            self.msg.linear.x = 0.0
            self.msg.angular.z = 0.0
            self.state = ''
        
        
        
        self.publisher_.publish(self.msg)

    def laser_listener_callback(self, msg):

        self.angle_increment = msg.angle_increment
        self.zero = int((0.0 - msg.angle_min) / self.angle_increment) # finds index of 0 degree LIDAR ray

        msg.ranges = [msg.range_max if i > msg.range_max else i for i in msg.ranges] # makes sure no LIDAR ray is above range max

        # averages values +- 5 degrees around target LIDAR
        self.zero_range = np.mean([msg.ranges[self.angle(i)] for i in list(range(0, 6)) + list(range(355, 361))])
        self.ninty_range = np.mean([msg.ranges[self.angle(i)] for i in range(85, 96)])
        self.twoseventy_range = np.mean([msg.ranges[self.angle(i)] for i in range(260, 280)])

        # find the angle of the closest distance to the wall.  
        # if parallel is 90.  if > 90, should rotate CW.  if < 90, should rotate CCW
        #left_ranges = msg.ranges[self.angle(45) : self.angle(135)]
        self.wall_distance = min(msg.ranges)

        self.index_min_dist = msg.ranges.index(self.wall_distance)
        self.angle_min_dist = self.indexToAngle(self.index_min_dist)

        #debugging outputs
        self.get_logger().info(str(self.twoseventy_range) + '\t' + str(self.zero_range) + '\t' + str(self.ninty_range) + '\t' + str(self.state))



def main(args=None):
    rclpy.init(args=args)
    my_follower = Follow()
    rclpy.spin(my_follower)
    my_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()