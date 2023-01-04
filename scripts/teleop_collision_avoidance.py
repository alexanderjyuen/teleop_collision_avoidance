#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TwistStamped, TransformStamped, PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from prometheus_client import start_http_server, Histogram, Gauge
from tf import TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class CollisionChecker:

    def __init__(self):
        rospy.loginfo('teleop_collision_avoidance started')

        #Subscribers
        self._laser_scan_sub: rospy.Subscriber
        self._cmd_vel_sub: rospy.Subscriber
        self._cmd_vel_pub: rospy.Publisher

        self._laser_scan = LaserScan()
        self._cmd_vel = Twist()
        
        self._slow_down_distance = 10
        self._stopping_distance = 5
        self._velocity_scaling_factor = 1.0

        self._setup_sub_pub()
        self._run()

    def _setup_sub_pub(self):
        rospy.loginfo("creating subscribers and publishers")
        self._laser_scan_sub = rospy.Subscriber("/front/scan_os3/filtered", LaserScan, self._laser_scan_callback)
        self._cmd_vel_sub = rospy.Subscriber("/nav_vel_pre", Twist, self._cmd_vel_callback)
        self._cmd_vel_pub = rospy.Publisher('/nav_vel', Twist, queue_size=10)
        return

    def _laser_scan_callback(self, msg):
        self._laser_scan = msg
        min_range = float('inf')
        for range in self._laser_scan.ranges:
        	if range < min_range:
        		min_range = range
        
        distance_to_obstacle = max([0.0, min_range - self._stopping_distance])
        self._velocity_scaling_factor = min([1.0, distance_to_obstacle/(self._slow_down_distance - self._stopping_distance)])
               
        #rospy.loginfo('min_range: %f', min_range)
        #rospy.loginfo('velocity_scaling_factor %f', self._velocity_scaling_factor)
        
        return
        
    def _cmd_vel_callback(self, msg):
        self._cmd_vel = msg
        self._cmd_vel.linear.x = self._cmd_vel.linear.x*self._velocity_scaling_factor
        rospy.loginfo('self._cmd_vel.linear.x %f', self._cmd_vel.linear.x)
        self._cmd_vel_pub.publish(self._cmd_vel)
        return


    def _run(self):
        rate = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('teleop_collision_avoidance_node', anonymous=True)
    rospy.loginfo('started teleop_collision_avoidance_node')
    rospy.loginfo('waiting 1 seconds to begiteleop collision avoidance')
    rospy.sleep(1)
    checker = CollisionChecker()
