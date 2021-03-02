#! /usr/bin/env python

import roslib
import rospy
import actionlib
import tf
import math
import time

from geometry_msgs.msg import PoseStamped, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from copy import copy

global trajectory
trajectory = []

DEFAULT_TOLERANCE = 1
DEFAULT_RATE = 10
DEFAULT_TIMEOUT = 30

def callback(msg):
    trajectory.append(msg.pose.pose)

class MoveRobot:
    def __init__(self):
        
        rospy.init_node('tx2_action_server')
        tolerance = rospy.get_param('~tolerance', DEFAULT_TOLERANCE)
        rate = rospy.get_param('~rate', DEFAULT_RATE)
        timeout = rospy.get_param('~timeout', DEFAULT_TIMEOUT)
        print('TOLERANCE IS', tolerance)
        self.server = actionlib.SimpleActionServer( 'move_base', MoveBaseAction, self.execute, False )
        self.server.start()
        self.goal_publisher = rospy.Publisher('/exploration_goal', PoseStamped, queue_size=5)
        self.task_publisher = rospy.Publisher('/task', Float32MultiArray, queue_size=5)
        self.Subscriber = rospy.Subscriber('odom', Odometry, callback)
        self.tf_listener = tf.TransformListener()
        self.tolerance = tolerance
        self.rate = rospy.Rate(rate)
        self.timeout = timeout

    def get_robot_pose(self):
        try:
            pos, quat = self.tf_listener.lookupTransform(
                                'map', 'odom',
                                self.tf_listener.getLatestCommonTime('map',
                                'odom'))
        except:
            print('NO TRANSFORM FROM ODOM TO MAP!!!!')
            return None, None
        _, __, angle = tf.transformations.euler_from_quaternion(quat)
        current_pose = trajectory[-1]
        current_x, current_y = current_pose.position.x, current_pose.position.y
        current_x_new = current_x * math.cos(-angle) + current_y * math.sin(-angle)
        current_y_new = -current_x * math.sin(-angle) + current_y * math.cos(-angle)
        current_x_new += pos[0]
        current_y_new += pos[1]
        return current_x_new, current_y_new

    def wait_until_come(self, target_x, target_y):
        start_time = time.time()
        succeeded = False
        while not succeeded and time.time() - start_time < self.timeout:
            current_x_new, current_y_new = self.get_robot_pose()
            self.publish_pathplanning_task(current_x_new, current_y_new, target_x, target_y)
            dst_to_goal = math.sqrt((current_x_new -  target_x) ** 2 + (current_y_new - target_y) ** 2)
            if dst_to_goal < self.tolerance:
                print('Goal reached!')
                succeeded = True
            self.rate.sleep()
        return succeeded

    def publish_pathplanning_task(self, start_x, start_y, goal_x, goal_y):
        task = Float32MultiArray()
        task.layout.data_offset = 0;
        task.layout.dim.append(MultiArrayDimension())
        task.layout.dim[0].label = "width"
        task.layout.dim[0].size  = 4
        task.layout.dim[0].stride  = 4
        task.data = [start_x, start_y, goal_x, goal_y]
        self.task_publisher.publish(task)

    def execute( self, goal ):
        pose = goal.target_pose
        target_x, target_y = pose.pose.position.x, pose.pose.position.y
        print( "Recieved goal: {}, {}".format(target_x, target_y))
        self.goal_publisher.publish(goal.target_pose)
        goal_reached = self.wait_until_come(target_x, target_y)
        if goal_reached:
            self.server.set_succeeded()
        else:
            self.server.set_aborted()

if __name__ == '__main__':
    server = MoveRobot()
    rospy.spin()
