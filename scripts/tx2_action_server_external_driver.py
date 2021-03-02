#! /usr/bin/env python

import roslib
import rospy
import actionlib
import tf
import math
import time

from geometry_msgs.msg import PoseStamped, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from copy import copy

current_pose = []
path = []

DEFAULT_TOLERANCE = 1
DEFAULT_RATE = 10
DEFAULT_TIMEOUT = 30
DEFAULT_N_FAILS = 5


class MoveRobot:
    def __init__(self):
        
        rospy.init_node('tx2_action_server')
        tolerance = rospy.get_param('~tolerance', DEFAULT_TOLERANCE)
        rate = rospy.get_param('~rate', DEFAULT_RATE)
        timeout = rospy.get_param('~timeout', DEFAULT_TIMEOUT)
        max_path_fails = rospy.get_param('~max_path_fails', DEFAULT_N_FAILS)
        print('TOLERANCE IS', tolerance)
        self.server = actionlib.SimpleActionServer( 'move_base', MoveBaseAction, self.execute, False )
        self.server.start()
        self.goal_publisher = rospy.Publisher('/exploration_goal', PoseStamped, queue_size=5)
        self.task_publisher = rospy.Publisher('/task', Float32MultiArray, queue_size=5)
        self.tolerance = tolerance
        self.rate = rospy.Rate(rate)
        self.timeout = timeout
        self.max_path_fails = max_path_fails
        self.current_pose = None
        self.path = []
        self.path_received = False
        self.tf_listener = tf.TransformListener()
        self.odom_subscriber = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.path_subscriber = rospy.Subscriber('path', Path, self.path_callback)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def path_callback(self, msg):
        self.path_received = True
        self.path = msg.poses

    def get_robot_pose(self):
        try:
            pos, quat = self.tf_listener.lookupTransform(
                                'map', 'odom',
                                self.tf_listener.getLatestCommonTime('map',
                                'odom'))
        except:
            print('NO TRANSFORM FROM ODOM TO MAP!!!!')
            return None, None
        if self.current_pose is None:
            print('NO ODOMETRY!!!')
            return None, None
        _, __, angle = tf.transformations.euler_from_quaternion(quat)
        current_x, current_y = self.current_pose.position.x, self.current_pose.position.y
        current_x_new = current_x * math.cos(-angle) + current_y * math.sin(-angle)
        current_y_new = -current_x * math.sin(-angle) + current_y * math.cos(-angle)
        current_x_new += pos[0]
        current_y_new += pos[1]
        return current_x_new, current_y_new

    def wait_until_come(self, target_x, target_y):
        start_time = time.time()
        self.path_received = False
        n_path_fails = 0
        succeeded = False
        while time.time() - start_time < self.timeout:
            current_x_new, current_y_new = self.get_robot_pose()
            self.publish_pathplanning_task(current_x_new, current_y_new, target_x, target_y)
            dst_to_goal = math.sqrt((current_x_new -  target_x) ** 2 + (current_y_new - target_y) ** 2)
            # if we reach the goal, finish with success
            if dst_to_goal < self.tolerance:
                print('Goal reached!')
                succeeded = True
                break
            self.rate.sleep()
            # if path to goal not found, finish without success
            if self.path_received and len(self.path) == 0:
                n_path_fails += 1
            if n_path_fails >= self.max_path_fails:
                print('Goal unavailable: path not found!')
                break
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
