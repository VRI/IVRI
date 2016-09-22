#!/usr/bin/env python

"""Competition_Code, 9/15/16, Sajad Azami, Taher Ahmadi"""
__author__ = 'sajjadaazami@gmail.com (Sajad Azami), 14taher@gmail.com (Taher Ahmadi)'

import random
import sys
import threading
import time

import actionlib
import rospy
import smach
import smach_ros
import tf
from actionlib_msgs.msg import GoalStatusArray
from behaviour_smach.msg import *
from move_base_msgs.msg import *
from nav_msgs.msg import OccupancyGrid

# Variables Declaration
goals_list = []  # goals given to robot will be appended to this
current_goal_status = 0  # goal status
status_dict = {'PENDING': 0, 'ACTIVE': 1, 'PREEMPTED': 2, 'SUCCEEDED': 3, 'ABORTED': 4, 'REJECTED': 5, 'PREEMPTING': 6,
               'RECALLING': 7, 'RECALLED': 8, 'LOST': 9}
global_costmap = 0  # 2d array of costmap
robot_namespace = ''
current_direction = 0  # current direction of robot explore(0-4)


# Functions Declaration

# get current position of robot using tf translation
def get_current_position():
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    flag = True
    trans = 0
    while flag and not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform((robot_namespace + '/map'), (robot_namespace + '/base_link'),
                                                    rospy.Time(0))
            rospy.loginfo(trans)
            flag = False
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    return trans


# subscriber method callback from /move_base/status
def callback_direction_status(data):
    global current_direction
    current_direction = data.direction
    return current_direction


# subscriber method from /move_base/status
def listener_direction_status():
    rospy.Subscriber((robot_namespace + "/direction_status"), DirectionStatus, callback_direction_status)
    return


# get current position of robot using tf translation
def get_current_direction():
    listener_direction_status()


# random goal generator(Use NE, NW, SW and SW for directions)
def get_random_goal(exp_type):
    if exp_type == 'NW':  # NW
        x = random.uniform(-50.0, 0)
        y = random.uniform(0, 50.0)
        w = 1
        z = 1
    elif exp_type == 'NE':  # NE
        x = random.uniform(0.0, 50.0)
        y = random.uniform(0.0, 50.0)
        w = 1
        z = 1
    elif exp_type == 'SW':  # SW
        x = random.uniform(-50.0, 0)
        y = random.uniform(-50.0, 0)
        w = 1
        z = 1
    elif exp_type == 'SE':  # SE
        x = random.uniform(0, 50.0)
        y = random.uniform(-50.0, 0)
        w = 1
        z = 1
    else:
        x = random.uniform(-50.0, 50.0)
        y = random.uniform(-50.0, 50.0)
        w = 1
        z = 1
    return [x, y, 0, w, 0, 0, z]


# subscriber method callback from /move_base/status
def callback_goal_status(data):
    global current_goal_status
    current_goal_status = data.status_list[len(data.status_list) - 1].status


# subscriber method from /move_base/status
def get_current_goal_status():
    rospy.Subscriber((robot_namespace + "/move_base/status"), GoalStatusArray, callback_goal_status)
    return current_goal_status


# subscriber method callback from /move_base/global_costmap/costmap
def callback_global_costmap(data):
    global global_costmap
    global_costmap = data.data


# subscriber method from /move_base/global_costmap/costmap
def listener_global_costmap():
    rospy.Subscriber((robot_namespace + "/move_base/global_costmap/costmap"), OccupancyGrid, callback_global_costmap)


# publishes goal on move_base/goal using SimpleActionClient
# inputs: position x, y, z, orientation w, x, y, z
def move_to(pos_x, pos_y, pos_z, ornt_w, ornt_x, ornt_y, ornt_z):
    # Simple Action Client
    sac = actionlib.SimpleActionClient((robot_namespace + '/move_base'), MoveBaseAction)

    # create goal
    goal = MoveBaseGoal()

    # set goal
    goal.target_pose.pose.position.x = pos_x
    goal.target_pose.pose.position.y = pos_y
    goal.target_pose.pose.orientation.w = ornt_w
    goal.target_pose.pose.orientation.z = ornt_z
    goal.target_pose.header.frame_id = (robot_namespace + '/odom')
    goal.target_pose.header.stamp = rospy.Time.now()

    # start listener
    sac.wait_for_server()

    # send goal
    sac.send_goal(goal)

    # finish
    # sac.wait_for_result()


# States Declaration

# define Detect state
class Detect(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goalReached', 'goalCancelled'])
        # self.mutex = threading.Lock()
        # self.found_received = False

    # def callback(self, msg):
    #     self.mutex.acquire()
    #     if msg.found == 1:
    #         self.found_received = True
    #     self.mutex.release()
    #
    def execute(self, userdata):
        rospy.loginfo('Executing WaitForVictim')
        while get_current_goal_status() == 1:
            rospy.loginfo('Detecting')
            rospy.loginfo(get_current_goal_status())
            time.sleep(1)
        return 'goalReached'


# define Explore state
class Explore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goalPublished', 'goalNotPublished'])
        # rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, callback_global_costmap())

    def execute(self, userdata):
        rospy.loginfo('Executing state Explore')
        # TODO add to Documentation : Goal format is goal_list_temp = [x, y, 0, w, 0, 0, x]
        # current_position = get_current_position()  # current translation of robot in an array[][]

        goal_temp = get_random_goal(5)  # get random goal
        goals_list.append(goal_temp)  # add goal to goal list(for further uses)
        # move_to(goal_temp[0] + current_position[0], goal_temp[1] + current_position[1], goal_temp[2],
        #         goal_temp[3], goal_temp[4], goal_temp[5], goal_temp[6], )
        move_to(goal_temp[0], goal_temp[1], goal_temp[2],
                goal_temp[3], goal_temp[4], goal_temp[5], goal_temp[6], )
        if get_current_goal_status() == 1:
            return 'goalPublished'
        else:
            return 'goalNotPublished'


# Main Function

def main():
    rospy.init_node('behaviour')
    sm = smach.StateMachine(
        outcomes=['SHUTDOWN'])
    global robot_namespace
    if len(sys.argv) > 1:
        robot_namespace = sys.argv[1]
    with sm:
        smach.StateMachine.add('EXPLORE', Explore(),
                               transitions={'goalPublished': 'DETECT', 'goalNotPublished': 'EXPLORE'})

        smach.StateMachine.add('DETECT', Detect(),
                               transitions={'goalCancelled': 'EXPLORE', 'goalReached': 'EXPLORE'})

        sis = smach_ros.IntrospectionServer('Behavior', sm, (robot_namespace + '/SM_ROOT'))
        sis.start()

        # Execute SMACH plan
        outcome = sm.execute()

        rospy.spin()
        sis.stop()


if __name__ == '__main__':
    main()
