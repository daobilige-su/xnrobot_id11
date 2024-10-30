#! /usr/bin/env python
import numpy as np
import math
import rospy
# from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import move_base_msgs.msg
import xnrobot_id01.msg
from geometry_msgs.msg import PoseStamped

from transform_tools import *

simple_move_base_client = actionlib.SimpleActionClient('simple_move_base', move_base_msgs.msg.MoveBaseAction)

start_pos = [-5.77, -6.0, 0.0]

spray_in_baselink_trans_ypr = np.array([[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]])
spray_in_baselink_M = transform_matrix_from_trans_ypr(spray_in_baselink_trans_ypr)

# all_tasks is in spray frame, to control robot, these x,y,theta needs to be converted into base_link frame
all_tasks = np.array([[-5.5, -2.8, 0.0*(math.pi/180),   -1, -1], # before start the 1st row, -1 means no wood to target
                      [-4.0, -2.8, 0.0*(math.pi/180),   27, 36], # x, y, theta, left wood id, right wood id
                      [-3.0, -2.8, 0.0*(math.pi/180),   26, 35],
                      [-2.0, -2.8, 0.0*(math.pi/180),   25, 34],
                      [-1.0, -2.8, 0.0*(math.pi/180),   24, 33],
                      [-0.0, -2.8, 0.0*(math.pi/180),   23, 32],
                      [1.0,  -2.8, 0.0*(math.pi/180),   22, 31],
                      [2.0,  -2.8, 0.0*(math.pi/180),   21, 30],
                      [3.0,  -2.8, 0.0*(math.pi/180),   20, 29],
                      [4.0,  -2.8, 0.0*(math.pi/180),   19, 28],
                      [5.5,  -2.8, 0.0*(math.pi/180),   -1, -1], # transition
                      [5.5,  2.8,  180.0*(math.pi/180), -1, -1], # transition
                      [4.0,  2.8,  180.0*(math.pi/180), 10, 1], # next rows
                      [3.0,  2.8,  180.0*(math.pi/180), 11, 2],
                      [2.0,  2.8,  180.0*(math.pi/180), 12, 3],
                      [1.0,  2.8,  180.0*(math.pi/180), 13, 4],
                      [0.0,  2.8,  180.0*(math.pi/180), 14, 5],
                      [-1.0, 2.8,  180.0*(math.pi/180), 15, 6],
                      [-2.0, 2.8,  180.0*(math.pi/180), 16, 7],
                      [-3.0, 2.8,  180.0*(math.pi/180), 17, 8],
                      [-4.0, 2.8,  180.0*(math.pi/180), 18, 9],
                      [-5.5, 2.8,  180.0*(math.pi/180), -1, -1], # transition
                      [start_pos[0], start_pos[1],  start_pos[2]*(math.pi/180), -1, -1]]) # come back to the start point

nyjqr2021_spray_client = actionlib.SimpleActionClient('nyjqr2021_spray', xnrobot_id01.msg.nyjqr2021_spray_actionAction)

def send_goal_pose(pose_msg):
    goal = move_base_msgs.msg.MoveBaseGoal()
    goal.target_pose = pose_msg

    simple_move_base_client.send_goal(goal)
    rospy.loginfo('simple_move_base_client: sent new goal (%f, %f, %f, %f, %f, %f, %f)' % (
            goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z,
            goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w))

    simple_move_base_client.wait_for_result()
    rospy.loginfo("simple_move_base_client: goal completed")


def send_spray_action(wood_list):
    wood_list = [int(wood_list[0]), int(wood_list[1])]

    goal = xnrobot_id01.msg.nyjqr2021_spray_actionGoal()
    goal.target_wood = wood_list
    nyjqr2021_spray_client.send_goal(goal)

    nyjqr2021_spray_client.wait_for_result()
    rospy.loginfo("nyjqr2021_spray_client: goal completed")


def nyjqr2021_task_client():
    # Waits until the action server has started up and started
    # listening for goals.
    simple_move_base_client.wait_for_server()
    rospy.loginfo('simple_move_base_server connected.')

    nyjqr2021_spray_client.wait_for_server()
    rospy.loginfo('nyjqr2021_spray_server connected.')

    rospy.loginfo('Going to Start after 5s ...')
    rospy.sleep(5)
    rospy.loginfo('Started!')

    goal_pose = PoseStamped()
    for i in range(all_tasks.shape[0]):
        task_pose_in_spray_frame = all_tasks[i,0:3]
        task_pose_in_spray_frame_M = transform_matrix_from_trans_ypr(np.array([[task_pose_in_spray_frame[0]],[task_pose_in_spray_frame[1]],[0],
                                                                [task_pose_in_spray_frame[2]],[0],[0]]))
        task_pose_M = task_pose_in_spray_frame_M @ np.linalg.pinv(spray_in_baselink_M)
        task_pose_trans_ypr = transform_matrix_to_pose_trans_ypr(task_pose_M)
        task_pose = [task_pose_trans_ypr[0,0], task_pose_trans_ypr[1,0], task_pose_trans_ypr[3,0]]

        task_spray = all_tasks[i, 3:5]
        goal_pose.pose.position.x = task_pose[0]
        goal_pose.pose.position.y = task_pose[1]
        quat = ypr2quat(np.array([[task_pose[2]],[0.0],[0.0]]))
        goal_pose.pose.orientation.x = quat[0, 0]
        goal_pose.pose.orientation.y = quat[1, 0]
        goal_pose.pose.orientation.z = quat[2, 0]
        goal_pose.pose.orientation.w = quat[3, 0]

        send_goal_pose(goal_pose)

        send_spray_action(task_spray)


    rospy.loginfo('All tasks should have been done. Good luck.')
    rospy.loginfo('Bye.')
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('nyjqr2021_task_client')
        result = nyjqr2021_task_client()
        # print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion")