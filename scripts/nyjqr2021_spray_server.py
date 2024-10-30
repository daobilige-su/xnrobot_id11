#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import xnrobot_id01.msg
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Int8MultiArray
import numpy as np
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError

from transform_tools import *

spray_in_baselink_trans_ypr = np.array([[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]])
spray_in_baselink_M = transform_matrix_from_trans_ypr(spray_in_baselink_trans_ypr)

spray_pt_left_in_spray_frame = np.array([[0.0], [1.4], [0.0]])
spray_pt_left_in_baselink_frame = spray_in_baselink_M @ np.block([[spray_pt_left_in_spray_frame],[np.ones((1,1))]])
spray_pt_left_in_baselink_frame = spray_pt_left_in_baselink_frame[0:3,:]

spray_pt_right_in_spray_frame = np.array([[0.0], [-1.4], [0.0]])
spray_pt_right_in_baselink_frame = spray_in_baselink_M @ np.block([[spray_pt_right_in_spray_frame],[np.ones((1,1))]])
spray_pt_right_in_baselink_frame = spray_pt_right_in_baselink_frame[0:3,:]

laser_in_baselink_trans_ypr = np.array([[0.1], [0], [0], [0], [0], [0]])
laser_in_baselink_M = transform_matrix_from_trans_ypr(laser_in_baselink_trans_ypr)

laser_dist_max = 3.0
laser_pc_search_thr = 0.3

angle_diff_thr = 3.0*(math.pi/180.0)
cmd_vel_theta_p = 1.0
cmd_vel_theta_max = 0.5

img_crop_h_min = 0
img_crop_h_max = 180
img_crop_w_min = 100
img_crop_w_max = 220

sample_color = [40.0,40.0,70.0]
moving_average_win_len = 20
wood_w_len_half = 5
white_strip_color = [255.0,255.0,255.0]

class nyjqr2021_spray_action(object):
    # create messages that are used to publish feedback/result
    _feedback = xnrobot_id01.msg.nyjqr2021_spray_actionFeedback()
    _result = xnrobot_id01.msg.nyjqr2021_spray_actionResult()

    def __init__(self):
        self._action_name = 'nyjqr2021_spray'
        self._as = actionlib.SimpleActionServer(self._action_name, xnrobot_id01.msg.nyjqr2021_spray_actionAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        self.tf_listener = tf.TransformListener()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd_spray_pub = rospy.Publisher('cmd_spray', Int8MultiArray, queue_size=5)
        self.wood_det_pub = rospy.Publisher('wood_det', Image, queue_size=5)
        # self.right_wood_det_pub = rospy.Publisher('rightcam_wood', Image, queue_size=5)

        self.bridge = CvBridge()

    def execute_direction_adjustment(self, expected_pt):
        expected_pt_in_baselink_anlge = np.arctan2(expected_pt[1,0],expected_pt[0,0])
        cmd_vel_msg = Twist()

        success = False

        while success != True:

            laser_msg = rospy.wait_for_message('/vrep/scan', LaserScan)
            laser_range = np.array(laser_msg.ranges).reshape(-1, 1)
            laser_angle = (laser_msg.angle_min + np.arange(0, laser_msg.angle_increment *laser_range.shape[0],laser_msg.angle_increment)).reshape(-1, 1)
            laser_pc = np.block([[laser_range * np.cos(laser_angle), laser_range * np.sin(laser_angle)]])

            # laser_pc to baselink frame
            laser_pc_filter = laser_in_baselink_M @ np.block([[laser_pc.T], [np.zeros((1, laser_pc.shape[0]))], [np.ones((1, laser_pc.shape[0]))]])
            laser_pc_filter = laser_pc_filter[0:2, :].T

            # take useful pc only
            laser_pc_filter = laser_pc_filter[((laser_range > 0) & (laser_range < laser_dist_max)).reshape(-1, ), :]
            laser_pc_filter = laser_pc_filter[((abs(laser_pc_filter[:,0] - expected_pt[0,0]) < laser_pc_search_thr) & (abs(laser_pc_filter[:,1] - expected_pt[1,0]) < laser_pc_search_thr)).reshape(-1, ), :]

            if laser_pc_filter.shape[0]!=0:
                min_v = np.min(np.sqrt((expected_pt[0,0] - laser_pc_filter[:,0]) ** 2 + (expected_pt[1,0] - laser_pc_filter[:,1]) ** 2))
                min_idx = np.argmin(np.sqrt((expected_pt[0,0] - laser_pc_filter[:,0]) ** 2 + (expected_pt[1,0] - laser_pc_filter[:,1]) ** 2))

                wood_pt = laser_pc_filter[min_idx:min_idx+1,:]
                wood_pt_in_baselink_angle = np.arctan2(wood_pt[0,1],wood_pt[0,0])

                angle_diff = -1.0*wrap_to_pi(expected_pt_in_baselink_anlge-wood_pt_in_baselink_angle)

                if abs(angle_diff) < angle_diff_thr:
                    cmd_vel_theta = 0.0
                    success = True
                else:
                    cmd_vel_theta = angle_diff * cmd_vel_theta_p

                if cmd_vel_theta>cmd_vel_theta_max:
                    cmd_vel_theta = cmd_vel_theta_max
                elif cmd_vel_theta<(-1.0*cmd_vel_theta_max):
                    cmd_vel_theta = -1.0*cmd_vel_theta_max

                cmd_vel_msg.angular.z = cmd_vel_theta

                # publish "cmd_vel"
                rospy.loginfo('expected_pt_in_baselink_anlge: %f, wood_pt_in_baselink_angle: %f, angle_diff: %f, cmd_vel_theta: %f' %
                              (expected_pt_in_baselink_anlge, wood_pt_in_baselink_angle, angle_diff, cmd_vel_theta))
                self.cmd_vel_pub.publish(cmd_vel_msg)

            else:
                rospy.loginfo('no laser pc detected: stopping')
                cmd_vel_theta = 0.0
                success = True

                cmd_vel_msg.angular.z = cmd_vel_theta

                # publish "cmd_vel"
                self.cmd_vel_pub.publish(cmd_vel_msg)

        return 0

    def find_white_strip(self, img_topic_name):
        img_msg = rospy.wait_for_message(img_topic_name, Image)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image = cv2.flip(cv_image, 0)
        cv_image_crop = cv_image[img_crop_h_min:img_crop_h_max,img_crop_w_min:img_crop_w_max]
        # cv_image_crop_gray = cv2.cvtColor(cv_image_crop, cv2.COLOR_BGR2GRAY)

        np_image_crop = np.asarray(cv_image_crop, dtype=np.float64)
        np_image_crop_sample_similiarity = np.abs(np_image_crop[:,:,0] - sample_color[0]) \
                                           + np.abs(np_image_crop[:,:,1] - sample_color[1]) \
                                           + np.abs(np_image_crop[:,:,2] - sample_color[2])

        horizontal_similiarity= np.sum(np_image_crop_sample_similiarity, 0)
        horizontal_similiarity_moving_average = np.convolve(horizontal_similiarity, np.ones(moving_average_win_len) / moving_average_win_len, mode='valid')
        wood_w_idx = np.argmin(horizontal_similiarity_moving_average)+int(moving_average_win_len/2)

        np_image_crop_wood_part = np_image_crop[:,(wood_w_idx-wood_w_len_half):(wood_w_idx+wood_w_len_half),:]
        np_image_crop_wood_part_white_strip_similiarity = np.abs(np_image_crop_wood_part[:, :, 0] - white_strip_color[0]) \
                                                            + np.abs(np_image_crop_wood_part[:, :, 1] - white_strip_color[1]) \
                                                            + np.abs(np_image_crop_wood_part[:, :, 2] - white_strip_color[2])

        vertical_similiarity = np.sum(np_image_crop_wood_part_white_strip_similiarity, 1)
        vertical_similiarity_moving_average = np.convolve(vertical_similiarity, np.ones(moving_average_win_len)/moving_average_win_len, mode='valid')
        wood_h_idx = np.argmin(vertical_similiarity_moving_average) + int(moving_average_win_len / 2)

        wood_w_idx_orig_img = wood_w_idx+img_crop_w_min
        wood_h_idx_orig_img = wood_h_idx+img_crop_h_min
        wood_img_idx = [wood_h_idx_orig_img,wood_w_idx_orig_img]

        cv_image_wood_show = cv_image.copy()
        cv_image_wood_show[wood_h_idx_orig_img,:] = [0,0,255]
        cv_image_wood_show[:, wood_w_idx_orig_img] = [0, 0, 255]
        # cv2.imshow('cv_image_wood_show', cv_image_wood_show)
        # cv2.waitKey(0)

        try:
            ros_image = self.bridge.cv2_to_imgmsg(cv_image_wood_show, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.wood_det_pub.publish(ros_image)
        return wood_img_idx

    def execute_cb(self, goal):
        # helper variables
        # r = rospy.Rate(5)
        success = False

        rospy.loginfo('%s: Executing, obtained goal wood of id: (%f, %f)' % (
            self._action_name, goal.target_wood[0], goal.target_wood[1]))

        target_wood_left = goal.target_wood[0]
        target_wood_right= goal.target_wood[1]

        # start executing the action
        msg_data = Int8MultiArray()

        # left spray
        if target_wood_left>0:
            expected_pt = spray_pt_left_in_baselink_frame
            self.execute_direction_adjustment(expected_pt)

            img_topic_name = 'leftcam'
            wood_img_idx = self.find_white_strip(img_topic_name) # wood_img_idx: [h,w]
            angle_incr = (wood_img_idx[0]-70.0)*(30.0/70.0)

            data = [1, -90-int(angle_incr), 0, 0, 0, 0]
            msg_data.data = data
            self.cmd_spray_pub.publish(msg_data)
            rospy.sleep(1.0)

            data = [1, -90-int(angle_incr), 1, 0, 0, 0]
            msg_data.data = data
            self.cmd_spray_pub.publish(msg_data)
            rospy.sleep(1.0)

            data = [0, 0, 0, 0, 0, 0]
            msg_data.data = data
            self.cmd_spray_pub.publish(msg_data)
            rospy.sleep(1.0)

        if target_wood_right>0:
            expected_pt = spray_pt_right_in_baselink_frame
            self.execute_direction_adjustment(expected_pt)

            img_topic_name = 'rightcam'
            wood_img_idx = self.find_white_strip(img_topic_name)  # wood_img_idx: [h,w]
            angle_incr = (wood_img_idx[0] - 70.0) * (30.0 / 70.0)

            # right spray
            data = [0, 0, 0, 1, -90-int(angle_incr), 0]
            msg_data.data = data
            self.cmd_spray_pub.publish(msg_data)
            rospy.sleep(1.0)

            data = [0, 0, 0, 1, -90-int(angle_incr), 1]
            msg_data.data = data
            self.cmd_spray_pub.publish(msg_data)
            rospy.sleep(1.0)

            data = [0, 0, 0, 0, 0, 0]
            msg_data.data = data
            self.cmd_spray_pub.publish(msg_data)
            rospy.sleep(1.0)

        success = True


        if success:
            # self._result.sequence = self._feedback.sequence
            #self._result.dummy_result.append(0)

            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('nyjqr2021_spray_server')
    server = nyjqr2021_spray_action()
    rospy.spin()