#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import numpy.matlib
import math
import tf

from transform_tools import *

import yaml

cfgfile_path = rospy.get_param('cfgfile')
with open(cfgfile_path, 'r') as stream:
    cfg_data = yaml.safe_load(stream)

######################

# init_pose = np.array([[-5.77,-6.2,0]]).T
init_pose = np.array([[cfg_data['keyparam']['init_pose'][0], cfg_data['keyparam']['init_pose'][1], cfg_data['keyparam']['init_pose'][2]]]).T

# laser_in_baselink_trans_ypr = np.array([[0.1], [0], [0], [0], [0], [0]])
laser_in_baselink_trans_ypr = np.array([[cfg_data['keyparam']['laser_in_baselink_trans_ypr'][0]],
                                        [cfg_data['keyparam']['laser_in_baselink_trans_ypr'][1]],
                                        [cfg_data['keyparam']['laser_in_baselink_trans_ypr'][2]],
                                        [cfg_data['keyparam']['laser_in_baselink_trans_ypr'][3]],
                                        [cfg_data['keyparam']['laser_in_baselink_trans_ypr'][4]],
                                        [cfg_data['keyparam']['laser_in_baselink_trans_ypr'][5]]])
laser_in_baselink_M = transform_matrix_from_trans_ypr(laser_in_baselink_trans_ypr)

##########################

br = tf.TransformBroadcaster()
##########################


def send_tf(pose_xy_theta, childframe, parentframe):
        pos = [pose_xy_theta[0],pose_xy_theta[1],0.0]
        quat = ypr2quat(np.array([pose_xy_theta[2], 0.0, 0.0]))

        # send tf
        # br.sendTransform(translation, rotation, time, child, parent)
        br.sendTransform((pos[0], pos[1], pos[2]),
                         (quat[0,0], quat[1,0], quat[2,0], quat[3,0]),
                         rospy.Time.now(),
                         childframe, parentframe)

######################################################
def ros_shutdown_func():
    rospy.sleep(1.0)  # Sleeps for 1.0 sec

def main():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('znnz2021_tf_manager', anonymous=True)

    rospy.on_shutdown(ros_shutdown_func)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # send laser_frame in laser_link tf for laser scanner's different frame name
        # send_tf([0.0,
        #          0.0,
        #          0.0],
        #         'laser_frame', 'laser_link')

        # send laser in baselink tf
        send_tf([laser_in_baselink_trans_ypr[0,0],
                 laser_in_baselink_trans_ypr[1,0],
                 laser_in_baselink_trans_ypr[3,0]],
                'laser_link','base_link')

        rate.sleep()


if __name__ == '__main__':
    main()
