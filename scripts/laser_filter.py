#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import rospkg
import yaml


class LaserFilter:
    def __init__(self):
        # cfgfile_path = rospy.get_param('cfgfile')
        # with open(cfgfile_path, 'r') as stream:
        #    self.param = yaml.safe_load(stream)

        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('xnrobot_id11') + '/'

        param_yaml_file = rospy.get_param('/param_yaml_file')
        with open(param_yaml_file, 'r') as file:
            self.param = yaml.safe_load(file)

        ######################################################

        self.valid_angle_ranges = np.array([[self.param['filter_laser']['valid_angle_ranges']]]).reshape(-1,2)

        self.laser_range_limit = self.param['filter_laser']['laser_range_limit']

        ######################################################

        self.laser_pub = rospy.Publisher("/scan", LaserScan, queue_size=5)
        self.laser_sub = rospy.Subscriber("/scan_raw", LaserScan, self.filter_laser)

        return

    def filter_laser(self, laser_msg):
        ranges = laser_msg.ranges[:]
        ranges_np = np.array(ranges)

        ranges_valid_np = -1.0*np.ones(ranges_np.shape)

        # 1. flip & and make all invalid range to -1
        ranges_np = ranges_np[::-1] # reverse order for the upside down laser scanner

        ranges_np[ranges_np<=0.0] = -1.0

        # 2. take valid beams

        # to obtain "valid_angle_ranges", which is from -pi to pi, park robot in a empty space (free space dist > 0.5)
        # enter debug mode and stop here, then:
        # a = np.array([i for i in range(0, 360)])
        # b = (ranges_np<0.5) & (ranges_np>0.0)
        # a[b] = 0
        # a
        # then write index of the valid a.

        valid_angle_ranges_num = self.valid_angle_ranges.shape[0]

        for i in range(valid_angle_ranges_num):
            ranges_valid_np[self.valid_angle_ranges[i,0]:self.valid_angle_ranges[i,1]] = ranges_np[self.valid_angle_ranges[i,0]:self.valid_angle_ranges[i,1]]

        # 3. apply range limit
        ranges_valid_np[ranges_valid_np>self.laser_range_limit] = -1.0

        # 4. apply global coord limit

        laser_msg.header.frame_id = 'laser_link'
        laser_msg.ranges = ranges_valid_np.tolist()
        self.laser_pub.publish(laser_msg)

        # rospy.logwarn('/scan published')


def ros_shutdown_func():
    rospy.sleep(1.0)  # Sleeps for 1.0 sec


def main():
    rospy.init_node('laser_filter_node', anonymous=True)
    rospy.on_shutdown(ros_shutdown_func)

    lf = LaserFilter()

    rospy.spin()


if __name__ == '__main__':
    main()
