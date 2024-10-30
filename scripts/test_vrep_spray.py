#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int8MultiArray


def ros_shutdown_func():
    rospy.sleep(1.0)  # Sleeps for 1.0 sec


def main():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('test_vrep_spray', anonymous=True)

    rospy.on_shutdown(ros_shutdown_func)

    pub = rospy.Publisher('cmd_spray', Int8MultiArray, queue_size=5)

    # msgs:
    msg_data = Int8MultiArray()

    while not rospy.is_shutdown():

        # ex 1
        data = [0, 0, 0, 0, 0, 0, 0, 0]
        msg_data.data = data
        pub.publish(msg_data)
        rospy.sleep(2.0)
        
        # ex 1
        data = [1, 30, 0, 0, 0, 0, 0, 0]
        msg_data.data = data
        pub.publish(msg_data)
        rospy.sleep(1.0)
        
        # ex 1
        data = [1, 0, 0, 1, 0, 0, 0, 0]
        msg_data.data = data
        pub.publish(msg_data)
        rospy.sleep(2.0)
        
        # ex 1
        data = [0, 0, 0, 0, 0, 0, 0, 0]
        msg_data.data = data
        pub.publish(msg_data)
        rospy.sleep(2.0)
        
        # ex 1
        data = [1, 0, 30, 0, 0, 0, 0, 0]
        msg_data.data = data
        pub.publish(msg_data)
        rospy.sleep(1.0)
        
        # ex 1
        data = [1, 0, 0, 1, 0, 0, 0, 0]
        msg_data.data = data
        pub.publish(msg_data)
        rospy.sleep(2.0)
        
        # ex 1
        data = [0, 0, 0, 0, 0, 0, 0, 0]
        msg_data.data = data
        pub.publish(msg_data)
        rospy.sleep(2.0)
        
        # ex 2
        data = [0, 0, 0, 0, 1, 30, 0, 0]
        msg_data.data = data
        pub.publish(msg_data)
        rospy.sleep(1.0)
        
        # ex 2
        data = [0, 0, 0, 0, 1, 0, 0, 1]
        msg_data.data = data
        pub.publish(msg_data)
        rospy.sleep(2.0)
        
        # ex 2
        data = [0, 0, 0, 0, 0, 0, 0, 0]
        msg_data.data = data
        pub.publish(msg_data)
        rospy.sleep(2.0)
        
        # ex 2
        data = [0, 0, 0, 0, 1, 0, 30, 0]
        msg_data.data = data
        pub.publish(msg_data)
        rospy.sleep(1.0)
        
        # ex 2
        data = [0, 0, 0, 0, 1, 0, 0, 1]
        msg_data.data = data
        pub.publish(msg_data)
        rospy.sleep(2.0)
        
        # ex 2
        data = [0, 0, 0, 0, 0, 0, 0, 0]
        msg_data.data = data
        pub.publish(msg_data)
        rospy.sleep(2.0)
        
        

        


if __name__ == '__main__':
    main()
