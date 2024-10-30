#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import numpy as np
import numpy.matlib
import math
import tf

from transform_tools import *

######################
slam_dist_thr = 0.10
slam_theta_thr = 10*(math.pi/180)

init_pose = np.array([[-5.77,-6.0,0]]).T
init_cov = np.array([[  1.0**2, 0, 0],
                    [   0, 1.0**2, 0],
                    [   0, 0, (10*(math.pi/180))**2]])
lm_cov = 0.1**2

laser_in_baselink_trans_ypr = np.array([[0.1], [0], [0], [0], [0], [0]])
laser_in_baselink_M = transform_matrix_from_trans_ypr(laser_in_baselink_trans_ypr)

odom_laser_t_diff_max = 0.3
laser_dist_max = 3.0
laser_data_asso_dist_thr = 0.3

odom_dist_cov = 0.01**2
odom_theta_cov = (5*(math.pi/180))**2
laser_dist_cov = 0.05**2
laser_ang_cov = (2*(math.pi/180))**2

lm_num = 4*9
#lm_x = repmat((4:-1:-4)',1,4)
lm_x = np.matlib.repmat(np.array([np.arange(4.0, -5.0, -1.0)]).T, 1, 4)
#lm_y = repmat((4.2:-2.8:-4.2),9,1)
lm_y = np.matlib.repmat(np.array([np.arange(4.2, -7.0, -2.8)]), 9, 1)

#lm_x = reshape(lm_x,[],1);
lm_x = lm_x.T.reshape((-1,1))
#lm_y = reshape(lm_y,[],1);
lm_y = lm_y.T.reshape((-1,1))

##########################

mu = np.zeros((3+4*9*2,1))
mu[0:3] = init_pose
mu[3::2] = lm_x
mu[4::2] = lm_y

init_pose_M = transform_matrix_from_trans_ypr( np.array([[init_pose[0]],[init_pose[1]],[0],[init_pose[2]],[0],[0]]) )
# P = [init_cov,zeros(3,4*9*2);
#      zeros(4*9*2,3),eye(4*9*2)*lm_cov];
P = np.block([[init_cov, np.zeros((3,4*9*2))],
              [np.zeros((4*9*2,3)), np.eye(4*9*2)*lm_cov]])

mu_hist =  None
odom_pre = None
odom_pre_hist = None
laser_msg_latest = {"t": 0.0, "msg": LaserScan()}

marker_pub = rospy.Publisher("nyjqrds2021_ekf_loc_res", MarkerArray, queue_size=5)

br = tf.TransformBroadcaster()
##########################

def prediction_step_2d(mu, sigma, u, u_dist_std, u_theta_std):
    # mu(1) = mu(1) + u.t * cos(mu(3) + u.r1);
    # mu(2) = mu(2) + u.t * sin(mu(3) + u.r1);
    # mu(3) = mu(3) + u.r1 + u.r2;
    # mu(3) = wrapToPi(mu(3));
    mu[0,0] = mu[0,0]+u["t"]*np.cos(mu[2,0]+u["r1"])
    mu[1, 0] = mu[1, 0] + u["t"] * np.sin(mu[2, 0] + u["r1"])
    mu[2,0] = mu[2,0] +u['r1']+u['r2']
    mu[2, 0] = wrap_to_pi(mu[2,0])

    # G_t_x = [1, 0, -1 * u.t * sin(mu(3) + u.r1);
    # 0, 1, u.t * cos(mu(3) + u.r1);
    # 0, 0, 1];
    G_t_x = np.array([ [1.0, 0.0, -1 * u["t"] * np.sin(mu[2,0] + u["r1"])] ,
                       [0.0, 1.0, u["t"] * np.cos(mu[2,0] + u["r1"])] ,
                       [0.0, 0.0, 1.0] ])

    # G_t = [G_t_x, zeros(3, (size(mu, 1) - 3));
    #         zeros((size(mu, 1) - 3), 3), eye(size(mu, 1) - 3)];
    G_t = np.block([[G_t_x, np.zeros((3,mu.shape[0]-3))],
                    [np.zeros((mu.shape[0]-3,3)), np.eye(mu.shape[0]-3)]])

    # R3 = [u_dist_std ^ 2, 0, 0;
    # 0, u_dist_std ^ 2, 0;
    # 0, 0, u_theta_std ^ 2];
    R3 = np.array([[u_dist_std**2, 0.0, 0.0],
                   [0.0, u_dist_std**2, 0.0],
                   [0.0, 0.0, u_theta_std**2]])
    # R = zeros(size(sigma, 1));
    # R(1: 3, 1: 3) = R3;
    R = np.zeros((sigma.shape[0],sigma.shape[0]))
    R[0:3,0:3] = R3
    # sigma = G_t*sigma*G_t'+R;
    sigma = G_t @ sigma @ (G_t.T) + R

    return mu, sigma


def correction_step_rblm_2d(mu, sigma, z,obs_rblm_azim_std,obs_rblm_r_std):
    m = len(z) # z is a list of dict

    # Z = zeros(m*2, 1);
    Z = np.zeros((m*2,1))
    expectedZ = np.zeros((m*2,1))

    H = None

    for i in range(m):
        landmarkId = z[i]["id"]

        # Z(2*i-1) = z(i).range;
        # Z(2*i) = z(i).bearing;
        Z[2*i,0] = z[i]["range"]
        Z[2*i+1,0] = z[i]["bearing"]

        # expectedZ(2*i-1) = sqrt((mu(2*landmarkId+2)-mu(1))^2 + (mu(2*landmarkId+3)-mu(2))^2);
        # expectedZ(2*i) = wrapToPi(atan2((mu(2*landmarkId+3)-mu(2)),(mu(2*landmarkId+2)-mu(1))) - mu(3));
        expectedZ[2*i,   0] = np.sqrt((mu[2*landmarkId+3,0]-mu[0,0])**2 + (mu[2*landmarkId+4,0]-mu[1,0])**2)
        expectedZ[2*i+1, 0] = wrap_to_pi( np.arctan2( mu[2*landmarkId+4,0]-mu[1,0], mu[2*landmarkId+3,0]-mu[0,0]) - mu[2,0] )

        # delta = [(mu(2*landmarkId+2)-mu(1));
        # 		 (mu(2*landmarkId+3)-mu(2))];
        # 	q = delta'*delta;
        delta = np.array([[mu[2*landmarkId+3,0]-mu[0,0]], [mu[2*landmarkId+4,0]-mu[1,0]]])
        q = ((delta.T) @ delta)[0,0]

        # Hi_low = (1/q)*[-sqrt(q)*delta(1),-sqrt(q)*delta(2),0,sqrt(q)*delta(1),sqrt(q)*delta(2);
        #                     delta(2),-delta(1),-q,-delta(2),delta(1)];
        Hi_low = (1/q)*np.array([[-np.sqrt(q)*delta[0,0], -np.sqrt(q)*delta[1,0], 0, np.sqrt(q)*delta[0,0], np.sqrt(q)*delta[1,0]],
                                 [delta[1,0], -delta[0,0], -q, -delta[1,0], delta[0,0]]])

        Hi = np.zeros((2, mu.shape[0]))
        # Hi(:, 1: 3) = Hi_low(:, 1: 3);
        # Hi(:, 2 * landmarkId + 2: 2 * landmarkId + 3) = Hi_low(:, 4: 5);
        Hi[:, 0:3] = Hi_low[:,0:3]
        Hi[:, 2 * landmarkId + 3: 2 * landmarkId + 5] = Hi_low[:, 3: 5]

        # H = [H;Hi];
        if H is None:
            H = Hi
        else:
            H = np.block([[H],[Hi]])

    # Q = diag(repmat([obs_rblm_r_std ^ 2; obs_rblm_azim_std ^ 2], m, 1));
    Q = np.diag( (np.matlib.repmat( np.array([[obs_rblm_r_std**2], [obs_rblm_azim_std**2]]), m,1)).reshape(-1,) )
    # K = sigma*(H')/(H*sigma*H'+Q);
    K = sigma @ (H.T) @ np.linalg.pinv(H @ sigma @ (H.T) + Q)

    # Z_diff = normalize_all_bearings_rblm_2d(Z - expectedZ);
    Z_diff = Z - expectedZ
    for i in range(m):
        Z_diff[2*i+1,0] = wrap_to_pi(Z_diff[2*i+1,0])

    # mu = mu + K * Z_diff;
    # sigma = (eye(size(mu, 1)) - K * H) * sigma;
    mu = mu + K @ Z_diff
    sigma = (np.eye(mu.shape[0]) - K @ H) @ sigma

    return mu, sigma

def plot_state(mu, sigma, z, odom, robot_pose_gt_hist,laser_pc_filter_in_map, mu_hist):
    marker_array = MarkerArray()
    marker_array.markers = []

    # landmarks
    lm_marker = Marker()
    lm_marker.header.frame_id = "map"
    lm_marker.ns = "nyjqrds2021_ekf_loc_" + "lm"
    lm_marker.id = 0
    lm_marker.type = Marker.CUBE_LIST
    lm_marker.action = Marker.ADD
    pose = Pose()
    pose.orientation.w = 1.0
    lm_marker.pose = pose
    # when list is used, color needs to be 1.0 not 255, such a bug!
    lm_marker.color.r, lm_marker.color.g, lm_marker.color.b = (1.0, 1.0, 0)
    lm_marker.color.a = 1.0
    lm_marker.scale.x, lm_marker.scale.y, lm_marker.scale.z = (0.1, 0.1, 0.1)

    lm_marker.points = []
    # lm_marker.colors = []
    for i in range(lm_num):
        pt = Point()
        pt.x = mu[3+2*i+0,0]
        pt.y = mu[3+2*i+1,0]
        pt.z = 0
        lm_marker.points.append(pt)

        # color = ColorRGBA()
        # color.r, color.g, color.b = (255, 255, 255)
        # color.a = 1.0
        # lm_marker.colors.append(color)

    marker_array.markers.append(lm_marker)

    # robot pose
    rob_marker = Marker()
    rob_marker.header.frame_id = "map"
    rob_marker.ns = "nyjqrds2021_ekf_loc_" + "rob"
    rob_marker.id = 0
    rob_marker.type = Marker.CUBE
    rob_marker.action = Marker.ADD

    pose = Pose()
    pose.position.x = mu[0,0]
    pose.position.y = mu[1,0]
    pose.position.z = 0.0
    quat = ypr2quat(np.array([mu[2, 0], 0.0, 0.0]))
    pose.orientation.x = quat[0,0]
    pose.orientation.y = quat[1,0]
    pose.orientation.z = quat[2,0]
    pose.orientation.w = quat[3,0]
    rob_marker.pose = pose

    rob_marker.color.r, rob_marker.color.g, rob_marker.color.b = (0, 0, 255)
    rob_marker.color.a = 1.0
    rob_marker.scale.x, rob_marker.scale.y, rob_marker.scale.z = (0.8, 0.8, 0.05)

    marker_array.markers.append(rob_marker)

    # send tf
    br.sendTransform((pose.position.x, pose.position.y, pose.position.z),
                     # br.sendTransform(translation, rotation, time, child, parent)
                     (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                     rospy.Time.now(),
                     'base_link', 'map')

    # pose direction
    rob_dir_marker = Marker()
    rob_dir_marker.header.frame_id = "map"
    rob_dir_marker.ns = "nyjqrds2021_ekf_loc_" + "rob"
    rob_dir_marker.id = 1
    rob_dir_marker.type = Marker.ARROW
    rob_dir_marker.action = Marker.ADD

    pose = Pose()
    pose.position.x = mu[0,0]
    pose.position.y = mu[1,0]
    pose.position.z = 0.0
    quat = ypr2quat(np.array([mu[2,0],0.0,0.0]))
    pose.orientation.x = quat[0,0]
    pose.orientation.y = quat[1,0]
    pose.orientation.z = quat[2,0]
    pose.orientation.w = quat[3,0]
    rob_dir_marker.pose = pose

    rob_dir_marker.color.r, rob_dir_marker.color.g, rob_dir_marker.color.b = (255, 0, 0)
    rob_dir_marker.color.a = 1.0
    rob_dir_marker.scale.x, rob_dir_marker.scale.y, rob_dir_marker.scale.z = (1.0, 0.05, 0.05)

    marker_array.markers.append(rob_dir_marker)

    # robot odom traj
    odom_marker = Marker()
    odom_marker.header.frame_id = "map"
    odom_marker.ns = "nyjqrds2021_ekf_loc_" + "odom"
    odom_marker.id = 0
    odom_marker.type = Marker.LINE_STRIP
    odom_marker.action = Marker.ADD
    pose = Pose()
    pose.orientation.w = 1
    odom_marker.pose = pose
    odom_marker.color.r, odom_marker.color.g, odom_marker.color.b = (1.0, 1.0, 1.0)
    odom_marker.color.a = 0.5
    odom_marker.scale.x, odom_marker.scale.y, odom_marker.scale.z = (0.05, 0.05, 0.05)

    odom_marker.points = []
    odom_num = odom.shape[1]
    for i in range(odom_num):
        pt = Point()
        pt.x = odom[0, i]
        pt.y = odom[1, i]
        pt.z = 0
        odom_marker.points.append(pt)

    marker_array.markers.append(odom_marker)

    # robot ekf traj
    ekf_traj_marker = Marker()
    ekf_traj_marker.header.frame_id = "map"
    ekf_traj_marker.ns = "nyjqrds2021_ekf_loc_" + "ekf_traj"
    ekf_traj_marker.id = 0
    ekf_traj_marker.type = Marker.LINE_STRIP
    ekf_traj_marker.action = Marker.ADD
    pose = Pose()
    pose.orientation.w = 1
    ekf_traj_marker.pose = pose
    ekf_traj_marker.color.r, ekf_traj_marker.color.g, ekf_traj_marker.color.b = (0, 0, 1.0)
    ekf_traj_marker.color.a = 1.0
    ekf_traj_marker.scale.x, ekf_traj_marker.scale.y, ekf_traj_marker.scale.z = (0.05, 0.05, 0.05)

    ekf_traj_marker.points = []
    ekf_traj_num = mu_hist.shape[1]
    for i in range(ekf_traj_num):
        pt = Point()
        pt.x = mu_hist[0, i]
        pt.y = mu_hist[1, i]
        pt.z = 0
        ekf_traj_marker.points.append(pt)

    marker_array.markers.append(ekf_traj_marker)

    # robot laser obs
    laser_obs_marker = Marker()
    laser_obs_marker.header.frame_id = "map"
    laser_obs_marker.ns = "nyjqrds2021_ekf_loc_" + "laser_obs"
    laser_obs_marker.id = 0
    laser_obs_marker.type = Marker.LINE_LIST
    laser_obs_marker.action = Marker.ADD

    pose = Pose()
    pose.position.x = mu[0, 0]
    pose.position.y = mu[1, 0]
    pose.position.z = 0.0
    quat = ypr2quat(np.array([mu[2, 0], 0.0, 0.0]))
    pose.orientation.x = quat[0, 0]
    pose.orientation.y = quat[1, 0]
    pose.orientation.z = quat[2, 0]
    pose.orientation.w = quat[3, 0]
    laser_obs_marker.pose = pose

    laser_obs_marker.color.r, laser_obs_marker.color.g, laser_obs_marker.color.b = (1.0, 0, 0.0)
    laser_obs_marker.color.a = 0.5
    laser_obs_marker.scale.x, laser_obs_marker.scale.y, laser_obs_marker.scale.z = (0.05, 0.05, 0.05)

    laser_obs_marker.points = []
    pt_rob = Point()
    pt_rob.x = 0 # in robot frame
    pt_rob.y = 0 # in robot frame
    pt_rob.z = 0 # in robot frame
    z_num = len(z)
    for i in range(z_num):
        # rob point
        laser_obs_marker.points.append(pt_rob)
        # z point
        pt = Point()
        pt.x = z[i]['range']*np.cos(z[i]['bearing']) # in robot frame
        pt.y = z[i]['range']*np.sin(z[i]['bearing']) # in robot frame
        pt.z = 0
        laser_obs_marker.points.append(pt)

    marker_array.markers.append(laser_obs_marker)

    # publish marker_array
    global marker_pub
    marker_pub.publish(marker_array)


def draw_robot_cur_pose(robot_cur_pose):
    marker_array = MarkerArray()
    marker_array.markers = []

    # robot current pose
    rob_marker = Marker()
    rob_marker.header.frame_id = "map"
    rob_marker.ns = "nyjqrds2021_ekf_loc_" + "robot_cur_pose"
    rob_marker.id = 0
    rob_marker.type = Marker.CUBE
    rob_marker.action = Marker.ADD

    pose = Pose()
    pose.position.x = robot_cur_pose[0, 0]
    pose.position.y = robot_cur_pose[1, 0]
    pose.position.z = 0.0
    quat = ypr2quat(np.array([robot_cur_pose[2, 0], 0.0, 0.0]))
    pose.orientation.x = quat[0,0]
    pose.orientation.y = quat[1,0]
    pose.orientation.z = quat[2,0]
    pose.orientation.w = quat[3,0]
    rob_marker.pose = pose

    rob_marker.color.r, rob_marker.color.g, rob_marker.color.b = (255, 255, 255)
    rob_marker.color.a = 0.5
    rob_marker.scale.x, rob_marker.scale.y, rob_marker.scale.z = (0.8, 0.8, 0.05)

    marker_array.markers.append(rob_marker)

    # send tf
    br.sendTransform((pose.position.x, pose.position.y, pose.position.z),
                     # br.sendTransform(translation, rotation, time, child, parent)
                     (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                     rospy.Time.now(),
                     'base_link', 'map')

    # pose direction
    rob_dir_marker = Marker()
    rob_dir_marker.header.frame_id = "map"
    rob_dir_marker.ns = "nyjqrds2021_ekf_loc_" + "robot_cur_pose"
    rob_dir_marker.id = 1
    rob_dir_marker.type = Marker.ARROW
    rob_dir_marker.action = Marker.ADD

    pose = Pose()
    pose.position.x = robot_cur_pose[0, 0]
    pose.position.y = robot_cur_pose[1, 0]
    pose.position.z = 0.0
    quat = ypr2quat(np.array([robot_cur_pose[2, 0], 0.0, 0.0]))
    pose.orientation.x = quat[0,0]
    pose.orientation.y = quat[1,0]
    pose.orientation.z = quat[2,0]
    pose.orientation.w = quat[3,0]
    rob_dir_marker.pose = pose

    rob_dir_marker.color.r, rob_dir_marker.color.g, rob_dir_marker.color.b = (255, 0, 0)
    rob_dir_marker.color.a = 0.5
    rob_dir_marker.scale.x, rob_dir_marker.scale.y, rob_dir_marker.scale.z = (1.0, 0.05, 0.05)

    marker_array.markers.append(rob_dir_marker)

    # publish marker_array
    global marker_pub
    marker_pub.publish(marker_array)

######################################################
def ros_shutdown_func():
    rospy.sleep(1.0)  # Sleeps for 1.0 sec

def handle_odom(odom_msg):
    # odom_t = msg{1,1}.Header.Stamp.Sec+msg{1,1}.Header.Stamp.Nsec*(1e-9);
    odom_t = odom_msg.header.stamp.secs+odom_msg.header.stamp.nsecs*(1e-9)
    # odom_q = [msg{1,1}.Pose.Pose.Orientation.W, msg{1,1}.Pose.Pose.Orientation.X, msg{1,1}.Pose.Pose.Orientation.Y, msg{1,1}.Pose.Pose.Orientation.Z];
    odom_q = np.array([[odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]])
    odom_ypr = quat2ypr(odom_q)
    odom_theta = odom_ypr[0,0]
    odom = np.array([[odom_msg.pose.pose.position.x],[odom_msg.pose.pose.position.y],[odom_theta]])

    lm_obs_id = np.zeros((4*9,1))
    z = []

    global odom_pre
    global mu
    global mu_hist
    global odom_pre_hist
    global P
    if odom_pre is None:
        odom_pre = odom
        # odom_pre_M = transform_matrix_from_trans_ypr([odom_pre(1);odom_pre(2);0;odom_pre(3);0;0])
        odom_pre_M = transform_matrix_from_trans_ypr( np.array([[odom_pre[0]],[odom_pre[1]],[0],[odom_pre[2]],[0],[0]]) )
        odom_pre_in_map_M = init_pose_M @ odom_pre_M # A @ B / A.dot(B)
        # odom_pre_in_map_trans_ypr = transform_matrix_to_pose_trans_ypr(odom_pre_in_map_M)
        odom_pre_in_map_trans_ypr = transform_matrix_to_pose_trans_ypr(odom_pre_in_map_M)
        # odom_pre_in_map = [odom_pre_in_map_trans_ypr(1);odom_pre_in_map_trans_ypr(2);odom_pre_in_map_trans_ypr(4)];
        odom_pre_in_map = np.array([[odom_pre_in_map_trans_ypr[0,0]],[odom_pre_in_map_trans_ypr[1,0]],[odom_pre_in_map_trans_ypr[3,0]]])
        # odom_pre_hist = [odom_pre_hist, odom_pre_in_map];
        odom_pre_hist = odom_pre_in_map
        # mu_hist = [mu_hist, mu(1:3)];
        mu_hist = mu[0:3,0:1]
    else:
        # odom_M = transform_matrix_from_trans_ypr([odom(1);odom(2);0;odom(3);0;0]);
        odom_M = transform_matrix_from_trans_ypr( np.array([[odom[0]], [odom[1]], [0], [odom[2]], [0], [0]]))
        odom_pre_M = transform_matrix_from_trans_ypr( np.array([[odom_pre[0]], [odom_pre[1]], [0], [odom_pre[2]], [0], [0]]))

        # odom_diff_M = odom_pre_M\odom_M;
        odom_diff_M = np.linalg.lstsq(odom_pre_M,odom_M,rcond=None)[0] # left division
        odom_diff_trans_ypr = transform_matrix_to_pose_trans_ypr(odom_diff_M)
        odom_diff = np.array([[odom_diff_trans_ypr[0,0]], [odom_diff_trans_ypr[1,0]], [odom_diff_trans_ypr[3,0]]])

        # u.r1 = wrapToPi(atan2(odom_diff(2), odom_diff(1)));
        # u.t = sqrt(odom_diff(1) ^ 2 + odom_diff(2) ^ 2);
        # u.r2 = wrapToPi(odom_diff(3) - u.r1);
        u = {"r1": 0.0, "t": 0.0, "r2": 0.0}
        u["r1"] = wrap_to_pi( np.arctan2(odom_diff[1,0], odom_diff[0,0]) )
        u["t"] = np.sqrt(odom_diff[0,0]**2 + odom_diff[1,0]**2)
        u["r2"] = wrap_to_pi(odom_diff[2,0] - u["r1"])

        if np.sqrt(odom_diff[0,0]**2+odom_diff[1,0]**2)>slam_dist_thr or abs(odom_diff[2,0])>slam_theta_thr:
            # prediction:
            # [mu, P] = prediction_step_2d(mu, P, u, sqrt(odom_dist_cov), sqrt(odom_theta_cov));
            mu, P = prediction_step_2d(mu, P, u, np.sqrt(odom_dist_cov), np.sqrt(odom_theta_cov))
            # mu(4: 2:end) = lm_x;
            # mu(5: 2:end) = lm_y;
            mu[3::2] = lm_x
            mu[4::2] = lm_y
            # P = [P(1:3, 1: 3), zeros(3, 4 * 9 * 2);
            # zeros(4 * 9 * 2, 3), eye(4 * 9 * 2) * lm_cov];
            P = np.block([[P[0:3, 0:3], np.zeros((3,4*9*2))],
                          [np.zeros((4*9*2,3)), np.eye(4*9*2)*lm_cov]])

            # check laser readings
            if abs(odom_t - laser_msg_latest["t"]) < odom_laser_t_diff_max:
                # laser_range = laser_msg_latest.msg.Ranges;
                laser_range = np.array(laser_msg_latest["msg"].ranges).reshape(-1,1)
                # laser_angle = laser_msg_latest.msg.AngleMin + (0:laser_msg_latest.msg.AngleIncrement:laser_msg_latest.msg.AngleIncrement * (size(laser_range, 1) - 1))';
                laser_angle = (laser_msg_latest["msg"].angle_min+np.arange(0,laser_msg_latest["msg"].angle_increment * laser_range.shape[0], laser_msg_latest["msg"].angle_increment)).reshape(-1,1)
                # laser_pc = [laser_range.*cos(laser_angle),laser_range.*sin(laser_angle)];
                laser_pc = np.block([[laser_range*np.cos(laser_angle), laser_range*np.sin(laser_angle)]])
                # laser_pc_filter = laser_pc(laser_range > 0 & laser_range < laser_dist_max,:);
                # laser_range_filter = laser_range(laser_range > 0 & laser_range < laser_dist_max);
                # laser_angle_filter = laser_angle(laser_range > 0 & laser_range < laser_dist_max);
                laser_pc_filter = laser_pc[((laser_range > 0) & (laser_range < laser_dist_max)).reshape(-1,),:]
                # laser_range_filter = laser_range[((laser_range > 0) & (laser_range < laser_dist_max)).reshape(-1,),:]
                # laser_angle_filter = laser_angle[((laser_range > 0) & (laser_range < laser_dist_max)).reshape(-1, ), :]

                # transform laser points to base_link robot frame
                # laser_pc_filter = laser_in_baselink_M * [laser_pc_filter'; zeros(1,size(laser_pc_filter,1)); ones(1,size(laser_pc_filter,1))];
                # laser_pc_filter = laser_pc_filter(1:2,:)';
                # laser_range_filter = sqrt(laser_pc_filter(:, 1).^ 2 + laser_pc_filter(:, 2).^ 2);
                # laser_angle_filter = atan2(laser_pc_filter(:, 2), laser_pc_filter(:, 1));
                laser_pc_filter = laser_in_baselink_M @ np.block([[laser_pc_filter.T], [np.zeros((1,laser_pc_filter.shape[0]))], [np.ones((1,laser_pc_filter.shape[0]))]])
                laser_pc_filter = laser_pc_filter[0:2,:].T
                laser_range_filter = np.sqrt(laser_pc_filter[:,0:1]**2 + laser_pc_filter[:,1:2]**2)
                laser_angle_filter = np.arctan2(laser_pc_filter[:,1:2],laser_pc_filter[:,0:1])


                if laser_pc_filter.shape[0]!=0:
                    # mu_rob_M = transform_matrix_from_trans_ypr([mu(1);mu(2);0;mu(3);0;0]);
                    mu_rob_M = transform_matrix_from_trans_ypr( np.array([[mu[0,0]], [mu[1,0]], [0], [mu[2,0]], [0], [0]]))

                    # laser_pc_filter_in_map = mu_rob_M * [laser_pc_filter';zeros(1,size(laser_pc_filter,1));ones(1,size(laser_pc_filter,1))];
                    # laser_pc_filter_in_map = laser_pc_filter_in_map(1:2,:)';
                    laser_pc_filter_in_map = mu_rob_M @ np.block([[laser_pc_filter.T],[np.zeros((1,laser_pc_filter.shape[0]))],[np.ones((1,laser_pc_filter.shape[0]))]])
                    # laser_pc_filter_in_map = laser_pc_filter_in_map(1:2,:)';
                    laser_pc_filter_in_map = laser_pc_filter_in_map[0:2,:].T

                    z_idx = 0
                    for i in range(laser_pc_filter_in_map.shape[0]):
                        pt = laser_pc_filter_in_map[i,:]
                        # [min_v,min_idx] = min(sqrt((pt(1)-lm_x).^2+(pt(2)-lm_y).^2));
                        min_v = np.min(np.sqrt((pt[0] - lm_x)**2 + (pt[1] - lm_y)**2))
                        min_idx = np.argmin(np.sqrt((pt[0] - lm_x) ** 2 + (pt[1] - lm_y) ** 2))

                        if min_v<laser_data_asso_dist_thr:
                            if lm_obs_id[min_idx,0]==0: # if lm is hit by more than 1 laser, only take the 1st one
                                z_idx = z_idx + 1
                                # z(z_idx).id = min_idx;
                                # z(z_idx).range = laser_range_filter(i);
                                # z(z_idx).bearing = laser_angle_filter(i);
                                z.append({"id":min_idx, "range":laser_range_filter[i,0], "bearing":laser_angle_filter[i,0]})

                                lm_obs_id[min_idx,0] = 1

                    if len(z)!=0:
                        # correction step:
                        # [mu, P] = correction_step_rblm_2d(mu, P, z,sqrt(laser_ang_cov),sqrt(laser_dist_cov));
                        mu, P = correction_step_rblm_2d(mu, P, z, np.sqrt(laser_ang_cov), np.sqrt(laser_dist_cov))
                        # mu(4:2:end) = lm_x;
                        # mu(5:2:end) = lm_y;
                        # P = [P(1:3,1:3), zeros(3,4*9*2);
                        #      zeros(4*9*2,3), eye(4*9*2)*lm_cov];
                        mu[3::2,:] = lm_x
                        mu[4::2,:] = lm_y
                        P = np.block([[P[0:3,0:3], np.zeros((3,4*9*2))],
                                      [np.zeros((4*9*2,3)), np.eye(4*9*2)*lm_cov]])






            robot_cur_pose = mu[0:3,0:1]

            odom_pre = odom
            # odom_pre_M = transform_matrix_from_trans_ypr([odom_pre(1);odom_pre(2);0;odom_pre(3);0;0]);
            odom_pre_M = transform_matrix_from_trans_ypr( np.array([[odom_pre[0]], [odom_pre[1]], [0], [odom_pre[2]], [0], [0]]))
            odom_pre_in_map_M = init_pose_M @ odom_pre_M  # A @ B / A.dot(B)
            # odom_pre_in_map_trans_ypr = transform_matrix_to_pose_trans_ypr(odom_pre_in_map_M)
            odom_pre_in_map_trans_ypr = transform_matrix_to_pose_trans_ypr(odom_pre_in_map_M)
            # odom_pre_in_map = [odom_pre_in_map_trans_ypr(1);odom_pre_in_map_trans_ypr(2);odom_pre_in_map_trans_ypr(4)];
            odom_pre_in_map = np.array([[odom_pre_in_map_trans_ypr[0, 0]], [odom_pre_in_map_trans_ypr[1, 0]],
                                        [odom_pre_in_map_trans_ypr[3, 0]]])
            # odom_pre_hist = [odom_pre_hist, odom_pre_in_map];
            odom_pre_hist = np.block([[odom_pre_hist,odom_pre_in_map]])
            # mu_hist = [mu_hist, mu(1:3)];
            mu_hist = np.block([[mu_hist,mu[0:3, 0:1]]])

            # plot ekf loc results
            # plot_state(mu, P, z, odom_pre_hist, [],laser_pc_filter_in_map, mu_hist);
            plot_state(mu, P, z, odom_pre_hist, None, None, mu_hist)
        else:
            # mu_rob_M = transform_matrix_from_trans_ypr([mu(1);mu(2);0;mu(3);0;0]);
            mu_rob_M = transform_matrix_from_trans_ypr( np.array([[mu[0,0]], [mu[1,0]], [0], [mu[2,0]], [0], [0]]))
            # robot_cur_pose_M = mu_rob_M*odom_diff_M;
            robot_cur_pose_M = mu_rob_M @ odom_diff_M
            # robot_cur_pose_trans_ypr = transform_matrix_to_pose_trans_ypr(robot_cur_pose_M);
            robot_cur_pose_trans_ypr = transform_matrix_to_pose_trans_ypr(robot_cur_pose_M)
            robot_cur_pose = np.array([[robot_cur_pose_trans_ypr[0, 0]], [robot_cur_pose_trans_ypr[1, 0]],[robot_cur_pose_trans_ypr[3, 0]]])

            # draw robot_cur_pose without opti
            draw_robot_cur_pose(robot_cur_pose)

def handle_laser(laser_msg):
    global laser_msg_latest
    laser_msg_latest["t"] = laser_msg.header.stamp.secs+laser_msg.header.stamp.nsecs*(1e-9)
    laser_msg_latest["msg"] = laser_msg


def main():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('nyjqrds2021_ekf_loc', anonymous=True)

    rospy.on_shutdown(ros_shutdown_func)

    rospy.Subscriber("/odom", Odometry, handle_odom)
    rospy.Subscriber("/vrep/scan", LaserScan, handle_laser)

    # pub = rospy.Publisher('loc_markers', MarkerArray, queue_size=5)

    rospy.spin()

    # msgs:
    # msg_data = Int8MultiArray()
    #
    # while not rospy.is_shutdown():
    #     # ex 1
    #     data = [0, 0, 0, 0, 0, 0, 0, 0]
    #     msg_data.data = data
    #     pub.publish(msg_data)
    #     rospy.sleep(2.0)


if __name__ == '__main__':
    main()
