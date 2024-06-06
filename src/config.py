import numpy as np

limit_joint1_upper =  1.571
limit_joint1_lower = -1.571
limit_joint2_upper =  0.050
limit_joint2_lower = -1.571
limit_joint3_upper =  0.050
limit_joint3_lower = -1.571
limit_joint4_upper =  1.571
limit_joint4_lower = -1.571

limit_joint_threshold_activate      = 0.05
limit_joint_threshold_deactivate    = 0.08

# Task hierarchy definition
limit_range_joint1   = np.array([limit_joint1_lower, limit_joint1_upper]).reshape(1,2)
limit_range_joint2   = np.array([limit_joint2_lower, limit_joint2_upper]).reshape(1,2)
limit_range_joint3   = np.array([limit_joint3_lower, limit_joint3_upper]).reshape(1,2)
limit_range_joint4   = np.array([limit_joint4_lower, limit_joint4_upper]).reshape(1,2)

threshold_joint      = np.array([limit_joint_threshold_activate, limit_joint_threshold_deactivate]).reshape(2,1)

weight_base_rotate      = 10.000
weight_base_translate   = 50.000
weight_joint1           = 0.500
weight_joint2           = 1.000
weight_joint3           = 1.000
weight_joint4           = 1.000

weight_matrix           = np.diag([weight_base_rotate,
                                    weight_base_translate, 
                                    weight_joint1, 
                                    weight_joint2, 
                                    weight_joint3,
                                    weight_joint4])

# joint_state_topic  = "/turtlebot/joint_states"
# aruco_pose_topic   = "/aruco_pose"
# task_topic         = "None"
# cmd_vel_topic      = "/cmd_vel"
# cmd_dq_topic       = "/turtlebot/swiftpro/joint_velocity_controller/command"

FRAME_MAP               = "map"

color_camera_SIL_topic  = "/turtlebot/kobuki/realsense/color/image_color"
color_camera_HIL_topic  = "/turtlebot/kobuki/realsense/color/image_raw"
camera_info_topic       = "/turtlebot/kobuki/realsense/color/camera_info"
odom_SIL_topic          = "/odom"
odom_HIL_topic          = "/state_estimation"
aruco_topic             = "/aruco_pose"
MODE                    = "SIL"
# Aruco
MAKER_SIZE              = 0.05
BOX_WIDTH               = 0.07
BOX_LENGTH              = 0.07
BOX_HEIGHT              = 0.07

CAM_BASE_X              = 0.136
CAM_BASE_Y              = -0.033
CAM_BASE_Z              = -0.116

CAM_BASE_QX             = 0.500
CAM_BASE_QY             = 0.500
CAM_BASE_QZ             = 0.500
CAM_BASE_QW             = 0.500


# MOBILE MANIPULATOR
MANI_BX                 = 0.0132        # [met]
MANI_BZ                 = 0.1080        # [met]
MANI_D1                 = 0.1420        # [met]
MANI_D2                 = 0.1588        # [met]
MANI_MZ                 = 0.0722        # [met]
MANI_MX                 = 0.0565        # [met]
MANI_ALPHA_SIL          = -np.pi/2.0    # [rad]
MANI_ALPHA_HIL          = np.pi/2.0     # [rad]

