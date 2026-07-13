import os


DEFAULT_WORKSPACE_PATH = "/home/yy/inspection_migrated_ws"
DEFAULT_ROS_SETUP_PATH = "/opt/ros/jazzy/setup.bash"
DEFAULT_MAP_PATH = os.path.join(DEFAULT_WORKSPACE_PATH, "maps", "inspection_map.yaml")

TOPIC_CMD_VEL = "/cmd_vel_teleop"
TOPIC_INITIAL_POSE = "/initialpose"
TOPIC_GOAL_POSE = "/goal_pose"
TOPIC_ODOM = "/odom"
TOPIC_LASER_ODOM = "/laser_odom"
TOPIC_WHEEL_ODOM = "/wheel_odom"  # optional; real platform may not publish it
TOPIC_SCAN = "/scan"
TOPIC_IMU_RAW = "/imu/data_raw"
TOPIC_MAP = "/map"
TOPIC_AMCL_POSE = "/amcl_pose"
TOPIC_CLICKED_POINT = "/clicked_point"
TOPIC_MISSION_GOAL_POSE = "/mission_goal_pose"
TOPIC_MISSION_PREVIEW_PATH = "/mission_preview_path"
TOPIC_MISSION_POINTS_MARKERS = "/mission_points_markers"
TOPIC_MISSION_STATUS = "/mission_status"
TOPIC_MISSION_STATUS_TYPED = "/mission_status_typed"
TOPIC_THERMAL = "/thermal_frame"
TOPIC_GAS = "/gas_data"
TOPIC_SAFETY = "/robot_safety_status"

ACTION_NAVIGATE_TO_POSE = "/navigate_to_pose"

SERVICE_LOCALIZE_ROBOT = "/localize_robot"
SERVICE_START_NAVIGATION = "/start_navigation"
SERVICE_CLEAR_RVIZ_POINTS = "/clear_rviz_points"
SERVICE_SET_REGION_MODE = "/set_region_mode"
SERVICE_CLEAR_INSPECTION_REGIONS = "/clear_inspection_regions"
SERVICE_SAVE_INSPECTION_REGIONS = "/save_inspection_regions"
SERVICE_LOAD_INSPECTION_REGIONS = "/load_inspection_regions"
SERVICE_ABORT_MISSION = "/abort_mission"
SERVICE_EMERGENCY_STOP = "/emergency_stop"
SERVICE_RESET_SAFETY_MONITOR = "/reset_safety_monitor"
SERVICE_UNDO_LAST_INSPECTION_REGION = "/undo_last_inspection_region"
SERVICE_UNDO_LAST_RVIZ_POINT = "/undo_last_rviz_point"

MAX_LINEAR_SPEED_MPS = 0.18
MAX_ANGULAR_SPEED_RADPS = 0.55
DEFAULT_LINEAR_SPEED_MPS = 0.10
DEFAULT_ANGULAR_SPEED_RADPS = 0.35
