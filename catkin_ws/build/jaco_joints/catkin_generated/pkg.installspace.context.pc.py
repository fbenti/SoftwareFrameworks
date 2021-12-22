# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "arm_components_name_manager;joint_trajectory_execution".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ljaco_joints;-ljaco_trajectory_action_server".split(';') if "-ljaco_joints;-ljaco_trajectory_action_server" != "" else []
PROJECT_NAME = "jaco_joints"
PROJECT_SPACE_DIR = "/home/filippo/catkin_ws/install"
PROJECT_VERSION = "0.0.0"
