# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;controller_interface;hardware_interface;pluginlib".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lmycontroller".split(';') if "-lmycontroller" != "" else []
PROJECT_NAME = "mycustomcontroller"
PROJECT_SPACE_DIR = "/home/ahmed/RosnoeticWS/install"
PROJECT_VERSION = "0.0.0"
