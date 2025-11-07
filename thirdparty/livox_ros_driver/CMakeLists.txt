cmake_minimum_required(VERSION 2.8.3)
project(livox_ros_driver)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs  
)

add_message_files(
  FILES
  CustomPoint.msg
  CustomMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS message_generation roscpp rospy std_msgs
)

#############
## Install ##
#############
# 只有消息定义不需要添加install规则，catkin install会自动安装消息文件