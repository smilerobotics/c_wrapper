// Copyright 2021 Smile Robotics, inc
#ifndef C_WRAPPER_ROS2__C_WRAPPER_ROS2_H_
#define C_WRAPPER_ROS2__C_WRAPPER_ROS2_H_

extern "C"
{
  void ros2_init(int argc, char **argv);
  void ros2_shutdown();
}

#endif  // C_WRAPPER_ROS2__C_WRAPPER_ROS2_H_
