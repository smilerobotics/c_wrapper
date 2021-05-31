// Copyright 2021 Smile Robotics, inc
#include <memory>

#include "c_wrapper_nav2/c_wrapper_nav2.h"
#include "c_wrapper_nav2/nav2_client.hpp"

using c_wrapper_nav2::Nav2Client;
std::shared_ptr<Nav2Client> g_nav_client;

int nav2_init()
{
  if (!g_nav_client)
  {
    rclcpp::NodeOptions option;
    g_nav_client = std::make_shared<c_wrapper_nav2::Nav2Client>(option);
    return 0;
  }
  else
  {
    RCLCPP_ERROR(g_nav_client->get_logger(), "Multiple nav2 is not supported yet");
    ;
    return -1;
  }
}

int nav2_send_goal(const double x, const double y, const double theta)
{
  if (!g_nav_client)
  {
    std::cerr << "call nav2_init() before" << std::endl;
    return -10;
  }
  if (g_nav_client->send_goal(x, y, theta) != 0)
  {
    std::cerr << "failed to send goal" << std::endl;
    return -1;
  }
  rclcpp::spin_some(g_nav_client);
  return 0;
}

int nav2_wait_until_reach(const double timeout_sec)
{
  if (!g_nav_client)
  {
    std::cerr << "call nav2_init() before" << std::endl;
    return -10;
  }
  return Nav2Client::wait_until_reach(g_nav_client, timeout_sec);
}

int nav2_cancel_all_goals()
{
  if (!g_nav_client)
  {
    std::cerr << "call nav2_init() before" << std::endl;
    return -10;
  }
  if (g_nav_client->cancel_all_goals())
  {
    rclcpp::spin_some(g_nav_client);
  }
  else
  {
    std::cerr << "failed to cancel" << std::endl;
    return -10;
  }

  return 0;
}
