// Copyright 2021 Smile Robotics, inc
#ifndef C_WRAPPER_NAV2__NAV2_CLIENT_HPP_
#define C_WRAPPER_NAV2__NAV2_CLIENT_HPP_

#include <memory>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace c_wrapper_nav2
{
class Nav2Client : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using Future = std::shared_future<GoalHandleNavigateToPose::SharedPtr>;
  explicit Nav2Client(const rclcpp::NodeOptions &options);
  bool send_goal(double x, double y, double theta);
  bool cancel_all_goals();
  // @return 0 success: negative: fail
  // @return -1 canceled
  // @return -2 timeout
  static int wait_until_reach(const std::shared_ptr<Nav2Client> &node, const double timeout_sec);
  const Future &get_goal_handle_future()
  {
    return m_goal_handle_future;
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
  Future m_goal_handle_future;
};

}  // namespace c_wrapper_nav2

#endif  // C_WRAPPER_NAV2__NAV2_CLIENT_HPP_
