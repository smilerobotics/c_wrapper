#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "c_wrapper_nav2/nav2_client.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace c_wrapper_nav2
{
    Nav2Client::Nav2Client(const rclcpp::NodeOptions &options)
        : Node("nav2_client", options)
    {
        this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
            this,
            "navigate_to_pose");
    }

    bool Nav2Client::send_goal(const double x, const double y, const double theta)
    {
        using namespace std::placeholders;
        if (!this->client_ptr_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return false;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.w = std::cos(theta * 0.5);
        goal_msg.pose.pose.orientation.z = std::sin(theta * 0.5);

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        m_goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        return true;
    }

    bool Nav2Client::cancel_all_goals() {
        this->client_ptr_->async_cancel_all_goals();
        return true;
    }

    int wait_until_reach(const std::shared_ptr<Nav2Client> &node, const double timeout_sec) {
    {
        auto result = rclcpp::spin_until_future_complete(node, node->get_goal_handle_future(), std::chrono::duration<double>(timeout_sec));
        switch (result)
        {
        case  rclcpp::FutureReturnCode::SUCCESS:
            return 0;
            break;
        case rclcpp::FutureReturnCode::INTERRUPTED:
            RCLCPP_ERROR(node->get_logger(), "Goal was interrupted");
            return -1;
        case rclcpp::FutureReturnCode::TIMEOUT:
            RCLCPP_ERROR(node->get_logger(), "Goal was timeouted");
            return -2;
        default:
            RCLCPP_ERROR(node->get_logger(), "Unknown result code");
            return -3;
        }
        std::stringstream ss;
        ss << "Result received: ";
        //ss << result.result->distance_remaining;
        RCLCPP_INFO(node->get_logger(), ss.str().c_str());
    }
    }
} // namespace c_wrapper_nav2

RCLCPP_COMPONENTS_REGISTER_NODE(c_wrapper_nav2::Nav2Client)
