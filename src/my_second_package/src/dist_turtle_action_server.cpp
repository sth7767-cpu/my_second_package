#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>
#include <vector>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

#include "my_first_package_msgs/action/dist_turtle.hpp"

class DistTurtleServer : public rclcpp::Node
{
public:
  using DistTurtle = my_first_package_msgs::action::DistTurtle;
  using GoalHandleDistTurtle = rclcpp_action::ServerGoalHandle<DistTurtle>;

  DistTurtleServer()
  : Node("dist_turtle_action_server"),
    total_dist_(0.0),
    is_first_time_(true)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose",
      10,
      std::bind(&DistTurtleServer::pose_callback, this, std::placeholders::_1)
    );

    action_server_ = rclcpp_action::create_server<DistTurtle>(
      this,
      "dist_turtle",
      std::bind(&DistTurtleServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DistTurtleServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&DistTurtleServer::handle_accepted, this, std::placeholders::_1)
    );

    this->declare_parameter("quatile_time", 0.75);
    this->declare_parameter("almost_goal_time", 0.95);

    this->get_parameter("quatile_time", quantile_time_);
    this->get_parameter("almost_goal_time", almosts_time_);

    std::string output_msg =
      "quantile_time is " + std::to_string(quantile_time_) + ". " +
      "and almost_goal_time is " + std::to_string(almosts_time_) + ".";

    RCLCPP_INFO(this->get_logger(), "Dist turtle action server is started.");
    RCLCPP_INFO(this->get_logger(), "%s", output_msg.c_str());

    parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&DistTurtleServer::parameter_callback, this, std::placeholders::_1)
    );
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
  rclcpp_action::Server<DistTurtle>::SharedPtr action_server_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  turtlesim::msg::Pose current_pose_;
  turtlesim::msg::Pose previous_pose_;

  double total_dist_;
  bool is_first_time_;
  double quantile_time_;
  double almosts_time_;

  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    current_pose_ = *msg;
  }

  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & params)
  {
    for (const auto & param : params) {
      std::cout << param.get_name() << " is changed to " << param.value_to_string() << std::endl;

      if (param.get_name() == "quatile_time") {
        quantile_time_ = param.as_double();
      }
      if (param.get_name() == "almost_goal_time") {
        almosts_time_ = param.as_double();
      }
    }

    std::string output_msg =
      "quantile_time is " + std::to_string(quantile_time_) + ". " +
      "and almost_goal_time is " + std::to_string(almosts_time_) + ".";

    RCLCPP_INFO(this->get_logger(), "%s", output_msg.c_str());

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  double calc_diff_pose()
  {
    if (is_first_time_) {
      previous_pose_.x = current_pose_.x;
      previous_pose_.y = current_pose_.y;
      is_first_time_ = false;
    }

    double diff_dist = std::sqrt(
      std::pow(current_pose_.x - previous_pose_.x, 2.0) +
      std::pow(current_pose_.y - previous_pose_.y, 2.0)
    );

    previous_pose_ = current_pose_;
    return diff_dist;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const DistTurtle::Goal> goal)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Received goal: dist=%.3f, linear_x=%.3f, angular_z=%.3f",
      goal->dist, goal->linear_x, goal->angular_z
    );
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleDistTurtle>)
  {
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleDistTurtle> goal_handle)
  {
    std::thread{std::bind(&DistTurtleServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleDistTurtle> goal_handle)
  {
    auto feedback = std::make_shared<DistTurtle::Feedback>();
    auto result = std::make_shared<DistTurtle::Result>();

    geometry_msgs::msg::Twist msg;
    msg.linear.x = goal_handle->get_goal()->linear_x;
    msg.angular.z = goal_handle->get_goal()->angular_z;

    rclcpp::WallRate loop_rate(100);

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        geometry_msgs::msg::Twist stop_msg;
        publisher_->publish(stop_msg);

        result->pos_x = current_pose_.x;
        result->pos_y = current_pose_.y;
        result->pos_theta = current_pose_.theta;
        result->result_dist = total_dist_;

        total_dist_ = 0.0;
        is_first_time_ = true;

        goal_handle->canceled(result);
        return;
      }

      total_dist_ += calc_diff_pose();
      feedback->remained_dist = goal_handle->get_goal()->dist - total_dist_;
      goal_handle->publish_feedback(feedback);
      publisher_->publish(msg);

      double tmp = feedback->remained_dist - goal_handle->get_goal()->dist * quantile_time_;
      tmp = std::abs(tmp);

      if (tmp < 0.02) {
        std::string output_msg =
          "The turtle passes the " + std::to_string(quantile_time_) +
          " point.  : " + std::to_string(tmp);
        RCLCPP_INFO(this->get_logger(), "%s", output_msg.c_str());
      }

      if (feedback->remained_dist < 0.2) {
        break;
      }

      loop_rate.sleep();
    }

    geometry_msgs::msg::Twist stop_msg;
    publisher_->publish(stop_msg);

    result->pos_x = current_pose_.x;
    result->pos_y = current_pose_.y;
    result->pos_theta = current_pose_.theta;
    result->result_dist = total_dist_;

    total_dist_ = 0.0;
    is_first_time_ = true;

    goal_handle->succeed(result);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DistTurtleServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}