#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "turtlesim/msg/pose.hpp"
#include "std_msgs/msg/bool.hpp"

#include <cmath>
#include <vector>
#include <algorithm>

class StanleyController : public rclcpp::Node
{
public:
    StanleyController() : Node("stanley_controller"), autonomous_mode_(false), last_target_idx_(0)
    {
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10, std::bind(&StanleyController::pathCallback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&StanleyController::poseCallback, this, std::placeholders::_1));

        mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "autonomous_mode", 10, std::bind(&StanleyController::modeCallback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Stanley controller started (continuous mode)");
    }

private:
    // --- ROS interfaces ---
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    // --- Internal state ---
    std::vector<geometry_msgs::msg::PoseStamped> path_;
    turtlesim::msg::Pose current_pose_;
    bool autonomous_mode_;
    int last_target_idx_;

    // --- Parameters ---
    double k_gain_ = 1.0;      // Cross-track gain
    double target_speed_ = 1.5;
    double lookahead_distance_ = 0.3; // For smoother transitions

    // --- Callbacks ---
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        path_ = msg->poses;
        RCLCPP_INFO(this->get_logger(), "Received path with %zu waypoints", path_.size());
    }

    void modeCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        autonomous_mode_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Autonomous mode: %s", autonomous_mode_ ? "ON" : "OFF");
    }

    void poseCallback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_pose_ = *msg;

        if (!autonomous_mode_ || path_.empty())
            return;

        // --- Find the closest waypoint ---
        double min_dist = 1e9;
        int closest_idx = 0;
        for (size_t i = 0; i < path_.size(); i++)
        {
            double dx = path_[i].pose.position.x - current_pose_.x;
            double dy = path_[i].pose.position.y - current_pose_.y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_idx = i;
            }
        }

        // --- Look ahead a few points for smoothness ---
        int lookahead_idx = (closest_idx + 5) % path_.size();
        last_target_idx_ = lookahead_idx;

        auto target = path_[lookahead_idx];

        // --- Compute path yaw ---
        double path_yaw = 2.0 * std::atan2(target.pose.orientation.z, target.pose.orientation.w);

        // --- Compute heading and cross-track errors ---
        double heading_error = path_yaw - current_pose_.theta;
        heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error));

        double dx = target.pose.position.x - current_pose_.x;
        double dy = target.pose.position.y - current_pose_.y;
        double cte = dy * std::cos(current_pose_.theta) - dx * std::sin(current_pose_.theta);

        // --- Stanley control law ---
        double steer = heading_error + std::atan2(k_gain_ * cte, target_speed_);

        // Clamp steer safely
        steer = std::clamp(steer, -2.0, 2.0);

        // --- Control command ---
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = std::max(1.5, target_speed_ * std::cos(heading_error)); // keep moving forward
        cmd.angular.z = steer;

        cmd_pub_->publish(cmd);

        // --- Debug ---
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "Pose(%.2f, %.2f, %.2f) | TargetIdx=%d | CTE=%.3f | HeadErr=%.3f | Steer=%.3f | v=%.2f",
                             current_pose_.x, current_pose_.y, current_pose_.theta,
                             lookahead_idx, cte, heading_error, steer, cmd.linear.x);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StanleyController>());
    rclcpp::shutdown();
    return 0;
}