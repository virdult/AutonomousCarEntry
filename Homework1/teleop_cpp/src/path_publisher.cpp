#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "turtlesim/msg/pose.hpp"
#include "std_msgs/msg/bool.hpp"   // For autonomous mode toggle
#include <cmath>

class PathPublisher : public rclcpp::Node
{
public:
    PathPublisher()
        : Node("path_publisher"), has_pose_(false), center_locked_(false)
    {
        // --- Publisher for the circular path ---
        pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

        // --- Subscribe to the turtle's current pose ---
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&PathPublisher::poseCallback, this, std::placeholders::_1));

        // --- Subscribe to autonomous mode changes ---
        mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "autonomous_mode", 10,
            std::bind(&PathPublisher::modeCallback, this, std::placeholders::_1));

        // --- Timer to republish path every 5 seconds ---
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&PathPublisher::publishPath, this));

        RCLCPP_INFO(this->get_logger(), "Circular path publisher started!");
    }

private:
    // --- Pose callback: just keeps track of latest turtle position ---
    void poseCallback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_x_ = msg->x;
        current_y_ = msg->y;
        has_pose_ = true;
    }

    // --- Mode callback: sets center every time autonomous mode turns ON ---
    void modeCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) // Autonomous mode ON
        {
            if (has_pose_)
            {
                center_x_ = current_x_;
                center_y_ = current_y_;
                center_locked_ = true;

                RCLCPP_INFO(this->get_logger(),
                            "Autonomous mode ON — locked new circle center at (%.2f, %.2f)",
                            center_x_, center_y_);

                publishPath(); // Immediately publish new path
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Cannot set center — no pose received yet!");
            }
        }
    }

    /*
    // --- Publishes a circular path around the current center ---
    void publishPath()
    {
        if (!has_pose_ || !center_locked_)
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for initial pose to lock circle center...");
            return;
        }

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";

        // --- Define circular path ---
        double radius = 2.0;  // Circle radius
        int num_points = 100; // Smoothness

        for (int i = 0; i <= num_points; i++)
        {
            double angle = 2.0 * M_PI * i / num_points;

            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;

            pose.pose.position.x = center_x_ + radius * std::cos(angle);
            pose.pose.position.y = center_y_ + radius * std::sin(angle);

            // Orientation tangent to circle
            double yaw = angle + M_PI_2;
            pose.pose.orientation.z = std::sin(yaw / 2.0);
            pose.pose.orientation.w = std::cos(yaw / 2.0);

            path_msg.poses.push_back(pose);
        }

        pub_->publish(path_msg);

        RCLCPP_INFO(this->get_logger(),
                    "Published circular path (r=%.2f, %d points) centered at (%.2f, %.2f)",
                    radius, num_points, center_x_, center_y_);
    }
    */

    void publishPath()
    {
        if (!has_pose_ || !center_locked_)
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for initial pose to lock square center...");
            return;
        }

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";

        // --- Define circular path ---
        double radius = 2.0;  // Circle radius
        int num_points = 4; // Smoothness

        for (int i = 0; i <= num_points; i++)
        {
            double angle = 2.0 * M_PI * i / num_points;

            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;

            pose.pose.position.x = center_x_ + radius * std::cos(angle);
            pose.pose.position.y = center_y_ + radius * std::sin(angle);

            // Orientation tangent to circle
            double yaw = angle + M_PI_2;
            pose.pose.orientation.z = std::sin(yaw / 2.0);
            pose.pose.orientation.w = std::cos(yaw / 2.0);

            path_msg.poses.push_back(pose);
        }

        pub_->publish(path_msg);

        RCLCPP_INFO(this->get_logger(),
                    "Published circular path (r=%.2f, %d points) centered at (%.2f, %.2f)",
                    radius, num_points, center_x_, center_y_);
    }

    // --- ROS interfaces ---
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --- Internal state ---
    double current_x_, current_y_;
    double center_x_, center_y_;
    bool has_pose_;
    bool center_locked_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPublisher>());
    rclcpp::shutdown();
    return 0;
}
