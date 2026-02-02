#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class PidPathFollower : public rclcpp::Node
{
public:
    PidPathFollower() : rclcpp::Node("pid_path_follower"),
                        tf_buffer_(this->get_clock()),
                        tf_listener_(tf_buffer_)
    {
        max_linear_vel_ = this->declare_parameter<double>("max_linear_vel", 0.5);
        max_angular_vel_ = this->declare_parameter<double>("max_angular_vel", 1.0);
        linear_kp_ = this->declare_parameter<double>("linear_kp", 0.5);
        linear_ki_ = this->declare_parameter<double>("linear_ki", 0.0);
        linear_kd_ = this->declare_parameter<double>("linear_kd", 0.1);
        angular_kp_ = this->declare_parameter<double>("angular_kp", 1.0);
        angular_ki_ = this->declare_parameter<double>("angular_ki", 0.0);
        angular_kd_ = this->declare_parameter<double>("angular_kd", 0.1);
        lookahead_distance_ = this->declare_parameter<double>("lookahead_distance", 0.5);
        goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.2);
        control_frequency_ = this->declare_parameter<double>("control_frequency", 20.0);

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/global_path", 10,
            std::bind(&PidPathFollower::pathCallback, this, std::placeholders::_1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_)),
            std::bind(&PidPathFollower::controlLoop, this));

        linear_integral_ = 0.0;
        angular_integral_ = 0.0;
        linear_last_error_ = 0.0;
        angular_last_error_ = 0.0;
        current_path_index_ = 0;
        path_received_ = false;
        max_integral_ = 1.0;

        RCLCPP_INFO(this->get_logger(), "PID Path Follower initialized");
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty path");
            return;
        }
        current_path_ = *msg;
        current_path_index_ = 0;
        path_received_ = true;
        linear_integral_ = 0.0;
        angular_integral_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "Received new path with %zu waypoints", msg->poses.size());
    }

    bool getRobotPose(geometry_msgs::msg::Pose& pose)
    {
        try {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
                "map", "base_link", tf2::TimePointZero);
            pose.position.x = transform.transform.translation.x;
            pose.position.y = transform.transform.translation.y;
            pose.position.z = transform.transform.translation.z;
            pose.orientation = transform.transform.rotation;
            return true;
        } catch (const tf2::TransformException& ex) {
            return false;
        }
    }

    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
    {
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    int findClosestWaypoint(const geometry_msgs::msg::Pose& robot_pose)
    {
        if (current_path_.poses.empty()) return 0;

        double min_dist = std::numeric_limits<double>::max();
        int closest_idx = current_path_index_;

        for (size_t i = current_path_index_; i < current_path_.poses.size(); ++i) {
            double dx = current_path_.poses[i].pose.position.x - robot_pose.position.x;
            double dy = current_path_.poses[i].pose.position.y - robot_pose.position.y;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }

        return closest_idx;
    }

    int findLookaheadWaypoint(const geometry_msgs::msg::Pose& robot_pose, int start_idx)
    {
        if (current_path_.poses.empty()) return start_idx;

        for (size_t i = start_idx; i < current_path_.poses.size(); ++i) {
            double dx = current_path_.poses[i].pose.position.x - robot_pose.position.x;
            double dy = current_path_.poses[i].pose.position.y - robot_pose.position.y;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist >= lookahead_distance_) {
                return i;
            }
        }

        return current_path_.poses.size() - 1;
    }

    void controlLoop()
    {
        if (!path_received_ || current_path_.poses.empty()) {
            return;
        }

        geometry_msgs::msg::Pose robot_pose;
        if (!getRobotPose(robot_pose)) {
            return;
        }

        current_path_index_ = findClosestWaypoint(robot_pose);
        int target_idx = findLookaheadWaypoint(robot_pose, current_path_index_);

        if (target_idx >= static_cast<int>(current_path_.poses.size())) {
            target_idx = current_path_.poses.size() - 1;
        }

        geometry_msgs::msg::Pose target_pose = current_path_.poses[target_idx].pose;

        double dx = target_pose.position.x - robot_pose.position.x;
        double dy = target_pose.position.y - robot_pose.position.y;
        double distance_error = std::sqrt(dx * dx + dy * dy);

        double target_yaw = std::atan2(dy, dx);
        double robot_yaw = getYawFromQuaternion(robot_pose.orientation);
        double angular_error = target_yaw - robot_yaw;

        while (angular_error > M_PI) angular_error -= 2.0 * M_PI;
        while (angular_error < -M_PI) angular_error += 2.0 * M_PI;

        if (target_idx == static_cast<int>(current_path_.poses.size() - 1) && distance_error < goal_tolerance_) {
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd);
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            path_received_ = false;
            return;
        }

        double dt = 1.0 / control_frequency_;

        linear_integral_ += distance_error * dt;
        linear_integral_ = std::clamp(linear_integral_, -max_integral_, max_integral_);
        double linear_derivative = (distance_error - linear_last_error_) / dt;
        double linear_output = linear_kp_ * distance_error + linear_ki_ * linear_integral_ + linear_kd_ * linear_derivative;
        linear_last_error_ = distance_error;

        angular_integral_ += angular_error * dt;
        angular_integral_ = std::clamp(angular_integral_, -max_integral_, max_integral_);
        double angular_derivative = (angular_error - angular_last_error_) / dt;
        double angular_output = angular_kp_ * angular_error + angular_ki_ * angular_integral_ + angular_kd_ * angular_derivative;
        angular_last_error_ = angular_error;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = std::clamp(linear_output, -max_linear_vel_, max_linear_vel_);
        cmd.angular.z = std::clamp(angular_output, -max_angular_vel_, max_angular_vel_);

        cmd_vel_pub_->publish(cmd);
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    nav_msgs::msg::Path current_path_;
    int current_path_index_;
    bool path_received_;

    double max_linear_vel_;
    double max_angular_vel_;
    double linear_kp_, linear_ki_, linear_kd_;
    double angular_kp_, angular_ki_, angular_kd_;
    double lookahead_distance_;
    double goal_tolerance_;
    double control_frequency_;

    double linear_integral_;
    double angular_integral_;
    double linear_last_error_;
    double angular_last_error_;
    double max_integral_;
    bool use_sim_time_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PidPathFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
