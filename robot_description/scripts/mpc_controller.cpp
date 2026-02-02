#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <vector>

class MpcController : public rclcpp::Node {
public:
    MpcController() : Node("mpc_controller"), tf_buf_(this->get_clock()), tf_lst_(tf_buf_) {
        horizon_steps_ = declare_parameter("horizon", 6);
        v_max_ = declare_parameter("v_max", 0.4);
        w_max_ = declare_parameter("w_max", 0.8);

        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/global_path", 10, std::bind(&MpcController::on_path, this, std::placeholders::_1));
        
        vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&MpcController::solve, this));
    }

private:
    void on_path(const nav_msgs::msg::Path::SharedPtr m) {
        path_ = *m;
        idx_ = 0;
        active_ = true;
    }

    double get_yaw(const geometry_msgs::msg::Quaternion& q) {
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    void solve() {
        if (!active_ || path_.poses.empty()) return;

        geometry_msgs::msg::Pose p;
        try {
            auto t = tf_buf_.lookupTransform("map", "base_link", tf2::TimePointZero);
            p.position.x = t.transform.translation.x;
            p.position.y = t.transform.translation.y;
            p.orientation = t.transform.rotation;
        } catch (...) { return; }

        double min_d = std::numeric_limits<double>::max();
        for (size_t i = idx_; i < path_.poses.size(); ++i) {
            double d = std::hypot(path_.poses[i].pose.position.x - p.position.x,
                                  path_.poses[i].pose.position.y - p.position.y);
            if (d < min_d) {
                min_d = d;
                idx_ = i;
            }
        }

        int target_idx = std::min((int)idx_ + horizon_steps_, (int)path_.poses.size() - 1);
        auto target = path_.poses[target_idx].pose;

        double dx = target.position.x - p.position.x;
        double dy = target.position.y - p.position.y;
        double dist = std::hypot(dx, dy);

        double ty = std::atan2(dy, dx);
        double ry = get_yaw(p.orientation);
        double err_w = ty - ry;
        while (err_w > M_PI) err_w -= 2 * M_PI;
        while (err_w < -M_PI) err_w += 2 * M_PI;

        if (idx_ == path_.poses.size() - 1 && dist < 0.15) {
            stop();
            active_ = false;
            return;
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = std::clamp(dist * 0.5, 0.0, v_max_);
        cmd.angular.z = std::clamp(err_w * 1.8, -w_max_, w_max_);
        vel_pub_->publish(cmd);
    }

    void stop() {
        geometry_msgs::msg::Twist msg;
        vel_pub_->publish(msg);
    }

    nav_msgs::msg::Path path_;
    size_t idx_ = 0;
    bool active_ = false;
    int horizon_steps_;
    double v_max_, w_max_;

    tf2_ros::Buffer tf_buf_;
    tf2_ros::TransformListener tf_lst_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MpcController>());
    rclcpp::shutdown();
    return 0;
}