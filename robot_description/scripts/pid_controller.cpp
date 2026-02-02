#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class PidController : public rclcpp::Node {
public:
    PidController() : Node("pid_controller"), tf_buf_(this->get_clock()), tf_lst_(tf_buf_) {
        v_max_ = declare_parameter("v_max", 0.4);
        w_max_ = declare_parameter("w_max", 0.8);
        kp_v = declare_parameter("kp_v", 0.6);
        kd_v = declare_parameter("kd_v", 0.1);
        kp_w = declare_parameter("kp_w", 1.2);
        kd_w = declare_parameter("kd_w", 0.1);
        l_dist_ = declare_parameter("lookahead", 0.4);
        tol_ = declare_parameter("tolerance", 0.15);

        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/global_path", 10, std::bind(&PidController::on_path, this, std::placeholders::_1));
        
        vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        loop_timer_ = create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&PidController::step, this));
    }

private:
    struct PIDState {
        double prev_err = 0;
        double integral = 0;
        double compute(double err, double kp, double ki, double kd, double dt) {
            integral = std::clamp(integral + err * dt, -0.5, 0.5);
            double der = (err - prev_err) / dt;
            prev_err = err;
            return kp * err + ki * integral + kd * der;
        }
    };

    void on_path(const nav_msgs::msg::Path::SharedPtr m) {
        active_path_ = *m;
        idx_ = 0;
        active_ = true;
    }
    double get_yaw(const geometry_msgs::msg::Quaternion& q)
    {
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    void step() {
        if (!active_ || active_path_.poses.empty()) return;

        geometry_msgs::msg::Pose p;
        try {
            auto t = tf_buf_.lookupTransform("map", "base_link", tf2::TimePointZero);
            p.position.x = t.transform.translation.x;
            p.position.y = t.transform.translation.y;
            p.orientation = t.transform.rotation;
        } catch (...) { return; }

        while (idx_ < active_path_.poses.size() - 1) {
            double d = std::hypot(active_path_.poses[idx_].pose.position.x - p.position.x,
                                  active_path_.poses[idx_].pose.position.y - p.position.y);
            if (d < l_dist_) idx_++; else break;
        }

        auto target = active_path_.poses[idx_].pose;
        double dx = target.position.x - p.position.x;
        double dy = target.position.y - p.position.y;
        double err_v = std::hypot(dx, dy);

        double ty = std::atan2(dy, dx);
        double ry = get_yaw(p.orientation);
        double err_w = ty - ry;
        while (err_w > M_PI) err_w -= 2 * M_PI;
        while (err_w < -M_PI) err_w += 2 * M_PI;

        if (idx_ == active_path_.poses.size() - 1 && err_v < tol_) {
            stop();
            active_ = false;
            return;
        }

        double dt = 0.05;
        double out_v = pid_v.compute(err_v, kp_v, 0.0, kd_v, dt);
        double out_w = pid_w.compute(err_w, kp_w, 0.0, kd_w, dt);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = std::clamp(out_v, -v_max_, v_max_);
        cmd.angular.z = std::clamp(out_w, -w_max_, w_max_);
        vel_pub_->publish(cmd);
    }

    void stop() {
        geometry_msgs::msg::Twist msg;
        vel_pub_->publish(msg);
    }

    PIDState pid_v, pid_w;
    nav_msgs::msg::Path active_path_;
    size_t idx_ = 0;
    bool active_ = false;
    double v_max_, w_max_, kp_v, kd_v, kp_w, kd_w, l_dist_, tol_;
    
    tf2_ros::Buffer tf_buf_;
    tf2_ros::TransformListener tf_lst_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr loop_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PidController>());
    rclcpp::shutdown();
    return 0;
}