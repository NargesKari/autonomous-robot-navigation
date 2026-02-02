#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserScanFrameIdConverter : public rclcpp::Node
{
public:
  LaserScanFrameIdConverter()
  : Node("frame_id_converter_node")
  {
    // Publisher: republish LaserScan with corrected frame_id
    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::QoS(rclcpp::SensorDataQoS())   // BestEffort QoS for sensor data
    );

    // Subscriber: listen to Gazebo laser scan
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/gz_lidar/scan",
      rclcpp::QoS(rclcpp::SensorDataQoS()),
      std::bind(&LaserScanFrameIdConverter::laserScanCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(),
                "LaserScan FrameIdConverter started. Listening to /gz_lidar/scan...");
  }

private:
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    auto new_msg = *msg;                     // Copy original message
    new_msg.header.frame_id = "rplidar_c1";  // Set correct TF frame
    pub_->publish(new_msg);                  // Publish updated scan
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanFrameIdConverter>());
  rclcpp::shutdown();
  return 0;
}
