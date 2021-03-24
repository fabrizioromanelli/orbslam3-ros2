#include <functional>
#include <memory>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class ORBSLAM3Subscriber : public rclcpp::Node
{
public:
  ORBSLAM3Subscriber()
  : Node("orbslam3_to_realsense_subscriber")
  {
    rclcpp::QoS video_qos(10);
    video_qos.keep_last(10);
    video_qos.best_effort();
    video_qos.durability_volatile();

    imuSubscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/camera/imu", 1000, std::bind(&ORBSLAM3Subscriber::imu_callback, this, _1));
    irLeftSubscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/infra1/image_rect_raw", video_qos, std::bind(&ORBSLAM3Subscriber::irLeft_callback, this, _1)
    );
    irRightSubscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/infra2/image_rect_raw", video_qos, std::bind(&ORBSLAM3Subscriber::irRight_callback, this, _1)
    );
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
  {
    std::cout << msg->header.stamp.sec << msg->header.stamp.nanosec << " " << msg->linear_acceleration.x << " " << msg->linear_acceleration.y << " " << msg->linear_acceleration.z << " " << msg->angular_velocity.x << " " << msg->angular_velocity.y << " " << msg->angular_velocity.z << std::endl;
  }

  void irLeft_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    std::cout << "L " << msg->header.stamp.sec << msg->header.stamp.nanosec << " " << msg->width << " " << msg->height << std::endl;
  }

  void irRight_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    std::cout << "R " << msg->header.stamp.sec << msg->header.stamp.nanosec << " " << msg->width << " " << msg->height << std::endl;
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr irLeftSubscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr irRightSubscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ORBSLAM3Subscriber>());
  rclcpp::shutdown();
  return 0;
}
