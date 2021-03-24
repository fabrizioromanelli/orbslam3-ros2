#include <functional>
#include <queue>
#include <thread>
#include <mutex>
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
      "/camera/imu", 1000, std::bind(&ORBSLAM3Subscriber::imu_callback, this, _1)
    );
    irLeftSubscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/infra1/image_rect_raw", video_qos, std::bind(&ORBSLAM3Subscriber::irLeft_callback, this, _1)
    );
    irRightSubscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/infra2/image_rect_raw", video_qos, std::bind(&ORBSLAM3Subscriber::irRight_callback, this, _1)
    );
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    mBufMutex.lock();
    // imuBuf.push(msg);
    std::cout << msg->header.stamp.sec << msg->header.stamp.nanosec << " " << msg->linear_acceleration.x << " " << msg->linear_acceleration.y << " " << msg->linear_acceleration.z << " " << msg->angular_velocity.x << " " << msg->angular_velocity.y << " " << msg->angular_velocity.z << std::endl;
    mBufMutex.unlock();
    return;
  }

  void irLeft_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::cout << "L " << msg->header.stamp.sec << msg->header.stamp.nanosec << " " << msg->width << " " << msg->height << std::endl;
  }

  void irRight_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::cout << "R " << msg->header.stamp.sec << msg->header.stamp.nanosec << " " << msg->width << " " << msg->height << std::endl;
  }

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr irLeftSubscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr irRightSubscription_;

  // Queues
  std::queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf;
  std::queue<sensor_msgs::msg::Image::SharedPtr> imgLeftBuf, imgRightBuf;

  // Mutex
  std::mutex mBufMutex;
  std::mutex mBufMutexLeft,mBufMutexRight;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ORBSLAM3Subscriber>());
  rclcpp::shutdown();
  return 0;
}
