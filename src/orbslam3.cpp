#include <functional>
#include <queue>
#include <thread>
#include <mutex>
#include <memory>
#include <chrono>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core/core.hpp>

#include "rclcpp/rclcpp.hpp"

#define MAX_IMU_BUFFER 7

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

  void runSLAM();

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> guard{mBufMutex};
    imuBuf.push(msg);
  }

  void irLeft_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock{mBufMutexLeft};
    if (!imgLeftBuf.empty())
      imgLeftBuf.pop();
    imgLeftBuf.push(msg);
  }

  void irRight_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock{mBufMutexRight};
    if (!imgRightBuf.empty())
      imgRightBuf.pop();
    imgRightBuf.push(msg);
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

void ORBSLAM3Subscriber::runSLAM()
{
  std::cout.precision(17);
  const rclcpp::Duration maxTimeDiff(0, 10000000); // 0.01s
  while(true)
  {
    cv::Mat imLeft, imRight;
    rclcpp::Time tImLeft(0), tImRight(0);
    if (!imgLeftBuf.empty() && !imgRightBuf.empty() && !imuBuf.empty())
    {
      tImLeft = imgLeftBuf.front()->header.stamp;
      tImRight = imgRightBuf.front()->header.stamp;

      std::unique_lock<std::mutex> lockR{mBufMutexRight};
      while ((tImLeft-tImRight) > maxTimeDiff && imgRightBuf.size() > 1)
      {
        imgRightBuf.pop();
        tImRight = imgRightBuf.front()->header.stamp;
      }
      lockR.unlock();

      std::unique_lock<std::mutex> lockL{mBufMutexLeft};
      while ((tImRight-tImLeft) > maxTimeDiff && imgLeftBuf.size() > 1)
      {
        imgLeftBuf.pop();
        tImLeft = imgLeftBuf.front()->header.stamp;
      }
      lockL.unlock();

      if ((tImLeft-tImRight) > maxTimeDiff || (tImRight-tImLeft) > maxTimeDiff)
      {
        continue;
      }

      if (tImLeft > imuBuf.back()->header.stamp)
        continue;

      lockL.lock();
      // imLeft = GetImage(imgLeftBuf.front());
      imgLeftBuf.pop();
      lockL.unlock();

      lockR.lock();
      // imRight = GetImage(imgRightBuf.front());
      imgRightBuf.pop();
      lockR.unlock();

      std::vector<std::vector<double>> tempImuBuf;
      std::vector<std::vector<double>>::iterator it;

      // vector<ORB_SLAM3::IMU::Point> vImuMeas;
      std::unique_lock<std::mutex> lock{mBufMutex};
      if (!imuBuf.empty())
      {
        // Load imu measurements from buffer
        // vImuMeas.clear();
        
        while (!imuBuf.empty() && tImLeft >= imuBuf.front()->header.stamp)
        {
          rclcpp::Time t = imuBuf.front()->header.stamp;
          cv::Point3f acc(imuBuf.front()->linear_acceleration.x, imuBuf.front()->linear_acceleration.y, imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(imuBuf.front()->angular_velocity.x, imuBuf.front()->angular_velocity.y, imuBuf.front()->angular_velocity.z);
          // vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
          tempImuBuf.push_back(std::vector<double>{t.seconds(),imuBuf.front()->linear_acceleration.x, imuBuf.front()->linear_acceleration.y, imuBuf.front()->linear_acceleration.z,imuBuf.front()->angular_velocity.x, imuBuf.front()->angular_velocity.y, imuBuf.front()->angular_velocity.z});
          imuBuf.pop();
        }

        if (tempImuBuf.size() > MAX_IMU_BUFFER)
        {
          int i = tempImuBuf.size() - MAX_IMU_BUFFER;
          for (it = tempImuBuf.begin(); it != tempImuBuf.end(), i > 0; i--)
          {
            tempImuBuf.erase(it);
          }
        }
      }

      lock.unlock();

      // mpSLAM->TrackStereo(imLeft,imRight,tImLeft,vImuMeas);

      std::chrono::milliseconds tSleep(1);
      std::this_thread::sleep_for(tSleep);
    }
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto test = std::make_shared<ORBSLAM3Subscriber>();
  std::thread sync_thread(&ORBSLAM3Subscriber::runSLAM,&(*test));
  rclcpp::spin(test);
  rclcpp::shutdown();
  return 0;
}
