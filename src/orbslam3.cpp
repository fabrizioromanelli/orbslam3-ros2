#include <functional>
#include <queue>
#include <thread>
#include <mutex>
#include <memory>
#include <chrono>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <ORB_SLAM3/System.h>

#include "rclcpp/rclcpp.hpp"

#define MAX_IMU_BUFFER 7

using std::placeholders::_1;

class ORBSLAM3Subscriber : public rclcpp::Node
{
public:

  ORBSLAM3Subscriber(ORB_SLAM3::System* pSLAM, ORB_SLAM3::System::eSensor _sensorType)
  : Node("orbslam3_to_realsense_subscriber"), mpSLAM(pSLAM), sensorType(_sensorType)
  {
    rclcpp::QoS video_qos(10);
    video_qos.keep_last(10);
    video_qos.best_effort();
    video_qos.durability_volatile();

    if (this->sensorType == ORB_SLAM3::System::IMU_STEREO || this->sensorType == ORB_SLAM3::System::IMU_RGBD)
    {
      imuSubscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/camera/imu", 1000, std::bind(&ORBSLAM3Subscriber::imu_callback, this, _1)
      );
    }

    irLeftSubscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/infra1/image_rect_raw", video_qos, std::bind(&ORBSLAM3Subscriber::irLeft_callback, this, _1)
    );

    if (this->sensorType == ORB_SLAM3::System::STEREO || this->sensorType == ORB_SLAM3::System::IMU_STEREO)
    {
      irRightSubscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/infra2/image_rect_raw", video_qos, std::bind(&ORBSLAM3Subscriber::irRight_callback, this, _1)
      );
    }

    if (this->sensorType == ORB_SLAM3::System::RGBD || this->sensorType == ORB_SLAM3::System::IMU_RGBD)
    {
      depthSubscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/aligned_depth_to_infra1/image_raw", video_qos, std::bind(&ORBSLAM3Subscriber::depth_callback, this, _1)
      );
    }
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
    std::cout << "ir" << std::endl;
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

  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock{mBufMutexDepth};
    std::cout << "depth" << std::endl;
    if (!imgDepthBuf.empty())
      imgDepthBuf.pop();
    imgDepthBuf.push(msg);
  }

  void imuSLAM();
  void stereoSLAM();
  void rgbdSLAM();

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr irLeftSubscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr irRightSubscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthSubscription_;

  // Queues
  std::queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf;
  std::queue<sensor_msgs::msg::Image::SharedPtr> imgLeftBuf, imgRightBuf, imgDepthBuf;

  // Mutex
  std::mutex mBufMutex;
  std::mutex mBufMutexLeft, mBufMutexRight, mBufMutexDepth;

  ORB_SLAM3::System* mpSLAM;
  ORB_SLAM3::System::eSensor sensorType;
};

void ORBSLAM3Subscriber::runSLAM()
{
  if (this->sensorType == ORB_SLAM3::System::IMU_STEREO || this->sensorType == ORB_SLAM3::System::IMU_RGBD)
    imuSLAM();
  else if (this->sensorType == ORB_SLAM3::System::STEREO)
    stereoSLAM();
  else if (this->sensorType == ORB_SLAM3::System::RGBD)
    rgbdSLAM();
}

void ORBSLAM3Subscriber::rgbdSLAM()
{
  std::cout << "a" << std::endl;
  const rclcpp::Duration maxTimeDiff(0, 10000000); // 0.01s
  while(true)
  {
    std::cout << "b" << std::endl;
    rclcpp::Time tImLeft(0), tImDepth(0);
    if (!imgLeftBuf.empty() && !imgDepthBuf.empty())
    {
      std::cout << "c" << std::endl;
      tImLeft = imgLeftBuf.front()->header.stamp;
      tImDepth = imgDepthBuf.front()->header.stamp;

      std::unique_lock<std::mutex> lockD{mBufMutexDepth};
      while ((tImLeft-tImDepth) > maxTimeDiff && imgDepthBuf.size() > 1)
      {
        imgDepthBuf.pop();
        tImDepth = imgDepthBuf.front()->header.stamp;
      }
      lockD.unlock();
std::cout << "d" << std::endl;
      std::unique_lock<std::mutex> lockL{mBufMutexLeft};
      while ((tImDepth-tImLeft) > maxTimeDiff && imgLeftBuf.size() > 1)
      {
        imgLeftBuf.pop();
        tImLeft = imgLeftBuf.front()->header.stamp;
      }
      lockL.unlock();
std::cout << "e" << std::endl;
      if ((tImLeft-tImDepth) > maxTimeDiff || (tImDepth-tImLeft) > maxTimeDiff)
        continue;
std::cout << "f" << std::endl;
      if (tImLeft > imuBuf.back()->header.stamp)
        continue;
std::cout << "g" << std::endl;
      lockL.lock();
      cv_bridge::CvImageConstPtr cv_ptrLeft;
      try
      {
          cv_ptrLeft = cv_bridge::toCvShare(imgLeftBuf.front());
      }
      catch (cv_bridge::Exception& e)
      {
          RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
          return;
      }
      imgLeftBuf.pop();
      lockL.unlock();
std::cout << "h" << std::endl;
      lockD.lock();
      cv_bridge::CvImageConstPtr cv_ptrRight;
      try
      {
          cv_ptrRight = cv_bridge::toCvShare(imgDepthBuf.front());
      }
      catch (cv_bridge::Exception& e)
      {
          RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
          return;
      }
      imgDepthBuf.pop();
      lockD.unlock();
std::cout << "i" << std::endl;
      mpSLAM->TrackRGBD(cv_ptrLeft->image, cv_ptrRight->image, tImLeft.seconds());
    }

    std::chrono::milliseconds tSleep(1);
    std::this_thread::sleep_for(tSleep);
  }
}

void ORBSLAM3Subscriber::stereoSLAM()
{
  const rclcpp::Duration maxTimeDiff(0, 10000000); // 0.01s
  while(true)
  {
    rclcpp::Time tImLeft(0), tImRight(0);
    if (!imgLeftBuf.empty() && !imgRightBuf.empty())
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
        continue;

      if (tImLeft > imuBuf.back()->header.stamp)
        continue;

      lockL.lock();
      cv_bridge::CvImageConstPtr cv_ptrLeft;
      try
      {
          cv_ptrLeft = cv_bridge::toCvShare(imgLeftBuf.front());
      }
      catch (cv_bridge::Exception& e)
      {
          RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
          return;
      }
      imgLeftBuf.pop();
      lockL.unlock();

      lockR.lock();
      cv_bridge::CvImageConstPtr cv_ptrRight;
      try
      {
          cv_ptrRight = cv_bridge::toCvShare(imgRightBuf.front());
      }
      catch (cv_bridge::Exception& e)
      {
          RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
          return;
      }
      imgRightBuf.pop();
      lockR.unlock();

      mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, tImLeft.seconds());
    }
  }
}

void ORBSLAM3Subscriber::imuSLAM()
{
  const rclcpp::Duration maxTimeDiff(0, 10000000); // 0.01s
  while(true)
  {
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
      cv_bridge::CvImageConstPtr cv_ptrLeft;
      try
      {
          cv_ptrLeft = cv_bridge::toCvShare(imgLeftBuf.front());
      }
      catch (cv_bridge::Exception& e)
      {
          RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
          return;
      }
      imgLeftBuf.pop();
      lockL.unlock();

      lockR.lock();
      cv_bridge::CvImageConstPtr cv_ptrRight;
      try
      {
          cv_ptrRight = cv_bridge::toCvShare(imgRightBuf.front());
      }
      catch (cv_bridge::Exception& e)
      {
          RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
          return;
      }
      imgRightBuf.pop();
      lockR.unlock();

      std::vector<std::vector<double>> tempImuBuf;
      std::vector<std::vector<double>>::iterator it;

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      std::unique_lock<std::mutex> lock{mBufMutex};
      if (!imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while (!imuBuf.empty() && tImLeft >= imuBuf.front()->header.stamp)
        {
          rclcpp::Time t = imuBuf.front()->header.stamp;
          cv::Point3f acc(imuBuf.front()->linear_acceleration.x, imuBuf.front()->linear_acceleration.y, imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(imuBuf.front()->angular_velocity.x, imuBuf.front()->angular_velocity.y, imuBuf.front()->angular_velocity.z);
          tempImuBuf.push_back(std::vector<double>{t.seconds(),imuBuf.front()->linear_acceleration.x, imuBuf.front()->linear_acceleration.y, imuBuf.front()->linear_acceleration.z,imuBuf.front()->angular_velocity.x, imuBuf.front()->angular_velocity.y, imuBuf.front()->angular_velocity.z});
          imuBuf.pop();
        }

        if (tempImuBuf.size() > MAX_IMU_BUFFER)
        {
          int i = tempImuBuf.size() - MAX_IMU_BUFFER;
          for (it = tempImuBuf.begin(); it != tempImuBuf.end(), i > 0; i--)
            tempImuBuf.erase(it);
        }

        for (it = tempImuBuf.begin(); it != tempImuBuf.end(); it++)
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(cv::Point3f(it[0][1], it[0][2], it[0][3]), cv::Point3f(it[0][4], it[0][5], it[0][6]), it[0][0]));
      }
      lock.unlock();

      if (this->sensorType == ORB_SLAM3::System::IMU_STEREO)
        mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, tImLeft.seconds(), vImuMeas);
      else if (this->sensorType == ORB_SLAM3::System::IMU_RGBD) // Not implemented
        mpSLAM->TrackRGBD(cv_ptrLeft->image, cv_ptrRight->image, tImLeft.seconds(), vImuMeas);
    }
  }
}

enum string_code {
    eStereo,
    eStereoInertial,
    eRGBD,
    eRGBDInertial
};

string_code hashit (std::string const& inString) {
    if (inString == "STEREO") return eStereo;
    if (inString == "STEREO-INERTIAL") return eStereoInertial;
    if (inString == "RGBD") return eRGBD;
    if (inString == "RGBD-INERTIAL") return eRGBDInertial;
    return eStereo;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  ORB_SLAM3::System::eSensor sensorType;
  switch (hashit(argv[3]))
  {
    case eStereo:
      sensorType = ORB_SLAM3::System::STEREO;
      break;
    case eStereoInertial:
      sensorType = ORB_SLAM3::System::IMU_STEREO;
      break;
    case eRGBD:
      sensorType = ORB_SLAM3::System::RGBD;
      break;
    case eRGBDInertial:
      sensorType = ORB_SLAM3::System::IMU_RGBD;
      break;
    default:
      break;
  }

  ORB_SLAM3::System SLAM(argv[1], argv[2], sensorType, true);
  auto test = std::make_shared<ORBSLAM3Subscriber>(&SLAM, sensorType);
  // RCLCPP_INFO_STREAM(test->get_logger(), "pose is " << argv[1] << argv[2] << argv[3] << argv[4] << argv[5] << argv[6]);
  std::thread sync_thread(&ORBSLAM3Subscriber::runSLAM,&(*test));
  rclcpp::spin(test);
  rclcpp::shutdown();
  return 0;
}
