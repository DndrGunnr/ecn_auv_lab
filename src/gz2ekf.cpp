#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

using namespace geometry_msgs::msg;
using namespace sensor_msgs::msg;

namespace Covariances
{

constexpr inline auto covarianceFrom(double cov)
{
  return cov * cov * 0.1;
}

constexpr auto eps{1e-5};
constexpr auto xyz{covarianceFrom(0.5)};
constexpr auto rpy{covarianceFrom(0.8 * M_PI / 180)};
constexpr auto w{covarianceFrom(0.08 * M_PI/180)};
constexpr auto a{covarianceFrom(0.002 * 9.81)};

}

struct ROVImu
{
  rclcpp::Publisher<Imu>::SharedPtr pub;
  rclcpp::Subscription<Imu>::SharedPtr sub;
  Imu imu;

  ROVImu(rclcpp::Node* node, const std::string &name)
      : pub{node->create_publisher<Imu>(name, 1)}
  {
    imu.header.frame_id = "bluerov2/" + name;

    for(auto i: {0, 4, 8})
    {
      imu.linear_acceleration_covariance[i] = Covariances::a;
      imu.angular_velocity_covariance[i] = Covariances::w;
      imu.orientation_covariance[i] = Covariances::rpy;
    }

    sub = node->create_subscription<Imu>(name+"_raw", 1, [&](Imu::UniquePtr msg)
                                         {fwdImu(*msg);});
  }

  inline void fwdImu(const Imu &msg)
  {
    imu.header.stamp = msg.header.stamp;
    imu.angular_velocity = msg.angular_velocity;
    imu.linear_acceleration = msg.linear_acceleration;
    imu.orientation = msg.orientation;
    pub->publish(imu);
  }
};

class Gz2Topics : public rclcpp::Node
{
public:
  Gz2Topics() : Node("gz2topics")
  {

    pose.header.frame_id = "world";

    if(use_pose)
    {
      for(auto i: {0, 7, 14, 21, 28, 35})
        pose.pose.covariance[i] = Covariances::eps;
      // just republish ground truth pose for EKF
      pose_pub = create_publisher<PoseWithCovarianceStamped>("pose", 1);
    }
    else
    {

      static auto lsm{ROVImu(this, "lsm")};
      static auto mpu{ROVImu(this, "mpu")};


      for(auto i: {0, 7, 14})
        pose.pose.covariance[i] = Covariances::xyz;
      for(auto i: {21, 28, 35})
        pose.pose.covariance[i] = Covariances::rpy;
      pose_pub = create_publisher<PoseWithCovarianceStamped>("usbl", 1);
    }

    static auto pose_sub = create_subscription<Pose>("pose_gt", 1, [&](Pose::UniquePtr msg)
                                                     {fwdPose(*msg);});
  }

private:

  bool use_pose{declare_parameter("use_pose", false)};
  PoseWithCovarianceStamped pose;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pose_pub;

  inline void fwdPose(const Pose &msg)
  {
    pose.pose.pose = msg;
    pose.header.stamp = get_clock()->now();
    pose_pub->publish(pose);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Gz2Topics>());
  rclcpp::shutdown();
  return 0;
}
