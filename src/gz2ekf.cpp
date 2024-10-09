#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <filesystem>
#include <yaml-cpp/yaml.h>

using namespace geometry_msgs::msg;
using namespace sensor_msgs::msg;


struct Covariance
{
  double eps{1e-5};
  YAML::Node root;
  Covariance()
  {
    const std::string param_dir{AUV_PARAMS};
    const auto cov_file{param_dir + "noise.yaml"};
    if(!std::filesystem::exists(cov_file))
      throw std::invalid_argument("Noise file does not exist: " + cov_file);

    root = YAML::LoadFile(cov_file);
  }

  inline auto from(const std::string &key) const
  {
    const auto range{root[key].as<double>()};
    return range * range * 0.3;
  }
};


struct ROVImu
{
  rclcpp::Publisher<Imu>::SharedPtr pub;
  rclcpp::Subscription<Imu>::SharedPtr sub;
  Imu imu;
  rclcpp::Node* node;

  ROVImu(rclcpp::Node* node, const std::string &name, const std::array<double, 3> &cov)
    : pub{node->create_publisher<Imu>(name, 1)}, node{node}
  {
    imu.header.frame_id = "bluerov2/" + name;

    for(auto i: {0, 4, 8})
    {
      imu.linear_acceleration_covariance[i] = cov[2];
      imu.angular_velocity_covariance[i] = cov[1]*M_PI/180;
      imu.orientation_covariance[i] = cov[0]*M_PI/180;
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
    imu.header.stamp = node->get_clock()->now();
    pub->publish(imu);
  }
};

struct ROVPose
{
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub;
  PoseWithCovarianceStamped pose;
  std::array<double,3> cov;
  rclcpp::Node* node;

  ROVPose(rclcpp::Node* node, const std::string &topic, std::array<double,3> cov)
    : pub{node->create_publisher<PoseWithCovarianceStamped>(topic, 1)},
      cov{cov},
      node{node}
  {
    pose.header.frame_id = "world";
    for(auto i: {0,1,2})
      pose.pose.covariance[7*i] = cov[i];
  }

  inline void fwdPose(const Pose &msg)
  {
    static std::default_random_engine engine;
    static std::normal_distribution<double> noise;

    pose.pose.pose.position.x = msg.position.x + cov[0]*noise(engine);
    pose.pose.pose.position.y = msg.position.y + cov[1]*noise(engine);
    pose.pose.pose.position.z = msg.position.z + cov[2]*noise(engine);
    pose.header.stamp = node->get_clock()->now();
    pub->publish(pose);
  }
};



class Gz2Topics : public rclcpp::Node
{
  rclcpp::Publisher<PoseStamped>::SharedPtr pose_pub;
  PoseStamped pose;



public:
  Gz2Topics() : Node("gz2topics")
  {
    // get noise params
    const Covariance cov;

    static auto lsm{ROVImu(this, "lsm", {cov.from("rpy"),cov.from("w"),cov.from("a")})};
    static auto mpu{ROVImu(this, "mpu", {cov.from("rpy"),cov.from("w"),cov.from("a")})};

    static auto usbl{ROVPose(this, "usbl", {cov.from("usbl"), cov.from("usbl"), cov.from("usbl")})};
    static auto depth{ROVPose(this, "depth", {1000, 1000, cov.from("depth")})};

    pose_pub = create_publisher<PoseStamped>("/bluerov2/pose_gt_stamped", 1);
    pose.header.frame_id = "world";

    static auto pose_sub = create_subscription<Pose>("pose_gt", 1, [&](Pose::UniquePtr msg)
    {usbl.fwdPose(*msg);depth.fwdPose(*msg);
    pose.pose = *msg;pose.header.stamp = get_clock()->now();
    pose_pub->publish(pose);
  });
  }

private:

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Gz2Topics>());
  rclcpp::shutdown();
  return 0;
}
