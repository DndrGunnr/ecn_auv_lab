#include <filesystem>
#include <math.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

struct Waypoint
{
  double x, y, z, theta;



  inline static auto load(std::vector<Waypoint> &waypoints,
                          double &pose_thr,
                          double &orient_thr)
  {
    const std::string param_dir{AUV_PARAMS};
    const auto wp_file{param_dir + "waypoints.yaml"};
    if(!std::filesystem::exists(wp_file))
      throw std::invalid_argument("Waypoint file does not exist: " + wp_file);

    auto root{YAML::LoadFile(wp_file)};
    pose_thr = root["threshold"].as<double>();
    orient_thr = root["threshold_angle"].as<double>();

    for(auto &wp: root["wp"].as<std::vector<std::map<std::string, double>>>())
    {
      auto &last{waypoints.emplace_back()};
      last.x = wp["x"];
      last.y = wp["y"];
      last.z = wp["z"];
      last.theta = wp["theta"];
    }
  }

  void write(geometry_msgs::msg::PoseStamped &pose) const
  {
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation.z = sin(theta/2);
    pose.pose.orientation.w = cos(theta/2);
  }




};
