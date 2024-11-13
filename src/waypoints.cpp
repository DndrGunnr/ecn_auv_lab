#include "waypoints.h"

#include <rclcpp/rclcpp.hpp>
#include <exception>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>
#include <cmath>

using namespace geometry_msgs::msg;
using namespace std::chrono_literals;

class Control : public rclcpp::Node
{
public:
  Control() : Node("waypoints")
  {
    set_parameter(rclcpp::Parameter("use_sim_time", true));
    // load waypoints and thresholds
    Waypoint::load(waypoints, position_thr, orientation_thr);

    pose_cmd.header.frame_id = "world";
    pub = create_publisher<PoseStamped>("/bluerov2/cmd_pose", 1);

    static auto timer{create_wall_timer(10ms, [&]()
      {
        if(!buffer.canTransform("world", "bluerov2/base_link", tf2::TimePointZero))
          return;
        const auto transform{buffer.lookupTransform("world", "bluerov2/base_link", tf2::TimePointZero)};
        trackWaypoint(transform.transform);
      })};
  }

private:
//vettore wp
  std::vector<Waypoint> waypoints;
  double position_thr, orientation_thr;

  tf2_ros::Buffer buffer{get_clock()};
  tf2_ros::TransformListener tl{buffer};
//pose_cmd oggetto prossimo wp
  PoseStamped pose_cmd;
  rclcpp::Publisher<PoseStamped>::SharedPtr pub;


  void trackWaypoint(const Transform &pose)
  {

    static auto cur_wp{0};

    // TODO update cur_wp to cycle through the waypoints when the current one is reached
    //pose posizione corrente del robot
    float x=pose.translation.x;
    float y=pose.translation.y;
    float z=pose.translation.z;
    float orientation = 2*atan2(pose.rotation.z,pose.rotation.w);


    float position_error=sqrt(pow(x-waypoints[cur_wp].x,2)+pow(y-waypoints[cur_wp].y,2)+pow(z-waypoints[cur_wp].z,2));
    float angular_error=abs(orientation-waypoints[cur_wp].theta);

    std::cout<<"position error" <<position_error <<std::endl;

    if (position_error<position_thr && angular_error < orientation_thr){
        cur_wp++;
        if(cur_wp==(int)waypoints.size())
            cur_wp=0;
        //RCLCPP_INFO(this->get_logger(), " dfgd");

    }



    waypoints[cur_wp].write(pose_cmd);
    pose_cmd.header.stamp = get_clock()->now();
    pub->publish(pose_cmd);
  }
};



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Control>());
  rclcpp::shutdown();
  return 0;
}
