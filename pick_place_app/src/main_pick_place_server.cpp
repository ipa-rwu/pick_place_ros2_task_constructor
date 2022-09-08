#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <tf2_eigen/tf2_eigen.h>

#include <pick_place_app/pick_place_server.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_server_demo");

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  const std::string node_name = "pick_place_server_demo";

  auto service_node = std::make_shared<pick_place_server::PickPlaceServer>(node_name, options);

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(service_node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
