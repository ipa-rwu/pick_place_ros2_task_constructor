#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <tf2_eigen/tf2_eigen.h>

#include <pick_place_app/pick_place_task.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_demo");

moveit_msgs::msg::CollisionObject createTable(
  const std::string & table_name, const geometry_msgs::msg::Pose & pose)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = table_name;
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions.resize(
    geometric_shapes::solidPrimitiveDimCount<shape_msgs::msg::SolidPrimitive::BOX>());
  object.primitives[0].dimensions.at(shape_msgs::msg::SolidPrimitive::BOX_X) = 0.2;
  object.primitives[0].dimensions.at(shape_msgs::msg::SolidPrimitive::BOX_Y) = 0.2;
  object.primitives[0].dimensions.at(shape_msgs::msg::SolidPrimitive::BOX_Z) = 0.25;
  object.primitive_poses.push_back(pose);
  object.primitive_poses.back().position.z +=
    0.5 * object.primitives[0].dimensions.at(
            shape_msgs::msg::SolidPrimitive::BOX_Z);  // align surface with world
  return object;
}

moveit_msgs::msg::CollisionObject createObject(
  const std::string object_name, const std::string frame_id, const geometry_msgs::msg::Pose & pose)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = object_name;
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions.resize(
    geometric_shapes::solidPrimitiveDimCount<shape_msgs::msg::SolidPrimitive::CYLINDER>());
  object.primitives[0].dimensions.at(shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT) = 0.1;
  object.primitives[0].dimensions.at(shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS) = 0.02;
  object.pose = pose;
  return object;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("pick_place_demo", "", options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &node]() {
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    executor.remove_node(node->get_node_base_interface());
  });

  PickPlaceTask::Parameters pick_place_parameters;
  pick_place_parameters.loadParameters(node);

  PickPlaceTask pick_place_task(node, pick_place_parameters);

  moveit::planning_interface::PlanningSceneInterface psi;
  // get pose from param
  psi.applyCollisionObject(createObject(
    pick_place_parameters.object_name, pick_place_parameters.object_frame_id,
    tf2::toMsg(Eigen::Isometry3d(Eigen::Translation3d(
      pick_place_parameters.object_pose.position.x, pick_place_parameters.object_pose.position.y,
      pick_place_parameters.object_pose.position.z)))));
  rclcpp::sleep_for(500ms);

  if (!pick_place_task.plan()) {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to plan");
  }

  if (!pick_place_task.execute()) {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to execute");
  }

  // Keep introspection alive
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
