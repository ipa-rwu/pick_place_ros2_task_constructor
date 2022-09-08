#include "pick_place_app/pick_place_server.hpp"

#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_eigen/tf2_eigen.h>

#include <chrono>
#include <pick_place_app/pick_place_server.hpp>
#include <pluginlib/class_loader.hpp>

using namespace std::chrono_literals;

namespace pick_place_server
{
using namespace std::placeholders;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_server");

PickPlaceServer::PickPlaceServer(const rclcpp::NodeOptions & node_options)
: node_{std::make_shared<rclcpp::Node>("tpick_place_server_node", node_options)}
{
}

PickPlaceServer::PickPlaceServer(const std::string & node_name, const rclcpp::NodeOptions & options)
: node_{std::make_shared<rclcpp::Node>(node_name, options)}
{
  // able to call a service server from a service server
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  service_ = node_->create_service<pick_place_msgs::srv::PickPlace>(
    "pick_place_service", std::bind(&PickPlaceServer::pickPlaceCallback, this, _1, _2, _3),
    rmw_qos_profile_services_default, callback_group_);

  pick_place_parameters_.loadParameters(node_);
  RCLCPP_INFO(LOGGER, "Creating pick_place_service");
}

PickPlaceServer::~PickPlaceServer() {}

void PickPlaceServer::pickPlaceCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<pick_place_msgs::srv::PickPlace::Request> request,
  std::shared_ptr<pick_place_msgs::srv::PickPlace::Response> response)
{
  auto object_pose = request->object_pose;
  auto object_name = request->object_name;
  auto place_pose = request->place_pose;

  pick_place_parameters_.object_name = object_name;
  pick_place_parameters_.object_frame_id = object_pose.header.frame_id;
  pick_place_parameters_.place_frame_id = place_pose.header.frame_id;
  pick_place_parameters_.place_pose = place_pose.pose;
  pick_place_parameters_.object_pose = object_pose.pose;

  PickPlaceTask pick_place_task(node_, pick_place_parameters_);

  RCLCPP_INFO(LOGGER, "object_name: %s", request->object_name.c_str());

  RCLCPP_INFO(LOGGER, "object_frame_id: %s", object_pose.header.frame_id.c_str());

  RCLCPP_INFO(LOGGER, "place_frame_id: %s", pick_place_parameters_.object_frame_id.c_str());

  moveit::planning_interface::PlanningSceneInterface psi;
  // get pose from param
  psi.applyCollisionObject(createObject(
    object_name, object_pose.header.frame_id,
    tf2::toMsg(Eigen::Isometry3d(Eigen::Translation3d(
      object_pose.pose.position.x, object_pose.pose.position.y, object_pose.pose.position.z)))));
  rclcpp::sleep_for(500ms);

  if (!pick_place_task.plan()) {
    response->result = false;
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to plan");
  }

  if (!pick_place_task.execute()) {
    response->result = false;
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to execute");
  }

  response->result = true;
  return;
}

moveit_msgs::msg::CollisionObject PickPlaceServer::createObject(
  const std::string object_name, const std::string frame_id, const geometry_msgs::msg::Pose & pose)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = object_name;
  object.header.frame_id = frame_id;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions.resize(
    geometric_shapes::solidPrimitiveDimCount<shape_msgs::msg::SolidPrimitive::CYLINDER>());
  object.primitives[0].dimensions.at(shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT) = 0.1;
  object.primitives[0].dimensions.at(shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS) = 0.02;
  object.pose = pose;
  return object;
}

}  // namespace pick_place_server

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pick_place_server::PickPlaceServer)
