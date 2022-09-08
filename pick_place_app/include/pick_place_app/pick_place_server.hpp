#pragma once

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <memory>
#include <pick_place_app/pick_place_task.hpp>
#include <string>
#include <vector>

#include "pick_place_msgs/srv/pick_place.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pick_place_server
{
class PickPlaceServer
{
public:
  /**
   * @brief A constructor
   */

  explicit PickPlaceServer(const rclcpp::NodeOptions & options);
  PickPlaceServer(const std::string & node_name, const rclcpp::NodeOptions & options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }

  /**
   * @brief A Destructor for GenerateTrajectory
   */
  ~PickPlaceServer();

  moveit_msgs::msg::CollisionObject createObject(
    const std::string object_name, const std::string frame_id,
    const geometry_msgs::msg::Pose & pose);

protected:
  rclcpp::Service<pick_place_msgs::srv::PickPlace>::SharedPtr service_;

  /**
   * @brief Trajectory generating service callback
   * @param request Service request
   * @param response Service response
   */
  void pickPlaceCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<pick_place_msgs::srv::PickPlace::Request> request,
    std::shared_ptr<pick_place_msgs::srv::PickPlace::Response> response);

private:
  std::shared_ptr<rclcpp::Node> node_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  PickPlaceTask::Parameters pick_place_parameters_;
};

}  // namespace pick_place_server
