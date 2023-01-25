#include "rclcpp/rclcpp.hpp"
#include "ur_msgs/srv/set_io.hpp"

#include <memory>

bool setio(const std::shared_ptr<ur_msgs::srv::SetIO::Request> request,
           std::shared_ptr<ur_msgs::srv::SetIO::Response> response)
{
  response->success = true;

  RCLCPP_INFO(rclcpp::get_logger("SetIO"), "Setting digital output '%d' to state: '%1.0f'.", request->pin,
              request->state);
  return true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("mock_IO_server");

  rclcpp::Service<ur_msgs::srv::SetIO>::SharedPtr service =
      node->create_service<ur_msgs::srv::SetIO>("~/set_io", &setio);

  RCLCPP_INFO(rclcpp::get_logger("SetIO"), "mocking IO server.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
