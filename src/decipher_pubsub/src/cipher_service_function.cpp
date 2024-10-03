#include "rclcpp/rclcpp.hpp"
#include "cipher_interfaces/srv/cipher_answer.hpp"

#include <memory>

void check(const std::shared_ptr<cipher_interfaces::srv::CipherAnswer::Request> request,
          std::shared_ptr<cipher_interfaces::srv::CipherAnswer::Response>      response)
{
  response->result = true;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %s",
                request->answer.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%d]", response->result);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("cipher_server");

  rclcpp::Service<cipher_interfaces::srv::CipherAnswer>::SharedPtr service =
    node->create_service<cipher_interfaces::srv::CipherAnswer>("cipher", &check);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to check answer.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}