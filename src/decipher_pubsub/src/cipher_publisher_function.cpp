#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "cipher_interfaces/msg/cipher_message.hpp"                                         // CHANGE

using namespace std::chrono_literals;

class CipherPublisher : public rclcpp::Node
{
public:
  CipherPublisher()
  : Node("cipher_publisher")
  {
    publisher_ = this->create_publisher<cipher_interfaces::msg::CipherMessage>("topic", 10);  // CHANGE
    timer_ = this->create_wall_timer(
      500ms, std::bind(&CipherPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = cipher_interfaces::msg::CipherMessage();                                   // CHANGE
    message.message = "SRKDOMZVECZVEC";
    message.key = 10;                                                     // CHANGE
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing message: '" << message.message << "' and key: " << std::to_string(message.key) << "");    // CHANGE
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<cipher_interfaces::msg::CipherMessage>::SharedPtr publisher_;             // CHANGE
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CipherPublisher>());
  rclcpp::shutdown();
  return 0;
}