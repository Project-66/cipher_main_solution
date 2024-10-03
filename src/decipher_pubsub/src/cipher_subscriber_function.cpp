#include <memory>
#include <string>
#include <chrono>
#include <cstdlib>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "cipher_interfaces/msg/cipher_message.hpp"
#include "cipher_interfaces/srv/cipher_answer.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

class CipherSubscriber : public rclcpp::Node
{
  public:
    CipherSubscriber()
    : Node("cipher_subscriber")
    {
      subscription_ = this->create_subscription<cipher_interfaces::msg::CipherMessage>(
      "topic", 10, std::bind(&CipherSubscriber::topic_callback, this, _1));
    }



  private:

    void topic_callback(const cipher_interfaces::msg::CipherMessage & msg) const
    {
        std::string message = msg.message;
        std::string decodedMsg = message;

        for (unsigned long i = 0; i < message.length(); i++) {
            int key = msg.key % 26;
            decodedMsg[i] = message[i] - key;
            if (message[i] >= 'a' && message[i] <= 'z' && decodedMsg[i] < 'a') {
                decodedMsg[i] += 26;
            }

            if (message[i] >= 'A' && message[i] <= 'Z' && decodedMsg[i] < 'A') {
                decodedMsg[i] += 26;
            }
        }

        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("cipher_client");
        rclcpp::Client<cipher_interfaces::srv::CipherAnswer>::SharedPtr client =
            node->create_client<cipher_interfaces::srv::CipherAnswer>("cipher");

        auto request = std::make_shared<cipher_interfaces::srv::CipherAnswer::Request>();
        request->answer = decodedMsg;

        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                exit(1);
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result = client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node, result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            if (result.get()->result) {
                RCLCPP_INFO(this->get_logger(), "The decoded message is '%s', and has been marked correct", decodedMsg.c_str());
            } else {
                 RCLCPP_INFO(this->get_logger(), "The partially decoded message is '%s', and has been marked incorrect", decodedMsg.c_str());
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service cipher_service");
        }

        rclcpp::shutdown();
    }
    rclcpp::Subscription<cipher_interfaces::msg::CipherMessage>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CipherSubscriber>());

  rclcpp::shutdown();
  return 0;
}