#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class GnssSubsriber : public rclcpp::Node
{
public:
    GnssSubsriber() : Node("GNSS_Subscriber")
    {
      subscription = this->create_subscription<std_msgs::msg::String>(
      "GNSS", 10, std::bind(&GnssSubsriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "GNSS json: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
};

auto main(int argc, char * argv[]) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssSubsriber>());
  rclcpp::shutdown();
  return 0;
}
