#include "rclcpp/rclcpp.hpp"

#include <gnss_someip_lib/msg/position.hpp>

using GpsDataMsg = gnss_someip_lib::msg::Position;

class GnssTopicSubsriber : public rclcpp::Node
{
  static constexpr auto topic = "GNSS";
  static constexpr auto qos = 10;

public:
    GnssTopicSubsriber() : Node("GNSS_Subscriber")
    {
      subscription = this->create_subscription<GpsDataMsg>(topic, qos, std::bind(&GnssTopicSubsriber::gnss_topic_callback, this, std::placeholders::_1));
    }

private:
    void gnss_topic_callback(const GpsDataMsg & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "GNSS position latitude %f, longitude %f", 
          msg.fix.latitude,
          msg.fix.longitude
        );
    }

    rclcpp::Subscription<GpsDataMsg>::SharedPtr subscription;
};

auto main(int argc, char * argv[]) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssTopicSubsriber>());
  rclcpp::shutdown();
  return 0;
}
