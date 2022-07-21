#include "rclcpp/rclcpp.hpp"

#include <gnss_someip_lib/msg/gnss_data.hpp>

using GpsDataMsg = gnss_someip_lib::msg::GnssData;

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
          msg.position.fix.latitude,
          msg.position.fix.longitude
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
