#include "rclcpp/rclcpp.hpp"

#include <gnss_ros_lib/msg/gps_data.hpp>

using GpsDataMsg = gnss_ros_lib::msg::GpsData;

class GnssSubsriber : public rclcpp::Node
{
  static constexpr auto topic = "GNSS";
  static constexpr auto qos = 10;

public:
    GnssSubsriber() : Node("GNSS_Subscriber")
    {
      subscription = this->create_subscription<GpsDataMsg>(topic, qos, std::bind(&GnssSubsriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const GpsDataMsg & msg) const
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
  rclcpp::spin(std::make_shared<GnssSubsriber>());
  rclcpp::shutdown();
  return 0;
}
