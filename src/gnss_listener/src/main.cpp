#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "std_msgs/msg/string.hpp"

#include <nlohmann/json.hpp>

using json = nlohmann::json;

class GnssListener : public rclcpp::Node
{
public:
  
  GnssListener() : Node("gnss_listener")
  {
  }


};

auto main(int argc, char * argv[]) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssListener>());
  rclcpp::shutdown();
  return 0;
}
