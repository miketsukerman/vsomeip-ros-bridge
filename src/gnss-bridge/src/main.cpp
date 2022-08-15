#include "abstract_someip_client.h"
#include "someip_publisher.h"

constexpr auto node_name = "GNSS_SOMEIP_Bridge";

auto main(int argc, char **argv) -> int 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SomeIpPublisher<GnssSomeIpClient>>(node_name));
    rclcpp::shutdown();
    return 0;
}
