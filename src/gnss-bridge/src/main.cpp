#include "abstract_someip_client.h"
#include "someip_publisher.h"

auto main(int argc, char **argv) -> int 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SomeIpPublisher<GnssSomeIpClient>>("GNSS_SOMEIP_Bridge"));
    rclcpp::shutdown();
    return 0;
}
