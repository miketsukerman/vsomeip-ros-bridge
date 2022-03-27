#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <CommonAPI/CommonAPI.hpp>
#include <v0/gnss/TimeServerProxy.hpp>

#include <nlohmann/json.hpp>

class GnssSomeIpPublisher : public rclcpp::Node
{
public:
    GnssSomeIpPublisher() 
        : Node("GNSS_SOMEIP_Bridge")
        , someip_proxy(CommonAPI::Runtime::get()->buildProxy<v0::gnss::TimeServerProxy>("local","TimeServer"))
        , availability_status(CommonAPI::AvailabilityStatus::UNKNOWN)
    {
        publisher = this->create_publisher<std_msgs::msg::String>("GNSS", 10);

        if(!someip_proxy)
        {
            //TODO: handle error case correctly
            RCLCPP_INFO(this->get_logger(), "Failed to create SOME/IP TimeServer proxy");
        }

        RCLCPP_INFO(this->get_logger(), "SOME/IP TimeServer has been created");
        
        auto availabilityStatusFuture = availability_status_promise.get_future();
        someip_proxy->getProxyStatusEvent().subscribe([this](CommonAPI::AvailabilityStatus status)
        {
            RCLCPP_INFO(get_logger(), "Receiving interface %s availability status = %d" 
                                     ,someip_proxy->getInterface() 
                                     ,static_cast<uint16_t>(status));

            if (status == CommonAPI::AvailabilityStatus::AVAILABLE)
            {
                availability_status_promise.set_value(status);
            }
        });

        std::future_status futureStatus = availabilityStatusFuture.wait_for(std::chrono::seconds(3));
        
        if (futureStatus == std::future_status::ready) {
            availability_status = availabilityStatusFuture.get();

            RCLCPP_INFO(this->get_logger(), "Proxy is ready, subscribing for an event");

            someip_proxy->getNowEvent().subscribe([this](const ::v0::gnss::common::Time & time) {
                RCLCPP_INFO(get_logger(), "GNSS Time broadcast received H=%d M=%d S=%d"
                            , time.getHours(), time.getMinutes(), time.getSeconds());

                log(time);
            });
        } else {
            //TODO: handle error case properly
            RCLCPP_INFO(this->get_logger(), "proxy not available");
        }
    }

private:

    void log(const v0::gnss::common::Time & time) {
        
        using json = nlohmann::json;
        
        json j = json({});
        auto message = std_msgs::msg::String();

        j["gnss"] = {
            {"time", 
                {"hours", time.getHours()},
                {"minutes", time.getMinutes()},
                {"seconds", time.getSeconds()} 
            }
        };

        std::stringstream strstream; 
        strstream << j;
        auto str = strstream.str();

        message.data = str;

        RCLCPP_INFO(this->get_logger(), "Publishing to the GNSS topic");
        RCLCPP_INFO(this->get_logger(), str.c_str());

        publisher->publish(message);
    }

private:
    CommonAPI::AvailabilityStatus availability_status;
    std::promise<CommonAPI::AvailabilityStatus> availability_status_promise;

    std::shared_ptr<v0::gnss::TimeServerProxy<>> someip_proxy;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
};


auto main(int argc, char **argv) -> int 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GnssSomeIpPublisher>());
    rclcpp::shutdown();
    return 0;
}