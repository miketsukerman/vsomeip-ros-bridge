#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include <CommonAPI/CommonAPI.hpp>
#include <v0/gnss/TimeServerStubDefault.hpp>

#include <chrono>

using namespace std::chrono;

class GnssSomeIpProvider : public v0::gnss::TimeServerStubDefault {

public:
    GnssSomeIpProvider() = default;
    ~GnssSomeIpProvider() = default;

    void fireNowEvent() {
        v0::gnss::common::Time time;

        auto current_time = system_clock::now().time_since_epoch();

        time.setHours(duration_cast<hours>(current_time).count());
        time.setMinutes(duration_cast<minutes>(current_time).count());
        time.setSeconds(duration_cast<seconds>(current_time).count());

        TimeServerStub::fireNowEvent(time);
    }

};

class GnssSomeIpReporter : public rclcpp::Node
{
public:
    GnssSomeIpReporter() 
        : Node("GNSS_SOMEIP_Reporter")
        , someip_provider(std::make_shared<GnssSomeIpProvider>())
    {
        if(!CommonAPI::Runtime::get()->registerService("local","TimeServer", someip_provider))
        {
            RCLCPP_INFO(this->get_logger(), "Failed to register SOME/IP TimeServer");
        }
        
        RCLCPP_INFO(this->get_logger(), "SOME/IP TimeServer has been registered");

        publish_timer = this->create_wall_timer(2s, [this]() {            
            RCLCPP_INFO(this->get_logger(), "Broadcast GNSS time over SOME/IP");
            someip_provider->fireNowEvent();
        });

    }

private:
    rclcpp::TimerBase::SharedPtr publish_timer;
    std::shared_ptr<GnssSomeIpProvider> someip_provider;
};

auto main(int argc, char **argv) -> int 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GnssSomeIpReporter>());
    rclcpp::shutdown();
    return 0;
}