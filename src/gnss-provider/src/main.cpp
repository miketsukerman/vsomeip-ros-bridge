#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include <boost/log/core.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

#include <boost/asio/io_service.hpp>

#include <CommonAPI/CommonAPI.hpp>
#include <v0/gnss/TimeServerStubDefault.hpp>

#include <chrono>

namespace logging = boost::log;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
namespace keywords = boost::log::keywords;

using namespace std::chrono;

using namespace logging::trivial;
src::severity_logger< severity_level > lg;

class GnssSomeIpProvider : public v0::gnss::TimeServerStubDefault {

public:
    GnssSomeIpProvider() = default;
    ~GnssSomeIpProvider() = default;

    void fireNowEvent() {
        v0::gnss::common::Time time;

        auto current_time = std::chrono::system_clock::now().time_since_epoch();

        time.setHours(std::chrono::duration_cast<std::chrono::hours>(current_time).count());
        time.setMinutes(std::chrono::duration_cast<std::chrono::minutes>(current_time).count());
        time.setSeconds(std::chrono::duration_cast<std::chrono::seconds>(current_time).count());

        BOOST_LOG_SEV(lg, info) << "broadcast current time " 
                                << static_cast<int>(time.getHours()) << ":" 
                                << static_cast<int>(time.getMinutes()) << ":"
                                << static_cast<int>(time.getSeconds());

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