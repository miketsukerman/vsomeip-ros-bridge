#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "std_msgs/msg/string.hpp"

#include <CommonAPI/CommonAPI.hpp>
#include <v0/gnss/GnssServerStubDefault.hpp>

#include <libgpsmm.h>

#include <gnss_ros_lib/msg/gps_data.hpp>

using namespace std::chrono;
using GpsDataMsg = gnss_ros_lib::msg::GpsData;
using GnssData = v0::gnss::common::GnssData;

namespace TypeConversion {

GnssData to_gnss_data(const GpsDataMsg & gps_data) {
    GnssData gnss_data;

    v0::gnss::common::Position position;
    v0::gnss::common::Time time;
    v0::gnss::common::Fix fix; 
    v0::gnss::common::Dop dop;

    fix.setLatitude(gps_data.fix.latitude);
    fix.setLongitude(gps_data.fix.longitude);
    
    dop.setHdop(gps_data.dop.hdop);
    dop.setVdop(gps_data.dop.vdop);
    dop.setPdop(gps_data.dop.pdop);

    position.setSatellites_visible(gps_data.satellites_visible);
    position.setSatellites_used(gps_data.satellites_used);
    position.setDop(dop);
    position.setFix(fix);

    gnss_data.setPosition(position);
    gnss_data.setTime(time);

    return gnss_data;
}

GpsDataMsg to_gps_data(const gps_data_t * gps_data) {
    GpsDataMsg gps_data_msg; 

    gps_data_msg.fix.latitude = gps_data->fix.latitude;
    gps_data_msg.fix.longitude = gps_data->fix.longitude;
    gps_data_msg.dop.hdop = gps_data->dop.hdop;
    gps_data_msg.dop.vdop = gps_data->dop.vdop;
    gps_data_msg.satellites_visible = gps_data->satellites_visible;
    gps_data_msg.satellites_used = gps_data->satellites_used;

    return gps_data_msg;
}

} // namespace TypeConversion

// inspired by https://gist.github.com/ncoder-1/8313815ac387e6757f751dc8960f03d7
class GpsdClient : public rclcpp::Node {

    static constexpr auto GpsdHost = "localhost";
    static constexpr auto WaitingTime = 1000000;

    static constexpr auto Topic = "GPSD";
    static constexpr auto QoS = 10;

public:
    GpsdClient()
        : Node("GPSD_Client_node"),
        gps_rec(GpsdHost, DEFAULT_GPSD_PORT) 
    {
        // callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);        
        publisher = this->create_publisher<GpsDataMsg>(Topic, QoS);

        if(!init()) {
            RCLCPP_WARN(this->get_logger(), "No connection to gpsd");
        }

        timer = this->create_wall_timer(500ms, std::bind(&GpsdClient::read, this));

        //TODO: handle case when there is no connection to gpsd
    }
    ~GpsdClient() = default;

    bool init() {
        if (gps_rec.stream(WATCH_ENABLE | WATCH_JSON) == nullptr) {
            return false;
        }        

        return true;
    }

    void read() {
        for(;;) 
        {
            std::lock_guard<std::mutex> lock_guard(mutex);
            
            if (!gps_rec.waiting(WaitingTime)) {
                RCLCPP_WARN(this->get_logger(), "Waiting, failed to read data from gpsd");
                continue;
            }

            struct gps_data_t* gpsd_data;

            if ((gpsd_data = gps_rec.read()) == nullptr) {
                RCLCPP_WARN(this->get_logger(), "Null, Failed to read data from gpsd");
                continue;
            }

            // const auto time_str{TimespecToTimeStr(gpsd_data->fix.time, ISO_8601)};  // you can change the 2nd argument to LOCALTIME, UTC, UNIX or ISO8601

            RCLCPP_WARN(this->get_logger(), "Obtained data from gpsd");

            GpsDataMsg msg = TypeConversion::to_gps_data(gpsd_data);
            
            publisher->publish(msg);

            return;
        }
    }

protected:

private:
    gpsmm gps_rec;

    std::mutex mutex;

    // rclcpp::CallbackGroup::SharedPtr callback_group;
    
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<GpsDataMsg>::SharedPtr publisher;
};

class GnssSomeIpProvider : public v0::gnss::GnssServerStubDefault {

public:
    GnssSomeIpProvider() = default;
    ~GnssSomeIpProvider() = default;

    void fireDataEvent(const GpsDataMsg & gps_data) {
        
        RCLCPP_INFO(rclcpp::get_logger("GNSS_SOMEIP_Provider"), "Sending gnss data over SOME/IP.");

        GnssData data = TypeConversion::to_gnss_data(gps_data);

        GnssServerStub::fireDataEvent(data);
    }

};

template <typename T>
class GnssSomeIpReporter : public rclcpp::Node
{
    static constexpr auto domain = "local";
    static constexpr auto instance = "GnssServer";
    static constexpr auto timer_duration = 2s;

    static constexpr auto topic = "GPSD";
    static constexpr auto qos = 10;

public:
    GnssSomeIpReporter()
        : Node("GNSS_SOMEIP_Reporter")
        , someip_provider(std::make_shared<T>())
    {
        if(register_someip_service()) {
            RCLCPP_INFO(this->get_logger(), "SOME/IP GnssServer has been registered");

            gpsd_data_subscription = this->create_subscription<GpsDataMsg>(topic, qos, std::bind(&GnssSomeIpReporter::on_gpsd_data, this, std::placeholders::_1));

            publish_timer = this->create_wall_timer(timer_duration, [this]() {            
                RCLCPP_INFO(this->get_logger(), "Timer: Broadcast GNSS data over SOME/IP");
        
                std::lock_guard<std::mutex> guard(mutex);

                someip_provider->fireDataEvent(gps_data);
            });
        }
    }

protected:

    bool register_someip_service() {
        if(!CommonAPI::Runtime::get()->registerService(domain,instance, someip_provider)) {
            //TODO: handle error case correctly
            RCLCPP_INFO(this->get_logger(), "Failed to register SOME/IP GnssServer");
            return false;
        }

        return true;
    }

    void on_gpsd_data(const GpsDataMsg & msg) 
    {
        std::lock_guard<std::mutex> guard(mutex);

        RCLCPP_INFO(this->get_logger(), "Received GPS raw data from GpsdClient node");

        gps_data = msg;
    }

private:
    rclcpp::TimerBase::SharedPtr publish_timer;
    std::shared_ptr<T> someip_provider;

    std::mutex mutex;

    GpsDataMsg gps_data;

    rclcpp::Subscription<GpsDataMsg>::SharedPtr gpsd_data_subscription;
};

auto main(int argc, char **argv) -> int 
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    auto gnss_someip_reporter_node = std::make_shared<GnssSomeIpReporter<GnssSomeIpProvider>>();
    auto gpsd_client_node = std::make_shared<GpsdClient>();

    executor.add_node(gnss_someip_reporter_node);
    executor.add_node(gpsd_client_node);

    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
