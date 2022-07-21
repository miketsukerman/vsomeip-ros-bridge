#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <CommonAPI/CommonAPI.hpp>
#include <v0/gnss/GnssServerProxy.hpp>

#include <gnss_someip_lib/msg/gnss_data.hpp>

using GpsDataMsg = gnss_someip_lib::msg::GnssData;
using GnssData = v0::gnss::common::GnssData;

namespace TypesConversion {

GpsDataMsg to_gps_data(const GnssData & gnss_data) {
    
    GpsDataMsg gps_data_msg; 

    auto position = gnss_data.getPosition();

    gps_data_msg.position.fix.latitude = position.getFix().getLatitude();
    gps_data_msg.position.fix.longitude = position.getFix().getLongitude();
    gps_data_msg.position.dop.hdop = position.getDop().getHdop();
    gps_data_msg.position.dop.vdop = position.getDop().getVdop();
    gps_data_msg.position.satellites_visible = position.getSatellites_visible();
    gps_data_msg.position.satellites_used = position.getSatellites_used();

    return gps_data_msg;
}

} // namespace TypesConversion

class AbstractSomeIpClient 
{
public: 
    virtual bool initialised() = 0;
    virtual std::optional<bool> available() = 0;
};

class GnssSomeIpClient : public AbstractSomeIpClient
{
    static constexpr auto domain = "local";
    static constexpr auto instance = "GnssServer";

    using MessageCallback = std::function<void(const GpsDataMsg & message)>;

public:
    GnssSomeIpClient() : someip_proxy(CommonAPI::Runtime::get()->buildProxy<v0::gnss::GnssServerProxy>(domain,instance)) 
    {
        init();
    }

    bool initialised() override {
        return someip_proxy != nullptr; 
    }

    std::optional<bool> available() override {
        return (initialised()) ? std::make_optional(someip_proxy->isAvailable()) : std::nullopt;
    }

    void setMessageCallback(MessageCallback callback) {
        message_callback = callback;
    }

protected: 

    void init() {
        if(!someip_proxy)
        {
            // RCLCPP_ERROR(this->get_logger(), "Not able to initialize SOME/IP proxy for GNSS");
            //TODO: handle error case correctly
            return;
        }

        someip_proxy->getProxyStatusEvent().subscribe(std::bind(&GnssSomeIpClient::onAvailable, this, std::placeholders::_1));
    }

    void onAvailable(CommonAPI::AvailabilityStatus status) {
        // first event is always CommonAPI::AvailabilityStatus::NOT_AVAILABLE

        if (status == CommonAPI::AvailabilityStatus::AVAILABLE)
        {
            someip_proxy->getDataEvent().subscribe([this](const ::v0::gnss::common::GnssData & data) {

                auto message = TypesConversion::to_gps_data(data);

                message_callback(message);
            });
        }    

    }

private:
    MessageCallback message_callback;

    std::shared_ptr<v0::gnss::GnssServerProxy<>> someip_proxy;
};

template <typename SomeIpClient> 
class SomeIpPublisher : public rclcpp::Node
{
    static constexpr auto Topic = "GNSS";
    static constexpr auto QoS = 10;

public:
    SomeIpPublisher(std::string node_name) 
        : Node(node_name)
    {
        publisher = this->create_publisher<GpsDataMsg>(Topic, QoS);

        someip_client.setMessageCallback(std::bind(&SomeIpPublisher::publish, this, std::placeholders::_1));
    }

private:

    void publish(const GpsDataMsg & message) {        

        RCLCPP_INFO(this->get_logger(), "Publishing SOME/IP message on topic %s", Topic);

        publisher->publish(message);
    }

private:
    SomeIpClient someip_client;
    
    rclcpp::Publisher<GpsDataMsg>::SharedPtr publisher;
};

auto main(int argc, char **argv) -> int 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SomeIpPublisher<GnssSomeIpClient>>("GNSS_SOMEIP_Bridge"));
    rclcpp::shutdown();
    return 0;
}
