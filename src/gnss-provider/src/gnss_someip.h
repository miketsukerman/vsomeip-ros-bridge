#pragma once 

#include <CommonAPI/CommonAPI.hpp>

#include <v0/gnss/GnssServerStubDefault.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono>


class GnssSomeIpProvider : public v0::gnss::GnssServerStubDefault {

public:
    GnssSomeIpProvider() = default;
    ~GnssSomeIpProvider() = default;

    void fireDataEvent(const GnssDataMsg & gps_data) {

        RCLCPP_INFO(rclcpp::get_logger("GNSS_SOMEIP_Provider"), "Sending gnss data over SOME/IP.");

        auto data = Types::Conversion::to_capi_type(gps_data);

        GnssServerStub::fireDataEvent(data);
    }

};

template <typename T>
class GnssSomeIpReporter : public rclcpp::Node
{
    static constexpr auto node_name = "GNSS_SOMEIP_Reporter";

public:
    GnssSomeIpReporter()
        : Node(node_name)
        , someip_provider(std::make_shared<T>())
    {
        this->declare_parameter<std::string>("someip_domain", "local");
        this->declare_parameter<std::string>("someip_instance", "GnssServer");
        this->declare_parameter<std::string>("gpsd_topic", "GPSD");
        this->declare_parameter<int>("gpsd_qos", 10);

        if (!try_register_and_start()) {
            RCLCPP_WARN(this->get_logger(), "SOME/IP registration failed, scheduling retry every 5 s");
            retry_timer_ = this->create_wall_timer(
                std::chrono::seconds(5),
                [this]() {
                    if (try_register_and_start()) {
                        retry_timer_->cancel();
                        retry_timer_.reset();
                    }
                });
        }
    }

protected:

    bool try_register_and_start() {
        const auto domain = this->get_parameter("someip_domain").as_string();
        const auto instance = this->get_parameter("someip_instance").as_string();

        if (!CommonAPI::Runtime::get()->registerService(domain, instance, someip_provider)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to register SOME/IP GnssServer");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "SOME/IP GnssServer has been registered");

        const auto topic = this->get_parameter("gpsd_topic").as_string();
        const auto qos = static_cast<int>(this->get_parameter("gpsd_qos").as_int());

        gpsd_data_subscription = this->create_subscription<GnssDataMsg>(
            topic, qos,
            std::bind(&GnssSomeIpReporter::on_gpsd_data, this, std::placeholders::_1));

        return true;
    }

    void on_gpsd_data(const GnssDataMsg & msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received GPS data, broadcasting over SOME/IP");

        someip_provider->fireDataEvent(msg);
    }

private:
    std::shared_ptr<T> someip_provider;
    rclcpp::TimerBase::SharedPtr retry_timer_;
    rclcpp::Subscription<GnssDataMsg>::SharedPtr gpsd_data_subscription;
};
