#pragma once

#include <CommonAPI/CommonAPI.hpp>
#include <v0/gnss/GnssServerProxy.hpp>

#include <types/conversion.h>

using GpsDataMsg = gnss_someip_lib::msg::GnssData;
using GnssData = v0::gnss::common::GnssData;

template <typename T>
class AbstractSomeIpClient 
{
public: 
    AbstractSomeIpClient(std::string domain, std::string instance) : proxy_(CommonAPI::Runtime::get()->buildProxy<v0::gnss::GnssServerProxy>(domain,instance)) 
    {
        init();
    }

    virtual std::optional<bool> available() {
        return (initialised()) ? std::make_optional(proxy_->isAvailable()) : std::nullopt;
    }

    virtual void onAvailable() = 0;

protected:
    std::shared_ptr<T> proxy() {
        return proxy_;
    }

    virtual bool initialised() {
        return proxy_ != nullptr; 
    }

    void init() {
        if(!proxy_)
        {
            // RCLCPP_ERROR(this->get_logger(), "Not able to initialize SOME/IP proxy for GNSS");
            //TODO: handle error case correctly
            return;
        }

        proxy_->getProxyStatusEvent().subscribe(std::bind(&AbstractSomeIpClient<T>::onAvailablilityStatusChange, this, std::placeholders::_1));
    }

    virtual void onAvailablilityStatusChange(CommonAPI::AvailabilityStatus status) {
        // first event is always CommonAPI::AvailabilityStatus::NOT_AVAILABLE

        if (status == CommonAPI::AvailabilityStatus::AVAILABLE)
        {
            onAvailable();
        }    
    }

private:
    std::shared_ptr<T> proxy_;
};

class GnssSomeIpClient : public AbstractSomeIpClient<v0::gnss::GnssServerProxy<>>
{
    static constexpr auto domain = "local";
    static constexpr auto instance = "GnssServer";

    using MessageCallback = std::function<void(const GpsDataMsg & message)>;

public:
    GnssSomeIpClient() : AbstractSomeIpClient<v0::gnss::GnssServerProxy<>>(domain, instance) {}

    void setMessageCallback(MessageCallback callback) {
        message_callback = callback;
    }

    void onAvailable() override {
        proxy()->getDataEvent().subscribe([this](const GnssData & data) {

            auto message = Types::Conversion::to_gps_data(data);

            message_callback(message);
        });
    }

private:
    MessageCallback message_callback;
};
