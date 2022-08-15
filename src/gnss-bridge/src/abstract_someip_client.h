#pragma once

#include <CommonAPI/CommonAPI.hpp>
#include <v0/gnss/GnssServerProxy.hpp>

#include <types/conversion.h>

using GpsDataMsg = gnss_someip_lib::msg::GnssData;
using GnssData = v0::gnss::common::GnssData;

template<template<typename ...> class P>
class AbstractSomeIpClient 
{
    using ProxyClass = P<>;
    using ProxyClassPtr = std::shared_ptr<ProxyClass>;

public: 
    AbstractSomeIpClient(std::string domain, std::string instance) 
        : proxy_(CommonAPI::Runtime::get()->buildProxy<P>(domain,instance)) 
    {
        init();
    }

    virtual std::optional<bool> available() {
        return (initialised()) ? std::make_optional(proxy_->isAvailable()) : std::nullopt;
    }

    virtual void onAvailable() = 0;

protected:
    ProxyClassPtr proxy() {
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

        proxy_->getProxyStatusEvent().subscribe(std::bind(&AbstractSomeIpClient<P>::onAvailablilityStatusChange, this, std::placeholders::_1));
    }

    virtual void onAvailablilityStatusChange(CommonAPI::AvailabilityStatus status) {
        // first event is always CommonAPI::AvailabilityStatus::NOT_AVAILABLE

        if (status == CommonAPI::AvailabilityStatus::AVAILABLE)
        {
            onAvailable();
        }
    }

private:
    ProxyClassPtr proxy_;
};

using GnssSomeIpProxyWrapper = AbstractSomeIpClient<v0::gnss::GnssServerProxy>;

class GnssSomeIpClient : public GnssSomeIpProxyWrapper
{
    static constexpr auto domain = "local";
    static constexpr auto instance = "GnssServer";

    using MessageCallback = std::function<void(const GpsDataMsg & message)>;

public:
    GnssSomeIpClient() : GnssSomeIpProxyWrapper(domain, instance) {}

    void setMessageCallback(MessageCallback callback) {
        message_callback = std::move(callback);
    }

    void onAvailable() override {
        proxy()->getDataEvent().subscribe([this](const GnssData & data) {

            auto message = Types::Conversion::from_capi_type(data);

            message_callback(message);
        });
    }

private:
    MessageCallback message_callback;
};
