#pragma once

#include "rclcpp/rclcpp.hpp"

#include <gnss_someip_lib/msg/gnss_data.hpp>

using GpsDataMsg = gnss_someip_lib::msg::GnssData;

template <typename SomeIpClient>
class SomeIpPublisher : public rclcpp::Node
{
public:
    SomeIpPublisher(std::string node_name)
        : Node(node_name)
    {
        this->declare_parameter<std::string>("gnss_topic", "GNSS");
        this->declare_parameter<int>("gnss_qos", 10);

        topic_ = this->get_parameter("gnss_topic").as_string();
        const auto qos = static_cast<int>(this->get_parameter("gnss_qos").as_int());

        publisher = this->create_publisher<GpsDataMsg>(topic_, qos);

        someip_client.setMessageCallback(std::bind(&SomeIpPublisher::publish, this, std::placeholders::_1));
    }

private:

    void publish(const GpsDataMsg & message) {
        RCLCPP_INFO(this->get_logger(), "Publishing SOME/IP message on topic %s", topic_.c_str());
        publisher->publish(message);
    }

private:
    std::string topic_;
    SomeIpClient someip_client;

    rclcpp::Publisher<GpsDataMsg>::SharedPtr publisher;
};
