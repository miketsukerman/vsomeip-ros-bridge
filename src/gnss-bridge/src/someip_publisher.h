#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <gnss_someip_lib/msg/gnss_data.hpp>

using GpsDataMsg = gnss_someip_lib::msg::GnssData;
using GnssData = v0::gnss::common::GnssData;


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
