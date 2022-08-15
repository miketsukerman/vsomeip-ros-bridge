#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <std_msgs/msg/string.hpp>

#include <libgpsmm.h>

namespace Types::Conversion {

/**
 * @brief converts gpsd datatype gps_data_t to ros2 msg generated type
 * 
 * @param gps_data 
 * @return GnssDataMsg 
 */
GnssDataMsg to_gnss_data_msg(const gps_data_t * gps_data) {
    GnssDataMsg msg; 

    msg.position.fix.latitude = gps_data->fix.latitude;
    msg.position.fix.longitude = gps_data->fix.longitude;
    msg.position.dop.hdop = gps_data->dop.hdop;
    msg.position.dop.vdop = gps_data->dop.vdop;
    msg.position.satellites_visible = gps_data->satellites_visible;
    msg.position.satellites_used = gps_data->satellites_used;

    return msg;
}

} // namespace Types::Conversion

// inspired by https://gist.github.com/ncoder-1/8313815ac387e6757f751dc8960f03d7
class GpsdClient : public rclcpp::Node {

    static constexpr auto node_name = "GPSD_Client_node";

    static constexpr auto gpsd_host = "localhost";
    static constexpr auto waiting_time = 1000000;

    static constexpr auto topic = "GPSD";
    static constexpr auto qos = 10;

    static constexpr auto gpsd_read_timer_delay = 500ms;

public:
    GpsdClient()
        : Node(node_name),
        gps_rec(gpsd_host, DEFAULT_GPSD_PORT) 
    {
        publisher = this->create_publisher<GnssDataMsg>(topic, qos);

        if(!init()) {
            RCLCPP_WARN(this->get_logger(), "No connection to gpsd");
        }

        timer = this->create_wall_timer(gpsd_read_timer_delay, std::bind(&GpsdClient::read, this));

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
            
            if (!gps_rec.waiting(waiting_time)) {
                RCLCPP_WARN(this->get_logger(), "Waiting, failed to read data from gpsd");
                continue;
            }

            struct gps_data_t* gpsd_data;

            if ((gpsd_data = gps_rec.read()) == nullptr) {
                RCLCPP_WARN(this->get_logger(), "Null, Failed to read data from gpsd");
                continue;
            }

            RCLCPP_WARN(this->get_logger(), "Obtained data from gpsd");

            auto msg = Types::Conversion::to_gnss_data_msg(gpsd_data);
            
            publisher->publish(msg);

            return;
        }
    }

protected:

private:
    gpsmm gps_rec;

    std::mutex mutex;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<GnssDataMsg>::SharedPtr publisher;
};
