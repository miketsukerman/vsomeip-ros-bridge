#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <libgpsmm.h>

#include <chrono>
#include <memory>

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
    msg.position.dop.pdop = gps_data->dop.pdop;
    msg.position.satellites_visible = gps_data->satellites_visible;
    msg.position.satellites_used = gps_data->satellites_used;

    return msg;
}

} // namespace Types::Conversion

// inspired by https://gist.github.com/ncoder-1/8313815ac387e6757f751dc8960f03d7
class GpsdClient : public rclcpp::Node {

    static constexpr auto node_name = "GPSD_Client_node";
    static constexpr int reconnect_cycles = 10;

public:
    GpsdClient()
        : Node(node_name)
    {
        this->declare_parameter<std::string>("gpsd_host", "localhost");
        this->declare_parameter<int>("gpsd_waiting_time_us", 1000000);
        this->declare_parameter<std::string>("gpsd_topic", "GPSD");
        this->declare_parameter<int>("gpsd_qos", 10);
        this->declare_parameter<int>("gpsd_read_interval_ms", 500);

        const auto topic = this->get_parameter("gpsd_topic").as_string();
        const auto qos = static_cast<int>(this->get_parameter("gpsd_qos").as_int());
        const auto interval_ms = this->get_parameter("gpsd_read_interval_ms").as_int();

        publisher = this->create_publisher<GnssDataMsg>(topic, qos);

        connected_ = init();
        if (!connected_) {
            RCLCPP_WARN(this->get_logger(), "No connection to gpsd, will retry every %d timer cycles", reconnect_cycles);
        }

        timer = this->create_wall_timer(
            std::chrono::milliseconds(interval_ms),
            std::bind(&GpsdClient::read, this));
    }

    ~GpsdClient() = default;

    bool init() {
        const auto host = this->get_parameter("gpsd_host").as_string();
        gps_rec_ = std::make_unique<gpsmm>(host.c_str(), DEFAULT_GPSD_PORT);
        if (gps_rec_->stream(WATCH_ENABLE | WATCH_JSON) == nullptr) {
            gps_rec_.reset();
            return false;
        }
        return true;
    }

    void read() {
        if (!connected_) {
            if (++reconnect_counter_ >= reconnect_cycles) {
                reconnect_counter_ = 0;
                RCLCPP_INFO(this->get_logger(), "Attempting to reconnect to gpsd...");
                connected_ = init();
                if (connected_) {
                    RCLCPP_INFO(this->get_logger(), "Reconnected to gpsd successfully");
                }
            }
            return;
        }

        const auto waiting_time = this->get_parameter("gpsd_waiting_time_us").as_int();

        if (!gps_rec_->waiting(waiting_time)) {
            RCLCPP_DEBUG(this->get_logger(), "No data available from gpsd");
            return;
        }

        struct gps_data_t * gpsd_data;

        if ((gpsd_data = gps_rec_->read()) == nullptr) {
            RCLCPP_WARN(this->get_logger(), "Failed to read data from gpsd");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Obtained data from gpsd");

        auto msg = Types::Conversion::to_gnss_data_msg(gpsd_data);

        publisher->publish(msg);
    }

private:
    std::unique_ptr<gpsmm> gps_rec_;
    bool connected_ = false;
    int reconnect_counter_ = 0;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<GnssDataMsg>::SharedPtr publisher;
};
