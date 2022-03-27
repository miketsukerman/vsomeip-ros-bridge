#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <CommonAPI/CommonAPI.hpp>
#include <v0/gnss/TimeServerProxy.hpp>

#include <nlohmann/json.hpp>

namespace Types {

// this template will handle all simple numeric types 
// TODO: we might need to specialise this more specifically
template <typename T> 
auto to_string(const T & t) -> std::string {
    std::stringstream strstream; 
    
    strstream << t;

    return strstream.str();
}

template <> 
auto to_string(const v0::gnss::common::Time & time) -> std::string {

    using json = nlohmann::json;

    std::stringstream strstream; 

    auto j = json({});
    auto message = std_msgs::msg::String();

    j["gnss"] = {
        {"time", 
            {"hours", time.getHours()},
            {"minutes", time.getMinutes()},
            {"seconds", time.getSeconds()} 
        }
    };

    strstream << j;

    return strstream.str();
}

template <typename T>
auto to_message(const T & t) -> std_msgs::msg::String {
    return std_msgs::msg::String();
}

template <>
auto to_message(const v0::gnss::common::Time & time) -> std_msgs::msg::String {
    
    auto message = std_msgs::msg::String();

    message.data = to_string(time);

    return message;
}

} // namespace Types

class AbstractSomeIpClient 
{
public: 
    virtual bool initialised() = 0;
    virtual std::optional<bool> available() = 0;
};

class TimeSomeIpClient : public AbstractSomeIpClient
{
    static constexpr auto domain = "local";
    static constexpr auto instance = "TimeServer";

    using MessageCallback = std::function<void(const std_msgs::msg::String & message)>;

public:
    TimeSomeIpClient() : 
          someip_proxy(CommonAPI::Runtime::get()->buildProxy<v0::gnss::TimeServerProxy>(domain,instance)) 
        , availability_status(CommonAPI::AvailabilityStatus::UNKNOWN) 
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
            //TODO: handle error case correctly
            return;
        }

        // auto availabilityStatusFuture = availability_status_promise.get_future();
        someip_proxy->getProxyStatusEvent().subscribe(std::bind(&TimeSomeIpClient::onAvailable, this, std::placeholders::_1));

        // std::future_status futureStatus = availabilityStatusFuture.wait_for(std::chrono::seconds(3));
        
        // if (futureStatus == std::future_status::ready) {
        //     availability_status = availabilityStatusFuture.get();
        // } else {
        //     //TODO: handle error case properly
        // }
    }

    void onAvailable(CommonAPI::AvailabilityStatus status) {
        // first event is always CommonAPI::AvailabilityStatus::NOT_AVAILABLE

        if (status == CommonAPI::AvailabilityStatus::AVAILABLE)
        {
            availability_status_promise.set_value(status);

            someip_proxy->getNowEvent().subscribe([this](const ::v0::gnss::common::Time & time) {

                auto message = Types::to_message(time);

                message_callback(message);
            });
        }    

    }

private:
    CommonAPI::AvailabilityStatus availability_status;
    std::promise<CommonAPI::AvailabilityStatus> availability_status_promise;

    MessageCallback message_callback;

    std::shared_ptr<v0::gnss::TimeServerProxy<>> someip_proxy;
};

class GnssSomeIpPublisher : public rclcpp::Node
{
    static constexpr auto Topic = "GNSS";
    static constexpr auto QoS = 10;

public:
    GnssSomeIpPublisher() 
        : Node("GNSS_SOMEIP_Bridge")
    {
        publisher = this->create_publisher<std_msgs::msg::String>(Topic, QoS);

        someip_client.setMessageCallback(std::bind(&GnssSomeIpPublisher::publish, this, std::placeholders::_1));
    }

private:

    void publish(const std_msgs::msg::String & message) {        

        RCLCPP_INFO(this->get_logger(), "Publishing to the GNSS topic");
        RCLCPP_INFO(this->get_logger(), message.data.c_str());

        publisher->publish(message);
    }

private:
    TimeSomeIpClient someip_client;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
};


auto main(int argc, char **argv) -> int 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GnssSomeIpPublisher>());
    rclcpp::shutdown();
    return 0;
}