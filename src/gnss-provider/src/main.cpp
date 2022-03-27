#include <iostream>

#include <boost/log/core.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

#include <boost/asio/io_service.hpp>

#include <CommonAPI/CommonAPI.hpp>
#include <v0/gnss/TimeServerStubDefault.hpp>

#include <chrono>

namespace logging = boost::log;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
namespace keywords = boost::log::keywords;

using namespace std::chrono;

using namespace logging::trivial;
src::severity_logger< severity_level > lg;

class GnssTimeServer : public v0::gnss::TimeServerStubDefault {

public:
    GnssTimeServer() = default;
    ~GnssTimeServer() = default;

    void fireNowEvent() {
        v0::gnss::common::Time time;

        auto current_time = std::chrono::system_clock::now().time_since_epoch();

        time.setHours(std::chrono::duration_cast<std::chrono::hours>(current_time).count());
        time.setMinutes(std::chrono::duration_cast<std::chrono::minutes>(current_time).count());
        time.setSeconds(std::chrono::duration_cast<std::chrono::seconds>(current_time).count());

        BOOST_LOG_SEV(lg, info) << "broadcast current time " 
                                << static_cast<int>(time.getHours()) << ":" 
                                << static_cast<int>(time.getMinutes()) << ":"
                                << static_cast<int>(time.getSeconds());

        TimeServerStub::fireNowEvent(time);
    }

};

auto main() -> int 
{
    BOOST_LOG_SEV(lg, trace) << "GNSS Server " << GNSS_SERVER_VERSION;

    try
    {
        boost::asio::io_service io_service;

        auto runtime = CommonAPI::Runtime::get();

        auto service = std::make_shared<GnssTimeServer>();

        BOOST_LOG_SEV(lg, info) << "Initialising " << GnssTimeServer::StubInterface::getInterface();

        if(!runtime->registerService("local","TimeServer", service))
        {
            BOOST_LOG_SEV(lg, error) << "failed to register GNSS SOME/IP provider " << GNSS_SERVER_VERSION;
            return 1;
        }

        BOOST_LOG_SEV(lg, info) << GnssTimeServer::StubInterface::getInterface() << " registered";

        while(true) {
            service->fireNowEvent();

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        
        io_service.run();
    }
    catch (std::exception& e)
    {
        std::printf("Exception: %s\n", e.what());
    }

    return 0;
}