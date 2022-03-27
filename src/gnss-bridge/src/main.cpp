#include <iostream>

#include <boost/log/core.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>

#include <boost/asio/io_service.hpp>

#include <CommonAPI/CommonAPI.hpp>
#include <v0/gnss/TimeServerProxy.hpp>

namespace logging = boost::log;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
namespace keywords = boost::log::keywords;

auto main() -> int 
{
    using namespace logging::trivial;
    src::severity_logger< severity_level > lg;

    BOOST_LOG_SEV(lg, trace) << "GNSS Client " << GNSS_CLIENT_VERSION;

    try
    {
        boost::asio::io_service io_service;

        auto runtime = CommonAPI::Runtime::get();

        auto timeServerProxy = runtime->buildProxy<v0::gnss::TimeServerProxy>("local","TimeServer");

        if(timeServerProxy == nullptr) {
            BOOST_LOG_SEV(lg, trace) << "GNSS client proxy is null";
            return 1;
        }

        CommonAPI::AvailabilityStatus availabilityStatus = CommonAPI::AvailabilityStatus::UNKNOWN;
        std::promise<CommonAPI::AvailabilityStatus> availabilityStatusPromise;
        
        auto availabilityStatusFuture = availabilityStatusPromise.get_future();
        timeServerProxy->getProxyStatusEvent().subscribe([&](CommonAPI::AvailabilityStatus status)
        {
            BOOST_LOG_SEV(lg, trace) << "Receiving interface " 
                                     << timeServerProxy->getInterface() 
                                     << " availability status= " 
                                     << static_cast<uint16_t>(status);

            if (status == CommonAPI::AvailabilityStatus::AVAILABLE)
            {
                availabilityStatusPromise.set_value(status);
            }
        });

        std::future_status futureStatus = availabilityStatusFuture.wait_for(std::chrono::seconds(3));
        
        if (futureStatus == std::future_status::ready) {
            availabilityStatus = availabilityStatusFuture.get();

            timeServerProxy->getNowEvent().subscribe([&](const ::v0::gnss::common::Time & time) {
                BOOST_LOG_SEV(lg, trace) << "time event";
            });
        } else {
            BOOST_LOG_SEV(lg, trace) << "proxy not available, exiting..." << std::endl;
            return 2;
        }

        while(true)
        {
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
