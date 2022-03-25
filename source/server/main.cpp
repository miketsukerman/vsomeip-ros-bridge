#include <iostream>

#include <CommonAPI/CommonAPI.hpp>

#include <v0/gnss/TimeServerStubDefault.hpp>

class GnssTimeServer : public v0::gnss::TimeServerStubDefault {

public:
    GnssTimeServer() = default;
    ~GnssTimeServer() = default;

    void fireNowEvent() {
        v0::gnss::common::Time _time;

        _time.setHours(1);
        _time.setMinutes(1);
        _time.setSeconds(1);

        TimeServerStub::fireNowEvent(_time);
    }

};

auto main() -> int 
{
    auto runtime = CommonAPI::Runtime::get();

    auto service = std::make_shared<GnssTimeServer>();

    runtime->registerService("local","gnss.TimeServer",service);

    service->fireNowEvent();

    return 0;
}