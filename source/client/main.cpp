#include <iostream>

#include <CommonAPI/CommonAPI.hpp>

#include <v0/gnss/TimeServerProxy.hpp>

auto main() -> int 
{
    auto runtime = CommonAPI::Runtime::get();

    auto service = runtime->buildProxy<v0::gnss::TimeServerProxy>("local","gnss.TimeServer");

    return 0;
}