#pragma once 

#include <v0/gnss/common.hpp>

#include <gnss_someip_lib/msg/gnss_data.hpp>

#include <libgpsmm.h>

using namespace std::chrono;
using GnssDataMsg = gnss_someip_lib::msg::GnssData;
using GnssData = v0::gnss::common::GnssData;

namespace Types::Conversion {

GnssData to_gnss_data(const GnssDataMsg & gps_data) {
    GnssData gnss_data;

    v0::gnss::common::Position position;
    v0::gnss::common::Time time;
    v0::gnss::common::Fix fix; 
    v0::gnss::common::Dop dop;

    fix.setLatitude(gps_data.position.fix.latitude);
    fix.setLongitude(gps_data.position.fix.longitude);
    
    dop.setHdop(gps_data.position.dop.hdop);
    dop.setVdop(gps_data.position.dop.vdop);
    dop.setPdop(gps_data.position.dop.pdop);

    position.setSatellites_visible(gps_data.position.satellites_visible);
    position.setSatellites_used(gps_data.position.satellites_used);
    position.setDop(dop);
    position.setFix(fix);

    gnss_data.setPosition(position);
    gnss_data.setTime(time);

    return gnss_data;
}

GnssDataMsg to_gps_data(const GnssData & gnss_data) {
    
    GnssDataMsg gps_data_msg; 

    auto position = gnss_data.getPosition();

    gps_data_msg.position.fix.latitude = position.getFix().getLatitude();
    gps_data_msg.position.fix.longitude = position.getFix().getLongitude();
    gps_data_msg.position.dop.hdop = position.getDop().getHdop();
    gps_data_msg.position.dop.vdop = position.getDop().getVdop();
    gps_data_msg.position.satellites_visible = position.getSatellites_visible();
    gps_data_msg.position.satellites_used = position.getSatellites_used();

    return gps_data_msg;
}


} // namespace TypeConversion
