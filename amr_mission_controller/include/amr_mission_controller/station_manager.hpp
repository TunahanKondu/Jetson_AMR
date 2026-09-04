#ifndef AMR_MISSION_CONTROLLER__STATION_MANAGER_HPP_
#define AMR_MISSION_CONTROLLER__STATION_MANAGER_HPP_

#include <string>
#include <vector>

#include "amr_mission_controller/station_types.hpp"


class StationManager
{
public:

    bool loadStations(
        const std::string &filePath);


    const StationInfo *findStationByName(
        const std::string &stationName) const;


    const StationInfo *findStationByQrCode(
        const std::string &qrCode) const;


    const std::vector<StationInfo> &
    stations() const;


private:

    std::vector<StationInfo>
        stations_;
};


#endif
