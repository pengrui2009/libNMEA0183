#include "nmea0183.h"

#include <iostream>
#include <string>
#include <memory>
#include <unordered_map>

int main()
{
    int ret = 0;
    std::unordered_map<uint8_t, std::string> gps_message = {
        {1, "$GNRMC,114811.00,A,3558.94849,N,14008.90158,E,0.076,,141115,,,A*69\r\n"},
        {2, "$GNVTG,,T,,M,0.076,N,0.141,K,A*38\r\n"},
        {3, "$GNGGA,114811.00,3558.94849,N,14008.90158,E,1,07,1.18,31.3,M,39.3,M,,*72\r\n"},
        {4, "$GNGSA,A,3,22,24,12,15,18,14,25,,,,,,3.03,1.18,2.80*14\r\n"},
        {5, "$GNGSA,A,3,,,,,,,,,,,,,3.03,1.18,2.80*1E\r\n"},
        {6, "$GNGLL,3558.94849,N,14008.90158,E,114811.00,A,A*70\r\n"}
    };

    std::shared_ptr<NMEA::NMEA0183> nmea_ptr = std::make_shared<NMEA::NMEA0183>();

    for (auto iter : gps_message)
    {
        ret = nmea_ptr->parse(reinterpret_cast<const uint8_t *>(iter.second.data()), iter.second.length());
        if (ret)
        {
            std::cout << "ret:" << ret << std::endl;
        }
        switch(nmea_ptr->get_frametype())
        {
        case NMEA::NMEA_FRAME_RMC:
            std::cout << "frame_type: NMEA_FRAME_RMC" << std::endl;
            {
                std::shared_ptr<NMEA::nmea_GxRMC_st> rmc_ptr = nmea_ptr->get_nmea_rmc_data();
                std::cout << "latitude:" << rmc_ptr->latitude << std::endl;
                std::cout << "longitude:" << rmc_ptr->longitude << std::endl;
            }
            break;
        case NMEA::NMEA_FRAME_GSA:
            std::cout << "frame_type: NMEA_FRAME_GSA" << std::endl;
            break;
        case NMEA::NMEA_FRAME_GGA:
            std::cout << "frame_type: NMEA_FRAME_GGA" << std::endl;
            break;
        case NMEA::NMEA_FRAME_UNKNOWN:
            std::cout << "frame_type: NMEA_FRAME_UNKNOWN" << std::endl;
            break;
        }
        
    }


    return 0;
}