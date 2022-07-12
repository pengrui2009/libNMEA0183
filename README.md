# libNMEA0183

gps message prase library, support:

- [x] RMC
- [x] GSA
- [x] GGA

# Usage

Add below into CMakeLists.txt file:
```
find_package(libnmea0183 REQUIRED)

target_link_libraries(xxx nmea0183)
```

# example code

```
    int ret = 0;
   
    std::string gps_msg = "$GNRMC,114811.00,A,3558.94849,N,14008.90158,E,0.076,,141115,,,A*69\r\n";

    std::shared_ptr<NMEA::NMEA0183> nmea_ptr = std::make_shared<NMEA::NMEA0183>();

    ret = nmea_ptr->parse(reinterpret_cast<const uint8_t *>(gps_msg.data()), gps_msg.length());
    if (ret ==0 )
    {
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
    
    
```