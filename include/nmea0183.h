#ifndef _NMEA0183_PARSE_H_
#define _NMEA0183_PARSE_H_

#include <cstdint>
#include <stdint.h>

#include <iostream>
#include <memory>

namespace NMEA {

/* NMEA0183 data segment: UTC time. */
typedef struct _nmea_utc_time_st
{    
    uint8_t  hour;
    uint8_t   min;
    uint8_t   sec;
    uint32_t msec;
    
}nmea_utc_time_st, *nmea_utc_time_st_ptr;

#define NMEA_UTC_TIME_ST_LEN    (sizeof(nmea_utc_time_st))

/* NMEA0183 data segment: UTC date. */
typedef struct _nmea_utc_date_st
{    
    uint16_t year;
    uint8_t mon;
    uint8_t day;
    
}nmea_utc_date_st, *nmea_utc_date_st_ptr;

#define NMEA_UTC_DATE_ST_LEN    (sizeof(nmea_utc_date_st))


/* NMEA0183 data segment: Gps quality indicator. */
typedef enum _nmea_gps_quality_em
{
    GPS_QUALITY_NO_FIX         = 0,
    GPS_QUALITY_AUTO_GNSS_FIX  = 1,
    GPS_QUALITY_DIFF_GNSS_FIX  = 2,
    //GPS_QUALITY_PPS_VALID     = 3,
    GPS_QUALITY_RTK_FIX        = 4,
    GPS_QUALITY_RTK_FLOAT      = 5,
    GPS_QUALITY_ESTIMATED_FIX  = 6,
    GPS_QUALITY_MANUAL_INPUT   = 7,
    GPS_GUALITY_SIMULATOR      = 8
    
}nmea_gps_quality_em, * nmea_gps_quality_em_ptr;

#define NMEA_GPS_QUALITY_EM_LEN    (sizeof(nmea_gps_quality_em))

/* NMEA0183 data segment: Gps posMode indicator. */
typedef enum _nmea_gps_posMode_em
{
    GPS_POSMODE_NO_FIX         = 0,
    GPS_POSMODE_AUTO_GNSS_FIX  = 1,
    GPS_POSMODE_DIFF_GNSS_FIX  = 2,
    GPS_POSMODE_RTK_FIX        = 4,
    GPS_POSMODE_RTK_FLOAT      = 5,
    GPS_POSMODE_ESTIMATED_FIX  = 6
    
}nmea_gps_posMode_em, * nmea_gps_posMode_em_ptr;

#define NMEA_GPS_POSMODE_EM_LEN    (sizeof(nmea_gps_posMode_em))

/* NMEA0183 data segment: Gps status indicator. */
typedef enum _nmea_gps_status_em
{
    GPS_STATUS_DATA_INVALID       = 0,
    GPS_STATUS_DATA_VALID         = 1
    
}nmea_gps_status_em, * nmea_gps_status_em_ptr;

#define NMEA_GPS_STATUS_EM_LEN    (sizeof(nmea_gps_status_em))

/* NMEA0183 data segment: Latitude N/S. */
typedef enum _nmea_latitude_em
{
    LATITUDE_N = 0,   /* North. */
    LATITUDE_S = 1    /* South. */
    
}nmea_latitude_em, * nmea_latitude_em_ptr;

#define NMEA_LATITUDE_EM_LEN    (sizeof(nmea_latitude_em))


/* NMEA0183 data segment: Longitude E/W. */
typedef enum _nmea_longitude_em
{
    LONGITUDE_E = 0,   /* East. */
    LONGITUDE_W = 1    /* West. */
    
}nmea_longitude_em, * nmea_longitude_em_ptr;

#define NMEA_LONGITUDE_EM_LEN    (sizeof(nmea_longitude_em))


/* NMEA0183 frame type. */
typedef enum _nmea_frametype_em
{
    NMEA_FRAME_GGA = 0,
    NMEA_FRAME_GSA = 1,
    NMEA_FRAME_RMC = 2
    
}nmea_frametype_em, * nmea_frametype_em_ptr;

#define NMEA_FRAMETYPE_EM_LEN    (sizeof(nmea_frametype_em))


/* NMEA0183 frame segment status: GxGGA. */
typedef struct _nmea_GxGGA_opt_st
{
    uint8_t               name :1;  /* Frame name. */
    
    uint8_t           utc_time :1;
    
    uint8_t           latitude :1;  /* Unit: degree. */
    uint8_t             lat_ns :1;
    uint8_t          longitude :1;  /* Unit: degree. */
    uint8_t             lon_ew :1;
    
    uint8_t             gps_qa :1;
    uint8_t      satellite_num :1;
    
    uint8_t               hdop :1;  /* Horizontal dilution of precision */
    uint8_t           altitude :1;  /* Unit: metre. */
    uint8_t geoidal_separation :1;  /* Unit: metre. */
    
    uint8_t      diff_data_age :1;  /* Unit: second. */
    uint8_t    diff_station_id :1;  /* Range: 0000 - 1023. */
    uint8_t          check_sum :1;
    
}nmea_GxGGA_opt_st, * nmea_GxGGA_opt_st_ptr;

#define NMEA_GXGGA_OPT_ST_LEN    (sizeof(nmea_GxGGA_opt_st))


/* NMEA0183 frame: GxGGA. */
typedef struct _nmea_GxGGA_st
{
    nmea_GxGGA_opt_st      opt;  /* Optional status. */
    
    char               name[7];  /* Frame name. */
    
    nmea_utc_time_st       utc_time;
    
    double            latitude;  /* Unit: degree. */
    nmea_latitude_em    lat_ns;
    double           longitude;  /* Unit: degree. */
    nmea_longitude_em   lon_ew;
    
    nmea_gps_quality_em gps_qa;
    uint8_t      satellite_num;
    
    double                hdop;  /* Horizontal dilution of precision */
    double            altitude;  /* Unit: metre. */
    double  geoidal_separation;  /* Unit: metre. */
    
    double       diff_data_age;  /* Unit: second. */
    uint16_t   diff_station_id;  /* Range: 0000 - 1023. */
    uint16_t         check_sum;  /* ASCII*/
    
}nmea_GxGGA_st, * nmea_GxGGA_st_ptr;

#define NMEA_GXGGA_ST_LEN    (sizeof(nmea_GxGGA_st))

/* NMEA0183 frame segment status: GxGSA. */
typedef struct _nmea_GxGSA_opt_st
{
    uint8_t                  name :1; /* Frame name. */
    uint8_t            op_mode :1;  
    uint8_t           nav_mode :1;
    
    uint8_t      satellite_id1 :1;  
    uint8_t      satellite_id2 :1;
    uint8_t      satellite_id3 :1;
    uint8_t      satellite_id4 :1;
    uint8_t      satellite_id5 :1;
    uint8_t      satellite_id6 :1;
    uint8_t      satellite_id7 :1;
    uint8_t      satellite_id8 :1;
    uint8_t      satellite_id9 :1;
    uint8_t     satellite_id10 :1;
    uint8_t     satellite_id11 :1;
    uint8_t     satellite_id12 :1;
    uint8_t               pdop :1;
    uint8_t               hdop :1;
    uint8_t               vdop :1;
    uint8_t          system_id :1;  /* NMEA v4.1 and above only */
    uint8_t          check_sum :1;
    
}nmea_GxGSA_opt_st, * nmea_GxGSA_opt_st_ptr;

#define NMEA_GXGSA_OPT_ST_LEN    (sizeof(_nmea_GxGSA_opt_st))

/* NMEA0183 data segment: Gps opMode indicator. */
typedef enum _nmea_gps_opMode_em
{
    GPS_OPMODE_MANUAL_2D_3D  = 0,
    GPS_OPMODE_AUTO_2D_3D    = 1
    
}nmea_gps_opMode_em, * nmea_gps_opMode_em_ptr;

#define NMEA_GPS_OPMODE_EM_LEN    (sizeof(_nmea_gps_opMode_em))

/* NMEA0183 data segment: Gps navMode indicator. */
typedef enum _nmea_gps_navMode_em
{
    GPS_NAVMODE_NOT_AVAILABLE  = 0,
    GPS_NAVMODE_2D_FIX         = 1,
    GPS_NAVMODE_3D_FIX         = 2
    
}nmea_gps_navMode_em, * nmea_gps_navMode_em_ptr;

#define NMEA_GPS_NAVMODE_EM_LEN    (sizeof(_nmea_gps_navMode_em))

/* NMEA0183 data segment: Gps navStatus indicator. */
typedef enum _nmea_gps_navStatus_em
{
    GPS_NAVSTATUS_NOT_PROVID  = 0
  
}nmea_gps_navStatus_em, * nmea_gps_navStatus_em_ptr;

#define NMEA_GPS_NAVSTATUS_EM_LEN    (sizeof(_nmea_gps_navStatus_em))

/* NMEA0183 frame: GxGSA. */
typedef struct _nmea_GxGSA_st
{
    nmea_GxGSA_opt_st        opt;  /* Optional status. */
    char                 name[7];  /* Frame name. */
    nmea_gps_opMode_em   op_mode;
    nmea_gps_navMode_em nav_mode;
    uint8_t        satellite_id[12];
    double                  pdop;  /* Position dilution of precision*/
    double                  hdop;
    double                  vdop;
    uint8_t            system_id;   
    uint16_t           check_sum;
    
}nmea_GxGSA_st, * nmea_GxGSA_st_ptr;

#define NMEA_GXGSA_ST_LEN    (sizeof(nmea_GxGSA_st))

/* NMEA0183 frame segment status: GxRMC. */
typedef struct _nmea_GxRMC_opt_st
{
    uint8_t                  name :1; /* Frame name. */
    uint8_t           utc_time :1;  
    uint8_t             status :1;
    
    uint8_t           latitude :1;  /* Unit: degree. */
    uint8_t             lat_ns :1;
    uint8_t          longitude :1;  /* Unit: degree. */
    uint8_t             lon_ew :1;
    uint8_t              speed :1;
    uint8_t                course :1;
    uint8_t           utc_date :1;
    uint8_t                 mv :1;
    uint8_t              mv_ew :1;  
    uint8_t              pos_mode :1;  /*NMEA v2.3 and above only*/
    uint8_t            nav_status :1;  /*NMEA v4.1 and above only*/
    uint8_t          check_sum :1;
    
}nmea_GxRMC_opt_st, * nmea_GxRMC_opt_st_ptr;

#define NMEA_GXRMC_OPT_ST_LEN    (sizeof(_nmea_GxRMC_opt_st))


/* NMEA0183 frame: GxRMC. */
typedef struct _nmea_GxRMC_st
{
    nmea_GxRMC_opt_st             opt;
    char                     name[7];  /* Frame name. */
    nmea_utc_time_st        utc_time;
    nmea_gps_status_em        status;
    double                  latitude;  /* Unit: degree. */
    nmea_latitude_em          lat_ns;
    double                 longitude;  /* Unit: degree. */
    nmea_longitude_em         lon_ew;
    double                       speed;
    double                      course;
    nmea_utc_date_st        utc_date;
    uint8_t                          mv;
    uint8_t                        mvEW;
    nmea_gps_posMode_em     pos_mode;
    nmea_gps_navStatus_em nav_status;
    uint16_t               check_sum;
    
}nmea_GxRMC_st, * nmea_GxRMC_st_ptr;

#define NMEA_GXRMC_ST_LEN    (sizeof(nmea_GxRMC_st))


/* NMEA0183 frame model: GxXXX. */
typedef struct _nmea_GxXXX_st
{
    uint8_t content[150];  /* Frame common content. */
    
}nmea_GxXXX_st, * nmea_GxXXX_st_ptr;

#define NMEA_GXXXX_ST_LEN    (sizeof(nmea_GxXXX_st))



class NMEA0183 {
public:
    NMEA0183();
    ~NMEA0183();

    NMEA0183(const NMEA0183 &) = delete;
    NMEA0183(const NMEA0183 &&) = delete;
    NMEA0183 &operator= (const NMEA0183 &) = delete;
    NMEA0183 &operator= (const NMEA0183 &&) = delete;

    int parse(uint8_t *data_ptr, uint16_t data_len);
    nmea_frametype_em get_frametype() const;
    std::shared_ptr<nmea_GxRMC_st> get_nmea_rmc_data() const;
    std::shared_ptr<nmea_GxGSA_st> get_nmea_gsa_data() const;
    std::shared_ptr<nmea_GxGGA_st> get_nmea_gga_data() const;
private:
    nmea_frametype_em frame_type_;
    std::shared_ptr<nmea_GxRMC_st> rmc_ptr_;
    std::shared_ptr<nmea_GxGSA_st> gsa_ptr_;
    std::shared_ptr<nmea_GxGGA_st> gga_ptr_;

    int check_xor(uint8_t *data_ptr, uint16_t length);
    void hex2ascii(uint8_t *ucp_hex, uint8_t *ucp_ascii, uint8_t len_asc);
    int parse_navigation_status(uint8_t *start_ptr, uint8_t *end_ptr, nmea_gps_navStatus_em *navstatus_ptr);
    int parse_position_mode(uint8_t *start_ptr, uint8_t *end_ptr, nmea_gps_posMode_em *posmode_ptr);
    int parse_status(uint8_t *start_ptr, uint8_t *end_ptr, nmea_gps_status_em *status_ptr);
    int parse_navigation_mode(uint8_t *start_ptr, uint8_t *end_ptr, nmea_gps_navMode_em *navmode_ptr);
    int parse_operation_mode(uint8_t *start_ptr, uint8_t *end_ptr, nmea_gps_opMode_em *opmode_ptr);
    int parse_quality(uint8_t *start_ptr, uint8_t *end_ptr, nmea_gps_quality_em *quality_ptr);
    int parse_longitude_ew(uint8_t *start_ptr, uint8_t *end_ptr, nmea_longitude_em *ew_ptr);
    int parse_longitude(uint8_t *start_ptr, uint8_t *end_ptr, double *longitude_ptr);
    int parse_latitude_ns(uint8_t *start_ptr, uint8_t *end_ptr, nmea_latitude_em *ns_ptr);
    int parse_latitude(uint8_t *start_ptr, uint8_t *end_ptr, double *latitude_ptr);
    int parse_utc_date(uint8_t *start_ptr, uint8_t *end_ptr, nmea_utc_date_st_ptr utc_ptr);
    int parse_utc_time(uint8_t *start_ptr, uint8_t *end_ptr, nmea_utc_time_st_ptr utc_ptr);

    int parse_GxRMC(nmea_GxRMC_st_ptr rmc_ptr, uint8_t *data_ptr, uint16_t len);
    int parse_GxGGA(nmea_GxGGA_st_ptr gga_ptr, uint8_t *data_ptr, uint16_t len);
    int parse_GxGSA(nmea_GxGSA_st_ptr gsa_ptr, uint8_t *data_ptr, uint16_t len);
};

} // namespace nmea

#endif
