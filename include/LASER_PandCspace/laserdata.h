//
// Created by jacky on 18-6-28.
//

#ifndef TESTLASER_LASERDATA_H
#define TESTLASER_LASERDATA_H

namespace LASER_PandCspace
{

static const int kLASERItemRepositorySize = 10; // Item buffer size.
static const int kLASERItemsToProduce = 1000;   // How many items we plan to produce.

struct lheaderinfo
{
    uint32_t seq;
    timeval stamp;
    std::string frame_id;
};
typedef lheaderinfo LHeaderInfo;

struct LASER_Message
{
    LHeaderInfo header;
    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float scan_time;
    float range_min;
    float range_max;
    float ranges[720];
    float intensities[720];
    int rangessize = 720;
    int intensitiessize = 720;
};
typedef LASER_Message LASERMessage;



}
#endif //TESTLASER_LASERDATA_H
