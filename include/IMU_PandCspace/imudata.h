//
// Created by jacky on 18-6-28.
//

#ifndef TESTLASER_IMUDATA_H
#define TESTLASER_IMUDATA_H

namespace IMU_PandCspace
{

static const int kIMUItemRepositorySize = 10; // Item buffer size.
static const int kIMUItemsToProduce = 1000;   // How many items we plan to produce.

struct iheaderinfo
{
    uint32_t seq;
    timeval stamp;
    std::string frame_id;
};
typedef iheaderinfo IHeaderInfo;

struct Vector3
{
    float x;
    float y;
    float z;
};
typedef Vector3 Vector3;

struct IMU_Message
{
    IHeaderInfo header;

    Vector3 angular_velocity;
    float angular_velocity_covariance[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

    Vector3 linear_acceleration;
    float linear_acceleration_covariance[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
};
typedef struct IMU_Message IMUMessage;

}
#endif //TESTLASER_IMUDATA_H
