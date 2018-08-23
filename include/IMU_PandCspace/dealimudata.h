//
// Created by jacky on 18-6-20.
//

#ifndef PRODUCEANDCONSUME_DEALIMUDATA_H
#define PRODUCEANDCONSUME_DEALIMUDATA_H
#include <condition_variable>
#include <thread>
#include <time.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
using namespace ::boost::asio;
#include "imudata.h"
#include "cartographer_ros/node.h"
//#include <fstream>
//#define IMU_DEBUG_OUT
#define IMU_DATA_OUT_FILE

namespace IMU_PandCspace
{

struct IMU_ItemRepository
{
    IMUMessage item_buffer[kIMUItemRepositorySize]; // 产品缓冲区, 配合 read_position 和 write_position 模型环形队列.
    size_t read_position; // 消费者读取产品位置.
    size_t write_position; // 生产者写入产品位置.
    std::mutex mtx; // 互斥量,保护产品缓冲区
    std::condition_variable repo_not_full; // 条件变量, 指示产品缓冲区不为满.
    std::condition_variable repo_not_empty; // 条件变量, 指示产品缓冲区不为空.
} gIMUItemRepository; // 产品库全局变量, 生产者和消费者操作该变量.
typedef struct IMU_ItemRepository IMUItemRepository;

void
IMU_DataUnpack(uint8_t *IMU_Array,
               int16_t *gyro_x,
               int16_t *gyro_y,
               int16_t *gyro_z,
               int16_t *accel_x,
               int16_t *accel_y,
               int16_t *accel_z)
{
    int16_t temp;

    //角速度X轴数值提取
    temp = *(IMU_Array + 3);
    temp = (temp << 8) | (*(IMU_Array + 2));
    *gyro_x = temp;
    //角速度Y轴数值提取
    temp = *(IMU_Array + 5);
    temp = (temp << 8) | (*(IMU_Array + 4));
    *gyro_y = temp;
    //角速度Z轴数值提取
    temp = *(IMU_Array + 7);
    temp = (temp << 8) | (*(IMU_Array + 6));
    *gyro_z = temp;

    //加速度X轴数值提取
    temp = *(IMU_Array + 9);
    temp = (temp << 8) | (*(IMU_Array + 8));
    *accel_x = temp;
    //加速度Y轴数值提取
    temp = *(IMU_Array + 11);
    temp = (temp << 8) | (*(IMU_Array + 10));
    *accel_y = temp;
    //加速度Z轴数值提取
    temp = *(IMU_Array + 13);
    temp = (temp << 8) | (*(IMU_Array + 12));
    *accel_z = temp;
}

void
ZeroDrift_DataUnpack(uint8_t *ZeroDrift_Array, float *DriftGyro_x, float *DriftGyro_y, float *DriftGyro_z)
{
    //定义一个共用体，用于串口数据转换为浮点类型数据
    union Gyro
    {
        float d;
        uint8_t data[4];
    } G1;

    //陀螺仪X轴角速度零飘数值提取
    G1.data[0] = *(ZeroDrift_Array + 2);
    G1.data[1] = *(ZeroDrift_Array + 3);
    G1.data[2] = *(ZeroDrift_Array + 4);
    G1.data[3] = *(ZeroDrift_Array + 5);
    *DriftGyro_x = G1.d;

    //陀螺仪Y轴角速度零飘数值提取
    G1.data[0] = *(ZeroDrift_Array + 6);
    G1.data[1] = *(ZeroDrift_Array + 7);
    G1.data[2] = *(ZeroDrift_Array + 8);
    G1.data[3] = *(ZeroDrift_Array + 9);
    *DriftGyro_y = G1.d;

    //陀螺仪Z轴角速度零飘数值提取
    G1.data[0] = *(ZeroDrift_Array + 10);
    G1.data[1] = *(ZeroDrift_Array + 11);
    G1.data[2] = *(ZeroDrift_Array + 12);
    G1.data[3] = *(ZeroDrift_Array + 13);
    *DriftGyro_z = G1.d;
}

uint8_t
AnalyeDataFromHost(unsigned char UnpackBuffer[])
{
    uint8_t i, length, temp;
    //起始位匹配变量
    uint16_t InitialBytes;

    //数据帧异或校验
    temp = UnpackBuffer[13];
    for (i = 0; i < 13; i++)
    {
        temp ^= UnpackBuffer[i];
    }
    if (temp == UnpackBuffer[14])
    {
        //数据帧起始码匹配
        InitialBytes = UnpackBuffer[0];
        InitialBytes = (InitialBytes << 8) | UnpackBuffer[1];
        switch (InitialBytes)
        {
//接收到IMU数据帧，解析并返回1
            case 0xfafa:
            {
//	           IMU_DataUnpack(UnpackBuffer,&IMU_GyroX,&IMU_GyroY,&IMU_GyroZ,&IMU_AccelX,&IMU_AccelY,&IMU_AccelZ);
                return 1;
            }
//接收到Location数据帧，解析并返回2
            case 0xfbfb:
            {
                return 2;
            }
//接收到ZeroDrift数据帧，解析并返回3
            case 0xfcfc:
            {
//					  ZeroDrift_DataUnpack(UnpackBuffer,&DriftGyro_X,&DriftGyro_Y,&DriftGyro_Z);
                return 3;

            }
            default: return 0;
        }

    }
    else
        return 0;
}

const float NUM[3] = {
    0.02008336556421123561544384017452102853,
    0.04016673112842247123088768034904205706,
    0.02008336556421123561544384017452102853
};
const int DL = 3;
const float DEN[3] = {
    1,
    -1.561018075800718163392843962355982512236,
    0.641351538057563175243558362126350402832
};

float Gyro_x_configure(float *Gyro_x)
{
    static float Data_In_gx[3]  =  {0,0,0};
    static float Data_Out_gx[3]  = {0,0,0};
    static float last_gx=0;
    if(*Gyro_x>3||*Gyro_x<-3)
    {
        *Gyro_x = last_gx;
    }
    Data_In_gx[0]  =*Gyro_x;
    Data_Out_gx[0] =-DEN[1]*Data_Out_gx[1]
        -DEN[2]*Data_Out_gx[2]
        +NUM[0]*Data_In_gx[0]
        +NUM[1]*Data_In_gx[1]
        +NUM[2]*Data_In_gx[2];
    Data_Out_gx[2] = Data_Out_gx[1];
    Data_Out_gx[1] = Data_Out_gx[0];
    Data_In_gx[2]  = Data_In_gx[1];
    Data_In_gx[1]  = Data_In_gx[0];
    *Gyro_x = Data_Out_gx[0];
    last_gx = *Gyro_x;
    return *Gyro_x;
}

float Gyro_y_configure(float *Gyro_y)
{
    static float Data_In_gy[3]  =  {0,0,0};
    static float Data_Out_gy[3]  = {0,0,0};
    static float last_gy=0;
    if(*Gyro_y>3||*Gyro_y<-3)
    {
        *Gyro_y = last_gy;
    }
    Data_In_gy[0]=*Gyro_y;
    Data_Out_gy[0] =-DEN[1]*Data_Out_gy[1]
        -DEN[2]*Data_Out_gy[2]
        +NUM[0]*Data_In_gy[0]
        +NUM[1]*Data_In_gy[1]
        +NUM[2]*Data_In_gy[2];
    Data_Out_gy[2] = Data_Out_gy[1];
    Data_Out_gy[1] = Data_Out_gy[0];
    Data_In_gy[2]  = Data_In_gy[1];
    Data_In_gy[1]  = Data_In_gy[0];
    *Gyro_y = Data_Out_gy[0];
    last_gy = *Gyro_y;
    return *Gyro_y;
}

float Gyro_z_configure(float *Gyro_z)
{
    static float Data_In_gz[3]  =  {0,0,0};
    static float Data_Out_gz[3]  = {0,0,0};
    static float last_gz=0;
    if(*Gyro_z>10||*Gyro_z<-10)
    {
        *Gyro_z = last_gz;
    }
    Data_In_gz[0]=*Gyro_z;
    Data_Out_gz[0] =-DEN[1]*Data_Out_gz[1]
        -DEN[2]*Data_Out_gz[2]
        +NUM[0]*Data_In_gz[0]
        +NUM[1]*Data_In_gz[1]
        +NUM[2]*Data_In_gz[2];
    Data_Out_gz[2] = Data_Out_gz[1];
    Data_Out_gz[1] = Data_Out_gz[0];
    Data_In_gz[2]  = Data_In_gz[1];
    Data_In_gz[1]  = Data_In_gz[0];
    *Gyro_z = Data_Out_gz[0];
    last_gz = *Gyro_z;
    return *Gyro_z;
}

float Acc_x_configure(float *Acc_x)
{
    static float Data_In_ax[3]  =  {0,0,0};
    static float Data_Out_ax[3]  = {0,0,0};
    static float last_ax=0;
    if(*Acc_x>20||*Acc_x<-20)
    {
        *Acc_x = last_ax;
    }
    Data_In_ax[0]=*Acc_x;
    Data_Out_ax[0] =-DEN[1]*Data_Out_ax[1]
        -DEN[2]*Data_Out_ax[2]
        +NUM[0]*Data_In_ax[0]
        +NUM[1]*Data_In_ax[1]
        +NUM[2]*Data_In_ax[2];
    Data_Out_ax[2] = Data_Out_ax[1];
    Data_Out_ax[1] = Data_Out_ax[0];
    Data_In_ax[2]  = Data_In_ax[1];
    Data_In_ax[1]  = Data_In_ax[0];
    *Acc_x = Data_Out_ax[0];
    last_ax = *Acc_x;
    return *Acc_x;
}

float Acc_y_configure(float *Acc_y)
{
    static float Data_In_ay[3]  =  {0,0,0};
    static float Data_Out_ay[3]  = {0,0,0};
    static float last_ay=0;
    if(*Acc_y>20||*Acc_y<-20)
    {
        *Acc_y = last_ay;
    }
    Data_In_ay[0]=*Acc_y;
    Data_Out_ay[0] =-DEN[1]*Data_Out_ay[1]
        -DEN[2]*Data_Out_ay[2]
        +NUM[0]*Data_In_ay[0]
        +NUM[1]*Data_In_ay[1]
        +NUM[2]*Data_In_ay[2];
    Data_Out_ay[2] = Data_Out_ay[1];
    Data_Out_ay[1] = Data_Out_ay[0];
    Data_In_ay[2]  = Data_In_ay[1];
    Data_In_ay[1]  = Data_In_ay[0];
    *Acc_y = Data_Out_ay[0];
    last_ay = *Acc_y;
    return *Acc_y;
}

float Acc_z_configure(float *Acc_z)
{
    static float Data_In_az[3]  =  {0,0,0};
    static float Data_Out_az[3]  = {0,0,0};
    static float last_az=0;
    if(*Acc_z>35||*Acc_z<-5)
    {
        *Acc_z = last_az;
    }
    Data_In_az[0]=*Acc_z;
    Data_Out_az[0] =-DEN[1]*Data_Out_az[1]
        -DEN[2]*Data_Out_az[2]
        +NUM[0]*Data_In_az[0]
        +NUM[1]*Data_In_az[1]
        +NUM[2]*Data_In_az[2];
    Data_Out_az[2] = Data_Out_az[1];
    Data_Out_az[1] = Data_Out_az[0];
    Data_In_az[2]  = Data_In_az[1];
    Data_In_az[1]  = Data_In_az[0];
    *Acc_z = Data_Out_az[0];
    last_az = *Acc_z;
    return *Acc_z;
}

enum
{
    BMI055_DATA_MAX = 32767,
    BMI055_DATA_MIN = -32768,
    BMI055_G_MAX = 4,
    BMI055_G_MIN = -4,
    BMI055_DPS_MAX = 2000,
    BMI055_DPS_MIN = -2000,
};
//double M_PI = 3.14159;
static const double G_MPSS = 9.80665;

void
createimumsgfun(IMUMessage *imsg,
                float IMU_fGyroX,
                float IMU_fGyroY,
                float IMU_fGyroZ,
                float IMU_fAccelX,
                float IMU_fAccelY,
                float IMU_fAccelZ)
{
    if (IMU_fGyroX > 0)
    {
        IMU_fGyroX = (IMU_fGyroX * BMI055_DPS_MAX * M_PI) / (180.0 * BMI055_DATA_MAX);
        Gyro_x_configure(&IMU_fGyroX);
    }
    else
    {
        IMU_fGyroX = (IMU_fGyroX * BMI055_DPS_MIN * M_PI) / (180.0 * BMI055_DATA_MIN);
        Gyro_x_configure(&IMU_fGyroX);
    }

    if (IMU_fGyroY > 0)
    {
        IMU_fGyroY = (IMU_fGyroY * BMI055_DPS_MAX * M_PI) / (180.0 * BMI055_DATA_MAX);
        Gyro_y_configure(&IMU_fGyroY);
    }
    else
    {
        IMU_fGyroY = (IMU_fGyroY * BMI055_DPS_MIN * M_PI) / (180.0 * BMI055_DATA_MIN);
        Gyro_y_configure(&IMU_fGyroY);
    }

    if (IMU_fGyroZ > 0)
    {
        IMU_fGyroZ = (IMU_fGyroZ * BMI055_DPS_MAX * M_PI) / (180.0 * BMI055_DATA_MAX);
        Gyro_z_configure(&IMU_fGyroZ);
    }
    else
    {
        IMU_fGyroZ = (IMU_fGyroZ * BMI055_DPS_MIN * M_PI) / (180.0 * BMI055_DATA_MIN);
        Gyro_z_configure(&IMU_fGyroZ);
    }

    if (IMU_fAccelX > 0)
    {
        IMU_fAccelX = (IMU_fAccelX * BMI055_G_MAX * G_MPSS) / BMI055_DATA_MAX;
        Acc_x_configure(&IMU_fAccelX);
    }
    else
    {
        IMU_fAccelX = (IMU_fAccelX * BMI055_G_MIN * G_MPSS) / BMI055_DATA_MIN;
        Acc_x_configure(&IMU_fAccelX);
    }

    if (IMU_fAccelY > 0)
    {
        IMU_fAccelY = (IMU_fAccelY * BMI055_G_MAX * G_MPSS) / BMI055_DATA_MAX;
        Acc_y_configure(&IMU_fAccelY);
    }
    else
    {
        IMU_fAccelY = (IMU_fAccelY * BMI055_G_MIN * G_MPSS) / BMI055_DATA_MIN;
        Acc_y_configure(&IMU_fAccelY);
    }

    if (IMU_fAccelZ > 0)
    {
        IMU_fAccelZ = (IMU_fAccelZ * BMI055_G_MAX * G_MPSS) / BMI055_DATA_MAX;
        Acc_z_configure(&IMU_fAccelZ);
    }
    else
    {
        IMU_fAccelZ = (IMU_fAccelZ * BMI055_G_MIN * G_MPSS) / BMI055_DATA_MIN;
        Acc_z_configure(&IMU_fAccelZ);
    }

    imsg->angular_velocity.x = IMU_fGyroX;
    imsg->angular_velocity.y = IMU_fGyroY;
    imsg->angular_velocity.z = IMU_fGyroZ;
    imsg->linear_acceleration.x = IMU_fAccelX;
    imsg->linear_acceleration.y = IMU_fAccelY;
    imsg->linear_acceleration.z = IMU_fAccelZ;

    return;
}

std::string imudev = "/dev/airobimu";
int Speed = 115200;
char tmpchar;

void
handle_read(char *buf, boost::system::error_code ec,
            std::size_t bytes_transferred)
{
    tmpchar = buf[0];
}

void
ProduceIMUItem(IMUItemRepository *ir, IMUMessage item)
{
    std::unique_lock<std::mutex> lock(ir->mtx);
    while (((ir->write_position + 1) % kIMUItemRepositorySize)
        == ir->read_position)
    { // item buffer is full, just wait here.

#ifdef IMU_DEBUG_OUT
        std::cout << "IMU_Producer is waiting for an empty slot...\n";
#endif

        (ir->repo_not_full).wait(lock); // 生产者等待"产品库缓冲区不为满"这一条件发生.
    }

    (ir->item_buffer)[ir->write_position] = item; // 写入产品.
    (ir->write_position)++; // 写入位置后移.

    if (ir->write_position == kIMUItemRepositorySize) // 写入位置若是在队列最后则重新设置为初始位置.
        ir->write_position = 0;

    (ir->repo_not_empty).notify_all(); // 通知消费者产品库不为空.
    lock.unlock(); // 解锁.
}

void
ProducerIMUTask(cartographer_ros::Node* nodeptr) // 生产者任务
{
    //for (int i = 1; i <= kItemsToProduce; ++i)
/*    int64_t i = 0;
    while(true)
    {
        // sleep(1);
        std::cout << "Produce the " << i << "^th imu item..." << std::endl;
        IMUMessage tmp_imu_message;
        ProduceItem(&gIMUItemRepository, tmp_imu_message); // 循环生产 kItemsToProduce 个产品.
        i++;
    }*/

#ifdef IMU_DATA_OUT_FILE
    std::ofstream imudataout("/home/pi/imu.data");
#endif

    if(nodeptr->imu_produce_running_.load(std::memory_order_acquire))
    {
        return;
    }
    nodeptr->imu_produce_running_.store(true, std::memory_order_release);
    nodeptr->imu_produce_threadHasStopped_.store(false, std::memory_order_release);

    io_service io_s;
    serial_port sp(io_s, imudev.data());
    if (!sp.is_open())
    {
        std::cout << "can not open imu device" << std::endl;
        nodeptr->imu_produce_threadHasStopped_.store(true, std::memory_order_release);// = true;
        return;
    }

    sp.set_option(serial_port::baud_rate(Speed));                         //比特率
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none)); //流量控制
    sp.set_option(serial_port::parity(serial_port::parity::none));            //奇偶校验
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));        //停止位
    sp.set_option(serial_port::character_size(8));                       //数据位
    char buf[1];

    bool havegetdrift = false;
    unsigned char recvdata[15];
    int16_t IMU_GyroX, IMU_GyroY, IMU_GyroZ, IMU_AccelX, IMU_AccelY, IMU_AccelZ;
    float IMU_fGyroX, IMU_fGyroY, IMU_fGyroZ;
    float DriftGyro_X, DriftGyro_Y, DriftGyro_Z;
    unsigned char Zerobuf[12];
    memset(Zerobuf, 0x00, 12);

    int buflen = 15;
    int tmplen = 13;
    unsigned char *readBuf = new unsigned char[buflen];
    bool gothead = false;
    int gotcontentlength = 0;
    char headchar;
    char lastRecved = 0x00;
    boost::system::error_code err;
    int sequence = 0;
//    while (true)
    while(nodeptr->imu_produce_running_.load(std::memory_order_relaxed) == true)
    {
        if (!havegetdrift)
            headchar = 0xfc;
        else
            headchar = 0xfa;
        do
        {
            char cRecved = 0x00;
//            async_read(sp, buffer(buf), boost::bind(handle_read, this, buf, _1, _2));
            sp.read_some(boost::asio::buffer(buf), err);
            if (!err)
            {
                cRecved = buf[0];
                if (gothead)
                {
                    readBuf[buflen - tmplen] = cRecved;
                    tmplen--;
                    gotcontentlength++;
                    if (13 == gotcontentlength)
                    {
                        readBuf[0] = headchar;
                        readBuf[1] = headchar;
                        break;
                    }
                }

                if (headchar == lastRecved)
                {
                    if (headchar == cRecved)
                    {
                        if (!gothead)
                        {
                            gothead = true;
                        }
                    }
                }
                lastRecved = cRecved;
            }
        }
        while (true);

        memcpy(recvdata, readBuf, 15);
        gothead = false;
        gotcontentlength = 0;
        tmplen = 13;
        lastRecved = 0x00;
        switch (AnalyeDataFromHost(recvdata))
        {
            case 1:
            {
                if (havegetdrift)
                {
#ifdef IMU_DEBUG_OUT
                    std::cout << "get imu data" << std::endl;
#endif
                    IMU_DataUnpack(recvdata,
                                   &IMU_GyroX,
                                   &IMU_GyroY,
                                   &IMU_GyroZ,
                                   &IMU_AccelX,
                                   &IMU_AccelY,
                                   &IMU_AccelZ);

                    IMU_fGyroX = (float) IMU_GyroX - DriftGyro_X;
                    IMU_fGyroY = (float) IMU_GyroY - DriftGyro_Y;
                    IMU_fGyroZ = (float) IMU_GyroZ - DriftGyro_Z;


                    IMUMessage imumsg;
                    createimumsgfun(&imumsg,
                                    IMU_fGyroX,
                                    IMU_fGyroY,
                                    IMU_fGyroZ,
                                    (float) IMU_AccelX,
                                    (float) IMU_AccelY,
                                    (float) IMU_AccelZ);

#ifdef IMU_DATA_OUT_FILE
                    imudataout<<imumsg.angular_velocity.x<<","<<imumsg.angular_velocity.y<<","<<imumsg.angular_velocity.z
                              <<","<<imumsg.linear_acceleration.x<<","<<imumsg.linear_acceleration.y<<","<<imumsg.linear_acceleration.z<<std::endl;
#endif

                    //imumsg.header.stamp =;
#ifdef IMU_DEBUG_OUT
                    std::cout<<"imu message sequence : "<<sequence<<std::endl;
#endif
                    timeval tv;
                    gettimeofday(&tv, 0);
                    imumsg.header.stamp = tv;
                    imumsg.header.seq = sequence;
                    sequence++;
                    imumsg.header.frame_id = "imu";
                    if(nodeptr->imu_consumer_running_.load(std::memory_order_relaxed) == true)
                    {
                        ProduceIMUItem(&gIMUItemRepository, imumsg);
                    }
                }
                break;
            }
            case 3:
            {
                if (memcmp(&recvdata[2], Zerobuf, 12))
                {
                    ZeroDrift_DataUnpack(recvdata, &DriftGyro_X, &DriftGyro_Y, &DriftGyro_Z);
                    havegetdrift = true;
                    unsigned char response[] = {0xAF, 0xAF, 0x11, 0x11, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
                    //ser.write(response, sizeof(response));   //发送串口数据
                    std::cout << "get head message" << std::endl;
                    boost::asio::write(sp, buffer(response, 10));
                }
            }
            default:break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
//        usleep(2 * 1000);
    }
    nodeptr->imu_produce_threadHasStopped_.store(true, std::memory_order_release);// = true;
    std::cout << "imu produce thread end " << std::endl;
    return;
}

IMUMessage
ConsumeIMUItem(IMUItemRepository *ir)
{
    IMUMessage data;
    std::unique_lock<std::mutex> lock(ir->mtx);
    // item buffer is empty, just wait here.
    while (ir->write_position == ir->read_position)
    {
#ifdef IMU_DEBUG_OUT
        std::cout << "IMU_Consumer is waiting for items...\n";
#endif
        (ir->repo_not_empty).wait(lock); // 消费者等待"产品库缓冲区不为空"这一条件发生.
    }

    data = (ir->item_buffer)[ir->read_position]; // 读取某一产品
    (ir->read_position)++; // 读取位置后移

    if (ir->read_position >= kIMUItemRepositorySize) // 读取位置若移到最后，则重新置位.
        ir->read_position = 0;

    (ir->repo_not_full).notify_all(); // 通知消费者产品库不为满.
    lock.unlock(); // 解锁.

    return data; // 返回产品.
}

void
ConsumerIMUTask(cartographer_ros::Node* nodeptr) // 消费者任务
{
    if(nodeptr->imu_consumer_running_.load(std::memory_order_acquire))
    {
        return;
    }
    nodeptr->imu_consumer_running_.store(true, std::memory_order_release);
    nodeptr->imu_consumer_threadHasStopped_.store(false, std::memory_order_release);

    static int cnt = 0;
//    while (1)
    while(nodeptr->imu_consumer_running_.load(std::memory_order_relaxed) == true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
//        usleep(2 * 1000);
        if(nodeptr->imu_consumer_threadHasStopped_.load(std::memory_order_relaxed) == false)
        {
            IMUMessage item = ConsumeIMUItem(&gIMUItemRepository); // 消费一个产品.
//        nodeptr->HandleImuMessage(0, "imu", std::ref(item));
            nodeptr->HandleImuMessage(0, "imu", item);
        }



//        std::cout << "Consume the " << item.header.seq << "^th item" << std::endl;
//        std::cout << "Angle : [" << item.angular_velocity.x << "," << item.angular_velocity.y << ","
//                  << item.angular_velocity.z << "]" << std::endl;
//        std::cout << "linear : [" << item.linear_acceleration.x << "," << item.linear_acceleration.y << ","
//                  << item.linear_acceleration.z << "]" << std::endl;


//        if (++cnt == kItemsToProduce)
//            break; // 如果产品消费个数为 kItemsToProduce, 则退出.
    }
    nodeptr->imu_consumer_threadHasStopped_.store(true, std::memory_order_release);
    std::cout<<"imu consumer thread end"<<std::endl;
}

void
InitIMUItemRepository(IMUItemRepository *ir)
{
    ir->write_position = 0; // 初始化产品写入位置.
    ir->read_position = 0; // 初始化产品读取位置.
}

}
#endif //PRODUCEANDCONSUME_DEALIMUDATA_H
