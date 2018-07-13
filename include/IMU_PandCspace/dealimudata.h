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
    }
    else
    {
        IMU_fGyroX = (IMU_fGyroX * BMI055_DPS_MIN * M_PI) / (180.0 * BMI055_DATA_MIN);
    }

    if (IMU_fGyroY > 0)
    {
        IMU_fGyroY = (IMU_fGyroY * BMI055_DPS_MAX * M_PI) / (180.0 * BMI055_DATA_MAX);
    }
    else
    {
        IMU_fGyroY = (IMU_fGyroY * BMI055_DPS_MIN * M_PI) / (180.0 * BMI055_DATA_MIN);
    }

    if (IMU_fGyroZ > 0)
    {
        IMU_fGyroZ = (IMU_fGyroZ * BMI055_DPS_MAX * M_PI) / (180.0 * BMI055_DATA_MAX);
    }
    else
    {
        IMU_fGyroZ = (IMU_fGyroZ * BMI055_DPS_MIN * M_PI) / (180.0 * BMI055_DATA_MIN);
    }

    if (IMU_fAccelX > 0)
    {
        IMU_fAccelX = (IMU_fAccelX * BMI055_G_MAX * G_MPSS) / BMI055_DATA_MAX;
    }
    else
    {
        IMU_fAccelX = (IMU_fAccelX * BMI055_G_MIN * G_MPSS) / BMI055_DATA_MIN;
    }

    if (IMU_fAccelY > 0)
    {
        IMU_fAccelY = (IMU_fAccelY * BMI055_G_MAX * G_MPSS) / BMI055_DATA_MAX;
    }
    else
    {
        IMU_fAccelY = (IMU_fAccelY * BMI055_G_MIN * G_MPSS) / BMI055_DATA_MIN;
    }

    if (IMU_fAccelZ > 0)
    {
        IMU_fAccelZ = (IMU_fAccelZ * BMI055_G_MAX * G_MPSS) / BMI055_DATA_MAX;
    }
    else
    {
        IMU_fAccelZ = (IMU_fAccelZ * BMI055_G_MIN * G_MPSS) / BMI055_DATA_MIN;
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
        std::cout << "IMU_Producer is waiting for an empty slot...\n";
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
ProducerIMUTask() // 生产者任务
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
    io_service io_s;
    serial_port sp(io_s, imudev.data());
    if (!sp.is_open())
    {
        std::cout << "can not open imu device" << std::endl;
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
    while (true)
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
                    std::cout << "get imu data" << std::endl;
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
                    //imumsg.header.stamp =;
                    std::cout<<"imu message sequence : "<<sequence<<std::endl;
                    timeval tv;
                    gettimeofday(&tv, 0);
                    imumsg.header.stamp = tv;
                    imumsg.header.seq = sequence;
                    sequence++;
                    imumsg.header.frame_id = "imu";
                    ProduceIMUItem(&gIMUItemRepository, imumsg);
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
        std::cout << "IMU_Consumer is waiting for items...\n";
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
    static int cnt = 0;
    while (1)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
//        usleep(2 * 1000);
        IMUMessage item = ConsumeIMUItem(&gIMUItemRepository); // 消费一个产品.
//        nodeptr->HandleImuMessage(0, "imu", std::ref(item));
        nodeptr->HandleImuMessage(0, "imu", item);

/*
        std::cout << "Consume the " << item.header.seq << "^th item" << std::endl;
        std::cout << "Angle : [" << item.angular_velocity.x << "," << item.angular_velocity.y << ","
                  << item.angular_velocity.z << "]" << std::endl;
        std::cout << "linear : [" << item.linear_acceleration.x << "," << item.linear_acceleration.y << ","
                  << item.linear_acceleration.z << "]" << std::endl;
*/
//        if (++cnt == kItemsToProduce)
//            break; // 如果产品消费个数为 kItemsToProduce, 则退出.
    }
}

void
InitIMUItemRepository(IMUItemRepository *ir)
{
    ir->write_position = 0; // 初始化产品写入位置.
    ir->read_position = 0; // 初始化产品读取位置.
}

}
#endif //PRODUCEANDCONSUME_DEALIMUDATA_H
