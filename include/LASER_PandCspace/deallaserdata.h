//
// Created by jacky on 18-6-20.
//

#ifndef PRODUCEANDCONSUME_DEALLASERDATA_H
#define PRODUCEANDCONSUME_DEALLASERDATA_H
#include "ydlidar/CYdLidar.h"
using namespace ydlidar;
#include "laserdata.h"
#include "cartographer_ros/node.h"

namespace LASER_PandCspace
{
struct LASERItemRepository
{
    LASERMessage item_buffer[kLASERItemRepositorySize]; // 产品缓冲区, 配合 read_position 和 write_position 模型环形队列.
    size_t read_position; // 消费者读取产品位置.
    size_t write_position; // 生产者写入产品位置.

    std::mutex mtx; // 互斥量,保护产品缓冲区
    std::condition_variable repo_not_full; // 条件变量, 指示产品缓冲区不为满.
    std::condition_variable repo_not_empty; // 条件变量, 指示产品缓冲区不为空.
} gLASERItemRepository; // 产品库全局变量, 生产者和消费者操作该变量.
typedef struct LASERItemRepository LASERItemRepository;

CYdLidar laser;
static bool running = false;

const std::string port = "/dev/ydlidar";
const int baud = 115200;
const int intensities = 0;

void
ProduceLASERItem(LASERItemRepository *ir, LASERMessage item)
{
    std::unique_lock<std::mutex> lock(ir->mtx);
    while (((ir->write_position + 1) % kLASERItemRepositorySize)
        == ir->read_position)
    { // item buffer is full, just wait here.
        std::cout << "LASER_Producer is waiting for an empty slot...\n";
        (ir->repo_not_full).wait(lock); // 生产者等待"产品库缓冲区不为满"这一条件发生.
    }

    (ir->item_buffer)[ir->write_position] = item; // 写入产品.
    (ir->write_position)++; // 写入位置后移.

    if (ir->write_position == kLASERItemRepositorySize) // 写入位置若是在队列最后则重新设置为初始位置.
        ir->write_position = 0;

    (ir->repo_not_empty).notify_all(); // 通知消费者产品库不为空.
    lock.unlock(); // 解锁.
}

LASERMessage
ConsumeLASERItem(LASERItemRepository * ir)
{
    LASERMessage data;
    std::unique_lock<std::mutex> lock(ir->mtx);
    // item buffer is empty, just wait here.
    while (ir->write_position == ir->read_position)
    {
        std::cout << "LASER_Consumer is waiting for items...\n";
        (ir->repo_not_empty).wait(lock); // 消费者等待"产品库缓冲区不为空"这一条件发生.
    }

    data = (ir->item_buffer)[ir->read_position]; // 读取某一产品
    (ir->read_position)++; // 读取位置后移

    if (ir->read_position >= kLASERItemRepositorySize) // 读取位置若移到最后，则重新置位.
        ir->read_position = 0;

    (ir->repo_not_full).notify_all(); // 通知消费者产品库不为满.
    lock.unlock(); // 解锁.

    return data; // 返回产品.
}

void
createlasermsgfun(LASERMessage*lmsg, LaserScan tmpscan)
{
    lmsg->angle_min = tmpscan.config.min_angle;
    lmsg->angle_max = tmpscan.config.max_angle;
    lmsg->range_min = tmpscan.config.min_range;
    lmsg->range_max = tmpscan.config.max_range;
    lmsg->scan_time = tmpscan.config.scan_time/1000000000;
    lmsg->angle_increment = tmpscan.config.ang_increment;
    lmsg->time_increment = tmpscan.config.time_increment/1000000000;

    std::cout<<"ranges:"<<std::endl;
    int i = 0;
    for (auto r_val:tmpscan.ranges)
    {
        lmsg->ranges[i] = r_val;
//        std::cout<<"  echoes: ["<<lmsg->ranges[i]<<"]"<<std::endl;
        i++;
    }
    i = 0;
    std::cout<<"intensities:"<<std::endl;
    for (auto i_val:tmpscan.intensities)
    {
        lmsg->intensities[i] = i_val;
//        std::cout<<"  echoes: ["<<lmsg->intensities[i]<<"]"<<std::endl;
        i++;
    }
    return;
}

void
ProducerLASERTask() // 生产者任务
{
/*
    for (int i = 1; i <= kLASERItemRepositorySize; ++i)
    {
        // sleep(1);
        std::cout << "Produce the " << i << "^th item..." << std::endl;
        ProduceLASERItem(&gLASERItemRepository, i); // 循环生产 kItemsToProduce 个产品.
    }
    */
    laser.setSerialPort(port);
    laser.setSerialBaudrate(baud);
    laser.setIntensities(intensities);

    laser.setMaxRange(16.0);
    laser.setMinRange(0.26);
    laser.setMaxAngle(180);
    laser.setMinAngle(-180);
    laser.setHeartBeat(false);
    laser.setReversion(false);
    laser.setFixedResolution(true);
    laser.setAutoReconnect(true);

    laser.initialize();

    int sequence = 0;

    while (!running)
    {
        bool hardError;
        LaserScan scan;

        if (laser.doProcessSimple(scan, hardError))
        {
            std::cout << "Scan received: " << (unsigned int) scan.ranges.size() << " ranges" << std::endl;
            std::cout << "Scan received: " << (unsigned int) scan.intensities.size() << " intensities" << std::endl;
            LASERMessage lasermsg;
            createlasermsgfun(&lasermsg, scan);
            std::cout<<"laser message sequence : "<<sequence<<std::endl;
            timeval tv;
            gettimeofday(&tv, 0);
            lasermsg.header.stamp = tv;
            lasermsg.header.seq = sequence;
            sequence++;
            lasermsg.header.frame_id = "horizonal_2d_laser";
            ProduceLASERItem(&gLASERItemRepository, lasermsg);

            /*
            std::cout<<"header:"<<std::endl<<"  seq: "<<lasermsg.header.seq<<std::endl<<"  stamp:"<<std::endl;
            std::cout<<"    secs: "<<lasermsg.header.stamp.tv_sec<<std::endl<<"    nsecs: "<<lasermsg.header.stamp.tv_usec*1000<<std::endl;
            std::cout<<"  frame_id: "<<lasermsg.header.frame_id<<std::endl;
            std::cout<<"angle_min: "<<lasermsg.angle_min<<std::endl;
            std::cout<<"angle_max: "<<lasermsg.angle_max<<std::endl;
            std::cout<<"angle_incremnet: "<<lasermsg.angle_increment<<std::endl;
            std::cout<<"time_increment: "<<lasermsg.time_increment<<std::endl;
            std::cout<<"scan_time: "<<lasermsg.scan_time<<std::endl;
            std::cout<<"range_min: "<<lasermsg.range_min<<std::endl;
            std::cout<<"range_max: "<<lasermsg.range_max<<std::endl;
            */
        }
//        usleep(50 * 1000);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    laser.turnOff();
    laser.disconnecting();

    return;
}

void
ConsumerLASERTask(cartographer_ros::Node* nodeptr) // 消费者任务
{
    static int cnt = 0;
    while (1)
    {
//        usleep(10 * 1000);;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        LASERMessage Laseritem = ConsumeLASERItem(&gLASERItemRepository); // 消费一个产品.
        //nodeptr->HandleMultiEchoLaserScanMessage(0, "echoes", std::ref(Laseritem));
        nodeptr->HandleMultiEchoLaserScanMessage(0, "echoes", Laseritem);
//        std::cout << "Consume the " << Laseritem.header.seq << "^th item" << std::endl;
    }
}

void
InitLASERItemRepository(LASERItemRepository *ir)
{
    ir->write_position = 0; // 初始化产品写入位置.
    ir->read_position = 0; // 初始化产品读取位置.
}
}
#endif //PRODUCEANDCONSUME_DEALLASERDATA_H
