#include <unistd.h>
#include <cstdlib>
#include <condition_variable>
#include <iostream>
#include <thread>

#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"

#include "IMU_PandCspace/dealimudata.h"
#include "LASER_PandCspace/deallaserdata.h"

#include "third/TCPClient.h"
#include "third/pub.h"


#define MAP_POSITION_OUT_FILE

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros
{
namespace
{

std::string FLAGS_configuration_directory;
std::string FLAGS_configuration_basename;

TCPClient tcp_client;
std::string server_ip;// = "192.168.1.113";
int server_port;// = 11999;

::cartographer::common::Mutex mutex_;
::cartographer_ros::Node* global_nodeptr;

void
milliseconds_sleep(unsigned long mSec)
{
    struct timeval tv;
    tv.tv_sec = mSec / 1000;
    tv.tv_usec = (mSec % 1000) * 1000;
    int err;
    do
    {
        err = select(0, NULL, NULL, NULL, &tv);
    }
    while (err < 0 && errno == EINTR);
}
#ifdef MAP_POSITION_OUT_FILE
std::ofstream mappositionout("/home/pi/mapposition.data");
#endif

#ifdef MAP_POSITION_OUT_FILE
void PointOutput(MyMarkerArray::Point points_out)
{
    mappositionout<<"      -"<<std::endl;
    mappositionout<<"       x: "<<points_out.x<<std::endl;
    mappositionout<<"       y: "<<points_out.y<<std::endl;
    mappositionout<<"       z: "<<points_out.z<<std::endl;
}

void MarkerOutput(MyMarkerArray::Marker marker)
{
    mappositionout<<"   header:"<<std::endl<<"      stamp:"<<std::endl<<"           secs: "<<marker.header.stamp.tv_sec<<std::endl
                  <<"           nsecs: "<<marker.header.stamp.tv_usec<<std::endl<<"     frame_id: "<<marker.header.frame_id<<std::endl;
    mappositionout<<"   ns: "<<marker.ns<<std::endl;
    mappositionout<<"   id :"<<marker.id<<std::endl<<"   points:"<<std::endl;
    for_each(marker.points.begin(), marker.points.end(), PointOutput);
}
#endif

void
mapcreator(cartographer_ros::Node *nodeptr)
{
    if(global_nodeptr->map_creater_running_.load(std::memory_order_acquire))
    {
        return;
    }
    global_nodeptr->map_creater_running_.store(true, std::memory_order_release);
    global_nodeptr->map_creater_threadHasStopped_.store(false, std::memory_order_release);
    const double resolution_ = 0.05;
    int tmptag = 0;
//    while (true)


    while(global_nodeptr->map_creater_running_.load(std::memory_order_relaxed) == true)
    {
        milliseconds_sleep(1000);

        nodeptr->GetMap();
        MyMarkerArray::MarkerArray markerarray = nodeptr->GetNodeList();

        if (nodeptr->submap_slices_.empty() || nodeptr->last_frame_id_.empty())
        {
            continue;
        }
        ::cartographer::common::MutexLocker locker(&mutex_);
        auto painted_slices = PaintSubmapSlices(nodeptr->submap_slices_, resolution_);
        std::string savemapstr;
        std::stringstream tmpstream;
        tmpstream << "//home//pi//tmpmap//" << tmptag << ".png";
        savemapstr = tmpstream.str();
        tmptag++;

        cairo_surface_write_to_png(painted_slices.surface.get(), savemapstr.data());

        std::cout<<"top:"<<painted_slices.origin[0]<<" left:"<<painted_slices.origin[1]<<std::endl;

#ifdef MAP_POSITION_OUT_FILE
        mappositionout<<savemapstr<<std::endl
                      <<"position: "<<painted_slices.origin[0]<<"  "<<painted_slices.origin[1]<<std::endl;
        mappositionout<<"markers:"<<std::endl;
        for_each(markerarray.markers.begin(), markerarray.markers.end(), MarkerOutput);
#endif

        int iport = 11999;
 //       std::string serverip= "192.168.1.131";
        std::cout<<"send file : "<<savemapstr<<std::endl;

        if(1 == send_work(server_ip.data(), server_port, savemapstr.data()))
        {
            std::cout<<"send success!"<<std::endl;
        }


        /*
        tcp_client.Send("newmap");
        string tmprec = tcp_client.receive();
        if(!tmprec.compare("ok1"))
        {
            //begin to send png
            fstream sendfile;
            sendfile.open(savemapstr.data(), ios::binary|ios::in);
            sendfile.seekg(0, sendfile.end);
            int srcsize = sendfile.tellg();
            char* tmpsendbuf = new char[1024];
            if(!srcsize)
            {
                memset(tmpsendbuf, 0x00, 1024);
                memcpy(tmpsendbuf, &srcsize, 4);
                tcp_client.Send(tmpsendbuf, 4);
                tmprec = tcp_client.receive();
                if(!tmprec.compare("ok2"))
                {
                    while(!sendfile.eof())
                    {
                        memset(tmpsendbuf, 0x00, 1024);
                        sendfile.read(tmpsendbuf, 1024);
                        tcp_client.Send(tmpsendbuf, sendfile.gcount());
                        tmprec = tcp_client.receive();
                        if(tmprec.compare("ok3"))
                        {
                            break;
                        }
                    }
                }
            }
            sendfile.close();
        }
        */
        //remove(savemapstr.data());

    }
    global_nodeptr->map_creater_threadHasStopped_.store(true, std::memory_order_release);
    std::cout<<"map creator thread end"<<std::endl;
    return;
}

void
Stop()
{
    std::cout << "thread beding to stop " << std::endl;

    //end imu produce;
    global_nodeptr->imu_produce_running_.store(false, std::memory_order_release);//= false;
    std::cout << "imu produce wait thread exit " << std::endl;
    while (global_nodeptr->imu_produce_threadHasStopped_.load(std::memory_order_relaxed) == false)
    {
    };
    std::cout << "imu produce thread has exit " << std::endl;
    if (nullptr == global_nodeptr->imu_produce_Thread_)
    {
        // 必须调用 join, 等待线程真正执行完毕再析构,
        // 否则析构时thread类调用 terminate 异常
        global_nodeptr->imu_produce_Thread_->join();
        delete global_nodeptr->imu_produce_Thread_;
        global_nodeptr->imu_produce_Thread_ = nullptr;
    }

    //end imu consumer
    global_nodeptr->imu_consumer_running_.store(false, std::memory_order_release);//= false;
    std::cout << "imu produce wait thread exit " << std::endl;
    while (global_nodeptr->imu_consumer_threadHasStopped_.load(std::memory_order_relaxed) == false)
    {
    };
    std::cout << "imu produce thread has exit " << std::endl;
    if (nullptr == global_nodeptr->imu_consumer_Thread_)
    {
        // 必须调用 join, 等待线程真正执行完毕再析构,
        // 否则析构时thread类调用 terminate 异常
        global_nodeptr->imu_consumer_Thread_->join();
        delete global_nodeptr->imu_consumer_Thread_;
        global_nodeptr->imu_consumer_Thread_ = nullptr;
    }

    //end laser produce
    global_nodeptr->laser_produce_running_.store(false, std::memory_order_release);//= false;
    std::cout << "imu produce wait thread exit " << std::endl;
    while (global_nodeptr->laser_produce_threadHasStopped_.load(std::memory_order_relaxed) == false)
    {
    };
    std::cout << "imu produce thread has exit " << std::endl;
    if (nullptr == global_nodeptr->laser_produce_Thread_)
    {
        // 必须调用 join, 等待线程真正执行完毕再析构,
        // 否则析构时thread类调用 terminate 异常
        global_nodeptr->laser_produce_Thread_->join();
        delete global_nodeptr->laser_produce_Thread_;
        global_nodeptr->laser_produce_Thread_ = nullptr;
    }

    //end laser consume
    global_nodeptr->laser_consumer_running_.store(false, std::memory_order_release);//= false;
    std::cout << "imu produce wait thread exit " << std::endl;
    while (global_nodeptr->laser_consumer_threadHasStopped_.load(std::memory_order_relaxed) == false)
    {
    };
    std::cout << "imu produce thread has exit " << std::endl;
    if (nullptr == global_nodeptr->laser_consumer_Thread_)
    {
        // 必须调用 join, 等待线程真正执行完毕再析构,
        // 否则析构时thread类调用 terminate 异常
        global_nodeptr->laser_consumer_Thread_->join();
        delete global_nodeptr->laser_consumer_Thread_;
        global_nodeptr->laser_consumer_Thread_ = nullptr;
    }

    return;
}

static bool mainRunning = true;
void
quitFunc(int signal)
{
    cout << "quit main" << endl;
    mainRunning = false;
    Stop();
}

void
Run()
{
/*    if(!tcp_client.setup(server_ip, server_port))
    {
        std::cout<<"display server cannot connected, exit!"<<std::endl;
    }
*/


    constexpr double kTfBufferCacheTimeInSeconds = 10.;
    NodeOptions node_options;
    TrajectoryOptions trajectory_options;
    std::tie(node_options, trajectory_options) =
        cartographer_ros::LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

    std::cout << node_options.map_frame << std::endl;
    std::cout << trajectory_options.published_frame << std::endl;

    auto map_builder =
        cartographer::common::make_unique<cartographer::mapping::MapBuilder>(node_options.map_builder_options);

    Node node(node_options, std::move(map_builder));//, &tf_buffer);

    global_nodeptr = &node;
    node.StartTrajectoryWithDefaultTopics(trajectory_options);

    IMU_PandCspace::InitIMUItemRepository(&IMU_PandCspace::gIMUItemRepository);
    LASER_PandCspace::InitLASERItemRepository(&LASER_PandCspace::gLASERItemRepository);
    std::thread imu_producer(IMU_PandCspace::ProducerIMUTask, &node); // 创建imu生产者线程.
    std::thread imu_consumer(IMU_PandCspace::ConsumerIMUTask, &node); // 创建imu消费之线程.
    std::thread laser_producer(LASER_PandCspace::ProducerLASERTask, &node);// 创建laser生产者线程.
    std::thread laser_consumer(LASER_PandCspace::ConsumerLASERTask, &node);// 创建laser生产者线程.
    std::thread map_creator(mapcreator, &node);// 创建地图生成线程.
    global_nodeptr->imu_produce_Thread_ = &imu_producer;
    global_nodeptr->imu_consumer_Thread_ = &imu_consumer;
    global_nodeptr->laser_produce_Thread_ = &laser_producer;
    global_nodeptr->laser_consumer_Thread_ = &laser_consumer;
    global_nodeptr->map_creater_Thread_ = &map_creator;

//    imu_producer.join();
//    imu_consumer.join();
//    laser_producer.join();
//    laser_consumer.join();
//    map_creator.join();
    std::chrono::seconds dura(2);
    std::signal(SIGINT, quitFunc);
    std::signal(SIGTERM, quitFunc);
    std::signal(SIGABRT, quitFunc);
    while (mainRunning)
    {
        std::this_thread::sleep_for(dura);
        std::cout << "main sleep: " << std::endl;
    }

    node.FinishAllTrajectories();
    node.RunFinalOptimization();

    //是否需要等待RunFinalOptimization的结束？
    std::this_thread::sleep_for(dura);
    global_nodeptr->map_creater_running_.store(false, std::memory_order_release);//= false;
    std::cout << "map creater wait thread exit " << std::endl;
    while (global_nodeptr->map_creater_threadHasStopped_.load(std::memory_order_relaxed) == false)
    {
    };
    std::cout << "map creater thread has exit " << std::endl;
    if (nullptr == global_nodeptr->map_creater_Thread_)
    {
        // 必须调用 join, 等待线程真正执行完毕再析构,
        // 否则析构时thread类调用 terminate 异常
        global_nodeptr->map_creater_Thread_->join();
        delete global_nodeptr->map_creater_Thread_;
        global_nodeptr->map_creater_Thread_ = nullptr;
    }

    if (!FLAGS_save_state_filename.empty())
    {
        node.SerializeState(FLAGS_save_state_filename);
    }
}

}

}

int
main(int argc, char **argv)
{

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    cartographer_ros::FLAGS_configuration_directory = argv[1];
    cartographer_ros::FLAGS_configuration_basename = argv[2];
    cartographer_ros::server_ip = argv[3];
    cartographer_ros::server_port = atoi(argv[4]);

    CHECK(!cartographer_ros::FLAGS_configuration_directory.empty())
    << "-configuration_directory is missing.";
    CHECK(!cartographer_ros::FLAGS_configuration_basename.empty())
    << "-configuration_basename is missing.";

    std::cout << "Hello, World!" << std::endl;

    cartographer_ros::ScopedRosLogSink ros_log_sink;
    cartographer_ros::Run();

    return 0;
}