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

::cartographer::common::Mutex mutex_;

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

void
mapcreator(cartographer_ros::Node *nodeptr)
{
    const double resolution_ = 0.05;
    int tmptag = 0;
    while (true)
    {
        milliseconds_sleep(300);
        nodeptr->GetMap();
        if (nodeptr->submap_slices_.empty() || nodeptr->last_frame_id_.empty())
        {
            continue;
        }
        ::cartographer::common::MutexLocker locker(&mutex_);
        auto painted_slices = PaintSubmapSlices(nodeptr->submap_slices_, resolution_);
        std::string savemapstr;
        std::stringstream tmpstream;
        tmpstream << "//home//jacky//Downloads//test//" << tmptag << ".png";
        savemapstr = tmpstream.str();
        tmptag++;
        cairo_surface_write_to_png(painted_slices.surface.get(), savemapstr.data());
    }
    return;
}

void
Run()
{
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


    node.StartTrajectoryWithDefaultTopics(trajectory_options);

    IMU_PandCspace::InitIMUItemRepository(&IMU_PandCspace::gIMUItemRepository);
    LASER_PandCspace::InitLASERItemRepository(&LASER_PandCspace::gLASERItemRepository);
    std::thread imu_producer(IMU_PandCspace::ProducerIMUTask); // 创建imu生产者线程.
    std::thread imu_consumer(IMU_PandCspace::ConsumerIMUTask, &node); // 创建imu消费之线程.
    std::thread laser_producer(LASER_PandCspace::ProducerLASERTask);// 创建laser生产者线程.
    std::thread laser_consumer(LASER_PandCspace::ConsumerLASERTask, &node);// 创建laser生产者线程.
    std::thread map_creator(mapcreator, &node);// 创建地图生成线程.
    imu_producer.join();
    imu_consumer.join();
    laser_producer.join();
    laser_consumer.join();
    map_creator.join();

//    ::ros::spin();

    node.FinishAllTrajectories();
    node.RunFinalOptimization();

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
    CHECK(!cartographer_ros::FLAGS_configuration_directory.empty())
    << "-configuration_directory is missing.";
    CHECK(!cartographer_ros::FLAGS_configuration_basename.empty())
    << "-configuration_basename is missing.";

    std::cout << "Hello, World!" << std::endl;

    cartographer_ros::ScopedRosLogSink ros_log_sink;
    cartographer_ros::Run();
    return 0;
}