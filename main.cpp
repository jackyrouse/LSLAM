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
Run()
{
    constexpr double kTfBufferCacheTimeInSeconds = 10.;
    NodeOptions node_options;
    TrajectoryOptions trajectory_options;
    std::tie(node_options, trajectory_options) =
        cartographer_ros::LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

    std::cout<<node_options.map_frame<<std::endl;
    std::cout<<trajectory_options.published_frame<<std::endl;
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