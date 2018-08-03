/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "node.h"

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros_msgs/StatusCode.h"

/*
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/StatusResponse.h"
 */

/*
#include "nav_msgs/Odometry.h"
#include "ros/serialization.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_eigen/tf2_eigen.h"
#include "visualization_msgs/MarkerArray.h"
*/

namespace cartographer_ros
{

namespace
{

cartographer_ros_msgs::SensorTopics
DefaultSensorTopics()
{
    cartographer_ros_msgs::SensorTopics topics;
    topics.laser_scan_topic = kLaserScanTopic;
    topics.multi_echo_laser_scan_topic = kMultiEchoLaserScanTopic;
    topics.point_cloud2_topic = kPointCloud2Topic;
    topics.imu_topic = kImuTopic;
    topics.odometry_topic = kOdometryTopic;
    topics.nav_sat_fix_topic = kNavSatFixTopic;
    topics.landmark_topic = kLandmarkTopic;
    return topics;
}

} // namespace

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

Node::Node(
    const NodeOptions &node_options,
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder)
//    MyBuffer *const tf_buffer)
    : node_options_(node_options),
      map_builder_bridge_(node_options_, std::move(map_builder))
{
    carto::common::MutexLocker lock(&mutex_);
/*
 *  获取地图的过程：
 *  1、从map_builder_bridge_.GetSubmapList()获取submaplist
 *  2、遍历submaplist中的数据得到submapid，根据submapid查询map_builder_bridge_.HandleSubmapQuery得到submap
 *  3、拼接submap成为map
 *  以下ros中的topic就是第一步，ros中的service就是第二步，第三步在occupancy_grid_node_main.cc中
    submap_list_publisher_ =
        node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(
            kSubmapListTopic, kLatestOnlyPublisherQueueSize);
  //  this service will be called by occupancy_grid_node which publish message to rivz
  //  return the texture of submap defined by submapid
    service_servers_.push_back(node_handle_.advertiseService(
        kSubmapQueryServiceName, &Node::HandleSubmapQuery, this));

    wall_timers_.push_back(node_handle_.createWallTimer(
        ::ros::WallDuration(node_options_.submap_publish_period_sec),
        &Node::PublishSubmapList, this));
*/
    imu_produce_running_.store(false, std::memory_order_relaxed);// = false;
    imu_produce_threadHasStopped_.store(true, std::memory_order_relaxed);// = true;

    imu_consumer_running_.store(false, std::memory_order_relaxed);// = false;
    imu_consumer_threadHasStopped_.store(true, std::memory_order_relaxed);// = true;

    laser_produce_running_.store(false, std::memory_order_relaxed);// = false;
    laser_produce_threadHasStopped_.store(true, std::memory_order_relaxed);// = true;

    laser_consumer_running_.store(false, std::memory_order_relaxed);// = false;
    laser_consumer_threadHasStopped_.store(true, std::memory_order_relaxed);// = true;

    map_creater_running.store(false, std::memory_order_relaxed);// = false;
    map_creater_threadHasStopped_.store(true, std::memory_order_relaxed);// = true;
}

Node::~Node() { FinishAllTrajectories(); }

/* this function is the callback of service whose name is kSubmapQueryServiceName
 * response contain the texture of submap defined by submapid
 *
bool
Node::HandleSubmapQuery(
    ::cartographer_ros_msgs::SubmapQuery::Request &request,
    ::cartographer_ros_msgs::SubmapQuery::Response &response)
{
    carto::common::MutexLocker lock(&mutex_);
    map_builder_bridge_.HandleSubmapQuery(request, response);
    return true;
}


void
Node::PublishSubmapList(const ::ros::WallTimerEvent &unused_timer_event)
{
    carto::common::MutexLocker lock(&mutex_);
    submap_list_publisher_.publish(map_builder_bridge_.GetSubmapList());
}
*/

void
Node::AddExtrapolator(const int trajectory_id,
                      const TrajectoryOptions &options)
{
    constexpr double kExtrapolationEstimationTimeSec = 0.001; // 1 ms
    CHECK(extrapolators_.count(trajectory_id) == 0);
    const double gravity_time_constant =
        node_options_.map_builder_options.use_trajectory_builder_3d()
        ? options.trajectory_builder_options.trajectory_builder_3d_options()
            .imu_gravity_time_constant()
        : options.trajectory_builder_options.trajectory_builder_2d_options()
            .imu_gravity_time_constant();
    extrapolators_.emplace(
        std::piecewise_construct, std::forward_as_tuple(trajectory_id),
        std::forward_as_tuple(
            ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
            gravity_time_constant));
}

void
Node::AddSensorSamplers(const int trajectory_id,
                        const TrajectoryOptions &options)
{
    CHECK(sensor_samplers_.count(trajectory_id) == 0);
    sensor_samplers_.emplace(
        std::piecewise_construct, std::forward_as_tuple(trajectory_id),
        std::forward_as_tuple(
            options.rangefinder_sampling_ratio, options.odometry_sampling_ratio,
            options.fixed_frame_pose_sampling_ratio, options.imu_sampling_ratio,
            options.landmarks_sampling_ratio));
}

std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
Node::ComputeExpectedSensorIds(
    const TrajectoryOptions &options,
    const cartographer_ros_msgs::SensorTopics &topics) const
{
    using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
    using SensorType = SensorId::SensorType;
    std::set<SensorId> expected_topics;
    // Subscribe to all laser scan, multi echo laser scan, and point cloud topics.
    for (const std::string &topic : ComputeRepeatedTopicNames(
        topics.laser_scan_topic, options.num_laser_scans))
    {
        expected_topics.insert(SensorId{SensorType::RANGE, topic});
    }
    for (const std::string &topic :
        ComputeRepeatedTopicNames(topics.multi_echo_laser_scan_topic,
                                  options.num_multi_echo_laser_scans))
    {
        expected_topics.insert(SensorId{SensorType::RANGE, topic});
    }
    for (const std::string &topic : ComputeRepeatedTopicNames(
        topics.point_cloud2_topic, options.num_point_clouds))
    {
        expected_topics.insert(SensorId{SensorType::RANGE, topic});
    }
    // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
    // required.
    if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
        (node_options_.map_builder_options.use_trajectory_builder_2d() &&
            options.trajectory_builder_options.trajectory_builder_2d_options()
                .use_imu_data()))
    {
        expected_topics.insert(SensorId{SensorType::IMU, topics.imu_topic});
    }
    // Odometry is optional.
    if (options.use_odometry)
    {
        expected_topics.insert(
            SensorId{SensorType::ODOMETRY, topics.odometry_topic});
    }
    // NavSatFix is optional.
    if (options.use_nav_sat)
    {
        expected_topics.insert(
            SensorId{SensorType::FIXED_FRAME_POSE, topics.nav_sat_fix_topic});
    }
    // Landmark is optional.
    if (options.use_landmarks)
    {
        expected_topics.insert(SensorId{SensorType::LANDMARK, kLandmarkTopic});
    }
    return expected_topics;
}

int
Node::AddTrajectory(const TrajectoryOptions &options,
                    const cartographer_ros_msgs::SensorTopics &topics)
{
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
        expected_sensor_ids = ComputeExpectedSensorIds(options, topics);
    const int trajectory_id =
        map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
    AddExtrapolator(trajectory_id, options);
    AddSensorSamplers(trajectory_id, options);

    is_active_trajectory_[trajectory_id] = true;
    for (const auto &sensor_id : expected_sensor_ids)
    {
        subscribed_topics_.insert(sensor_id.id);
    }
    return trajectory_id;
}

bool
Node::ValidateTrajectoryOptions(const TrajectoryOptions &options)
{
    if (node_options_.map_builder_options.use_trajectory_builder_2d())
    {
        return options.trajectory_builder_options
            .has_trajectory_builder_2d_options();
    }
    if (node_options_.map_builder_options.use_trajectory_builder_3d())
    {
        return options.trajectory_builder_options
            .has_trajectory_builder_3d_options();
    }
    return false;
}

bool
Node::ValidateTopicNames(
    const ::cartographer_ros_msgs::SensorTopics &topics,
    const TrajectoryOptions &options)
{
    for (const auto &sensor_id : ComputeExpectedSensorIds(options, topics))
    {
        const std::string &topic = sensor_id.id;
        if (subscribed_topics_.count(topic) > 0)
        {
            LOG(ERROR) << "Topic name [" << topic << "] is already used.";
            return false;
        }
    }
    return true;
}

cartographer_ros_msgs::StatusResponse
Node::FinishTrajectoryUnderLock(
    const int trajectory_id)
{
    cartographer_ros_msgs::StatusResponse status_response;

    // First, check if we can actually finish the trajectory.
    if (map_builder_bridge_.GetFrozenTrajectoryIds().count(trajectory_id))
    {
        const std::string error =
            "Trajectory " + std::to_string(trajectory_id) + " is frozen.";
        LOG(ERROR) << error;
        status_response.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
        status_response.message = error;
        return status_response;
    }
    if (is_active_trajectory_.count(trajectory_id) == 0)
    {
        const std::string error =
            "Trajectory " + std::to_string(trajectory_id) + " is not created yet.";
        LOG(ERROR) << error;
        status_response.code = cartographer_ros_msgs::StatusCode::NOT_FOUND;
        status_response.message = error;
        return status_response;
    }
    if (!is_active_trajectory_[trajectory_id])
    {
        const std::string error = "Trajectory " + std::to_string(trajectory_id) +
            " has already been finished.";
        LOG(ERROR) << error;
        status_response.code =
            cartographer_ros_msgs::StatusCode::RESOURCE_EXHAUSTED;
        status_response.message = error;
        return status_response;
    }

    // Shutdown the subscribers of this trajectory.
    CHECK(is_active_trajectory_.at(trajectory_id));
    map_builder_bridge_.FinishTrajectory(trajectory_id);
    is_active_trajectory_[trajectory_id] = false;
    const std::string message =
        "Finished trajectory " + std::to_string(trajectory_id) + ".";
    status_response.code = cartographer_ros_msgs::StatusCode::OK;
    status_response.message = message;
    return status_response;
}

void
Node::StartTrajectoryWithDefaultTopics(const TrajectoryOptions &options)
{
    carto::common::MutexLocker lock(&mutex_);
    CHECK(ValidateTrajectoryOptions(options));
    AddTrajectory(options, DefaultSensorTopics());
}

std::vector<
    std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
Node::ComputeDefaultSensorIdsForMultipleBags(
    const std::vector<TrajectoryOptions> &bags_options) const
{
    using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
    std::vector<std::set<SensorId>> bags_sensor_ids;
    for (size_t i = 0; i < bags_options.size(); ++i)
    {
        std::string prefix;
        if (bags_options.size() > 1)
        {
            prefix = "bag_" + std::to_string(i + 1) + "_";
        }
        std::set<SensorId> unique_sensor_ids;
        for (const auto &sensor_id :
            ComputeExpectedSensorIds(bags_options.at(i), DefaultSensorTopics()))
        {
            unique_sensor_ids.insert(SensorId{sensor_id.type, prefix + sensor_id.id});
        }
        bags_sensor_ids.push_back(unique_sensor_ids);
    }
    return bags_sensor_ids;
}

void
Node::GetMap()
{
    using SubmapId = ::cartographer::mapping::SubmapId;
    using SubmapSlice = ::cartographer::io::SubmapSlice;

    std::set<SubmapId> submap_ids_to_delete;

    MySubmapdata::MySubmapList msg = map_builder_bridge_.GetSubmapList();

    for (const auto &pair : submap_slices_)
    {
        submap_ids_to_delete.insert(pair.first);
    }

    for(const auto &submap_msg : msg.submap)
    {
        SubmapId id{submap_msg.my_trajectory_id, submap_msg.my_submap_index};
        submap_ids_to_delete.erase(id);
        SubmapSlice  &submap_slice = submap_slices_[id];
        submap_slice.pose = cartographer_ros::ToRigid3d(submap_msg.my_pose);
        submap_slice.metadata_version = submap_msg.my_submap_version;
        if (submap_slice.surface != nullptr &&
            submap_slice.version == submap_msg.my_submap_version)
        {
            continue;
        }

        MySubmapdata::MySubmapResponse my_fetched_textures;
        map_builder_bridge_.HandlerSubmapQuery(id, my_fetched_textures);
        if(!my_fetched_textures.status)
        {
            continue;
        }
        if(my_fetched_textures.textures.empty())
        {
            continue;
        }

        auto fetched_textures = ::cartographer::common::make_unique<::cartographer::io::SubmapTextures>();
        fetched_textures->version = my_fetched_textures.my_response_version;
        for(const auto &texture : my_fetched_textures.textures)
        {
            const std::string compressed_cells(texture.cells.begin(), texture.cells.end());
            fetched_textures->textures.emplace_back(::cartographer::io::SubmapTexture{
                ::cartographer::io::UnpackTextureData(compressed_cells, texture.width, texture.height),
                texture.width, texture.height,
                texture.resolution,
                ToRigid3d(texture.slice_pose)});
        }

        CHECK(!fetched_textures->textures.empty());
        submap_slice.version = fetched_textures->version;
        const auto fetched_texture = fetched_textures->textures.begin();
        submap_slice.width = fetched_texture->width;
        submap_slice.height = fetched_texture->height;
        submap_slice.slice_pose = fetched_texture->slice_pose;
        submap_slice.resolution = fetched_texture->resolution;
        submap_slice.cairo_data.clear();
        submap_slice.surface = ::cartographer::io::DrawTexture(fetched_texture->pixels.intensity,
                                                               fetched_texture->pixels.alpha,
                                                               fetched_texture->width,
                                                               fetched_texture->height,
                                                               &submap_slice.cairo_data);
    }

    for (const auto &id : submap_ids_to_delete)
    {
        submap_slices_.erase(id);
    }

    last_frame_id_ = msg.header.frame_id;
    last_timestamp_ = msg.header.stamp;
};

int
Node::AddOfflineTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId> &
    expected_sensor_ids,
    const TrajectoryOptions &options)
{
    carto::common::MutexLocker lock(&mutex_);
    const int trajectory_id =
        map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
    AddExtrapolator(trajectory_id, options);
    AddSensorSamplers(trajectory_id, options);
    is_active_trajectory_[trajectory_id] = true;
    return trajectory_id;
}


void
Node::FinishAllTrajectories()
{
    carto::common::MutexLocker lock(&mutex_);
    for (auto &entry : is_active_trajectory_)
    {
        const int trajectory_id = entry.first;
        if (entry.second)
        {
            CHECK_EQ(FinishTrajectoryUnderLock(trajectory_id).code,
                     cartographer_ros_msgs::StatusCode::OK);
        }
    }
}

bool
Node::FinishTrajectory(const int trajectory_id)
{
    carto::common::MutexLocker lock(&mutex_);
    return FinishTrajectoryUnderLock(trajectory_id).code ==
        cartographer_ros_msgs::StatusCode::OK;
}

void
Node::RunFinalOptimization()
{
    {
        carto::common::MutexLocker lock(&mutex_);
        for (const auto &entry : is_active_trajectory_)
        {
            CHECK(!entry.second);
        }
    }
    // Assuming we are not adding new data anymore, the final optimization
    // can be performed without holding the mutex.
    map_builder_bridge_.RunFinalOptimization();
}

void
Node::HandleImuMessage(const int trajectory_id,
                       const std::string &sensor_id,
                       //const sensor_msgs::Imu::ConstPtr &msg
                       const IMU_PandCspace::IMUMessage& msg)
{
    carto::common::MutexLocker lock(&mutex_);
    if (!sensor_samplers_.at(trajectory_id).imu_sampler.Pulse())
    {
        return;
    }
    auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
    auto imu_data_ptr = sensor_bridge_ptr->ToImuData(msg);
    if (imu_data_ptr != nullptr)
    {
        extrapolators_.at(trajectory_id).AddImuData(*imu_data_ptr);
    }
    sensor_bridge_ptr->HandleImuMessage(sensor_id, msg);
}

void
Node::HandleMultiEchoLaserScanMessage(
    const int trajectory_id, const std::string &sensor_id,
    //const sensor_msgs::MultiEchoLaserScan::ConstPtr &msg
    const LASER_PandCspace::LASERMessage& msg)
{
    carto::common::MutexLocker lock(&mutex_);
    if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse())
    {
        return;
    }
    map_builder_bridge_.sensor_bridge(trajectory_id)
        ->HandleMultiEchoLaserScanMessage(sensor_id, msg);
}


void
Node::SerializeState(const std::string &filename)
{
    carto::common::MutexLocker lock(&mutex_);
    CHECK(map_builder_bridge_.SerializeState(filename))
    << "Could not write state.";
}


} // namespace cartographer_ros
