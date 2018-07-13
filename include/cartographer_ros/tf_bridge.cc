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

#include "cartographer/common/make_unique.h"


#include "cartographer_ros/tf_bridge.h"


namespace cartographer_ros
{

TfBridge::TfBridge(const std::string &tracking_frame,
                   const double lookup_transform_timeout_sec)
    : tracking_frame_(tracking_frame),
      lookup_transform_timeout_sec_(lookup_transform_timeout_sec)
{}

std::unique_ptr<::cartographer::transform::Rigid3d> TfBridge::LookupToTracking(
    const ::cartographer::common::Time time,
    const std::string &frame_id) const
{
/*    ::ros::Duration timeout(lookup_transform_timeout_sec_);
    std::unique_ptr<::cartographer::transform::Rigid3d> frame_id_to_tracking;
    try
    {
        const ::ros::Time latest_tf_time =
            buffer_->lookupTransform(tracking_frame_, frame_id, ::ros::Time(0.), timeout).header.stamp;
        const ::ros::Time requested_time = ToRos(time);
        if (latest_tf_time >= requested_time)
        {
            // We already have newer data, so we do not wait. Otherwise, we would wait
            // for the full 'timeout' even if we ask for data that is too old.
            timeout = ::ros::Duration(0.);
        }
        return ::cartographer::common::make_unique<::cartographer::transform::Rigid3d>(ToRigid3d(buffer_->lookupTransform(
            tracking_frame_, frame_id, requested_time, timeout)));
    }
    catch (const tf2::TransformException &ex)
    {
        LOG(WARNING) << ex.what();
    }
    return nullptr;
    */
    //这函数的作用是查找坐标变换，主要是两个变换：laser和imu，其中laser以安装位置为准，imu不会变化
    //主要参考urdf文件
    //根据frame_id为标识去判断返回哪个rigid3d
    Eigen::Vector3d imu_translation(0., 0., 0.);
    Eigen::Vector3d laser_translation(0., 0., 0.);
    Eigen::Quaterniond imu_quaternion(1., 0., 0., 0.);
    Eigen::Quaterniond laser_quaternion(1., 0., 0., 0.);
    if(0 == frame_id.compare("imu"))
    {
        return ::cartographer::common::make_unique<::cartographer::transform::Rigid3d>(ToRigid3d(imu_translation, imu_quaternion));
    }
    if(0 == frame_id.compare("horizonal_2d_laser"))
    {
        return ::cartographer::common::make_unique<::cartographer::transform::Rigid3d>(ToRigid3d(laser_translation, laser_quaternion));
    }
    return nullptr;
}

} // namespace cartographer_ros
