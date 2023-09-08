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

#include "cartographer_ros/sensor_bridge.h"

#include "absl/memory/memory.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

namespace {

const std::string& CheckNoLeadingSlash(const std::string& frame_id) {
  if (frame_id.size() > 0) {
    CHECK_NE(frame_id[0], '/') << "The frame_id " << frame_id
                               << " should not start with a /. See 1.7 in "
                                  "http://wiki.ros.org/tf2/Migration.";
  }
  return frame_id;
}

}  // namespace

// SensorBridge初始化.map_builder_bridge_中,做初始化
SensorBridge::SensorBridge(
    const int num_subdivisions_per_laser_scan,
    const std::string& tracking_frame,
    const double lookup_transform_timeout_sec, tf2_ros::Buffer* const tf_buffer,
    carto::mapping::TrajectoryBuilderInterface* const trajectory_builder)      //trajectory_builder = coll
    : num_subdivisions_per_laser_scan_(num_subdivisions_per_laser_scan),
      tf_bridge_(tracking_frame, lookup_transform_timeout_sec, tf_buffer),     //tracking_frame = "base_link",
      trajectory_builder_(trajectory_builder) {}

// 里程计数据转换
std::unique_ptr<carto::sensor::OdometryData> SensorBridge::ToOdometryData(
    const nav_msgs::Odometry::ConstPtr& msg) {
  static bool flag = false;
  if(flag == false)
  {
    flag = true;
    LOG(WARNING)
      <<"\n nav_msgs::Odometry To sensor::OdometryData:"
      <<"\n msg->child_frame_id = "<<msg->child_frame_id;
  }
  const carto::common::Time time = FromRos(msg->header.stamp);
  // sensor_to_tracking = Tbase_base
  // msg->child_frame_id = base_link
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
      time, CheckNoLeadingSlash(msg->child_frame_id));
  if (sensor_to_tracking == nullptr) {
    return nullptr;
  }
  /***
   * 坐标转换过程：
   * ToRigid3d(msg->pose.pose) -> Todom_base
   * sensor_to_tracking -> Ttrackframe_base => sensor_to_tracking->inverse() -> Tbase_trackframe
   * ToRigid3d(msg->pose.pose) * sensor_to_tracking->inverse() -> Todom_base * Tbase_trackframe = Todom_trackframe
   * track_frame = base_link => Todom_base
   * */ 
  return absl::make_unique<carto::sensor::OdometryData>(
      carto::sensor::OdometryData{
          time, ToRigid3d(msg->pose.pose) * sensor_to_tracking->inverse()});
}

// 处理里程计数据
void SensorBridge::HandleOdometryMessage(
    const std::string& sensor_id, const nav_msgs::Odometry::ConstPtr& msg) {

  //将里程计数据格式转换，转换成　Todom_trackframe = Todom_base
  std::unique_ptr<carto::sensor::OdometryData> odometry_data =
      ToOdometryData(msg);
  if (odometry_data != nullptr) {

    // trajectory_builder_ = cartographer::mapping::MapBuilder
    // CollatedTrajectoryBuilder类型
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::OdometryData{odometry_data->time, odometry_data->pose});
  }
}

void SensorBridge::HandleNavSatFixMessage(
    const std::string& sensor_id, const sensor_msgs::NavSatFix::ConstPtr& msg) {
  const carto::common::Time time = FromRos(msg->header.stamp);       //获取时间
  if (msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::FixedFramePoseData{time, absl::optional<Rigid3d>()});
    return;
  }

  if (!ecef_to_local_frame_.has_value()) {
    ecef_to_local_frame_ =
        ComputeLocalFrameFromLatLong(msg->latitude, msg->longitude);   //通过gps返回的经纬度,计算坐标
    LOG(INFO) << "Using NavSatFix. Setting ecef_to_local_frame with lat = "
              << msg->latitude << ", long = " << msg->longitude << ".";
  }

  trajectory_builder_->AddSensorData(
      sensor_id, carto::sensor::FixedFramePoseData{
                     time, absl::optional<Rigid3d>(Rigid3d::Translation(
                               ecef_to_local_frame_.value() *
                               LatLongAltToEcef(msg->latitude, msg->longitude,
                                                msg->altitude)))});
}

void SensorBridge::HandleLandmarkMessage(
    const std::string& sensor_id,
    const cartographer_ros_msgs::LandmarkList::ConstPtr& msg) {
  auto landmark_data = ToLandmarkData(*msg);

  auto tracking_from_landmark_sensor = tf_bridge_.LookupToTracking(      //获取路标传感器相对 base_link 的坐标变换
      landmark_data.time, CheckNoLeadingSlash(msg->header.frame_id));    //路标传感器坐标系
  if (tracking_from_landmark_sensor != nullptr) {
    for (auto& observation : landmark_data.landmark_observations) {      //将所有观测到的路标,都转换到 base_link 坐标系下
      observation.landmark_to_tracking_transform =
          *tracking_from_landmark_sensor *
          observation.landmark_to_tracking_transform;
    }
  }
  trajectory_builder_->AddSensorData(sensor_id, landmark_data);
}

// IMU 数据格式转换
std::unique_ptr<carto::sensor::ImuData> SensorBridge::ToImuData(
    const sensor_msgs::Imu::ConstPtr& msg) {
  CHECK_NE(msg->linear_acceleration_covariance[0], -1)
      << "Your IMU data claims to not contain linear acceleration measurements "
         "by setting linear_acceleration_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";
  CHECK_NE(msg->angular_velocity_covariance[0], -1)
      << "Your IMU data claims to not contain angular velocity measurements "
         "by setting angular_velocity_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";

  const carto::common::Time time = FromRos(msg->header.stamp);
  // msg->header.frame_id = base_link
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
      time, CheckNoLeadingSlash(msg->header.frame_id));         //获取 陀螺仪坐标系相对 base_link 的坐标变换
  if (sensor_to_tracking == nullptr) {
    return nullptr;
  }
  CHECK(sensor_to_tracking->translation().norm() < 1e-5)
      << "The IMU frame must be colocated with the tracking frame. "
         "Transforming linear acceleration into the tracking frame will "
         "otherwise be imprecise.";
  return absl::make_unique<carto::sensor::ImuData>(carto::sensor::ImuData{
      time, sensor_to_tracking->rotation() * ToEigen(msg->linear_acceleration),  //将陀螺仪加速度方向和角速度方向 旋转到与 base_link 相同方向
      sensor_to_tracking->rotation() * ToEigen(msg->angular_velocity)});
}

// 处理IMU数据
void SensorBridge::HandleImuMessage(const std::string& sensor_id,
                                    const sensor_msgs::Imu::ConstPtr& msg) {
  std::unique_ptr<carto::sensor::ImuData> imu_data = ToImuData(msg);
  if (imu_data != nullptr) {
    // trajectory_builder_ = cartographer::mapping::MapBuilder
    // CollatedTrajectoryBuilder类型
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::ImuData{imu_data->time, imu_data->linear_acceleration,
                               imu_data->angular_velocity});
  }
}

// 将雷达数据转换成带时间的点云数据,并处理.
// 将雷达话题数据,转换成 carto::sensor::PointCloudWithIntensities(传感器数据)
void SensorBridge::HandleLaserScanMessage(
    const std::string& sensor_id, const sensor_msgs::LaserScan::ConstPtr& msg) {
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;

  //将雷达数据,转换为laser坐标系下的点云数据,并处理时间戳(起点为基准,转换成终点为基准)
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  
  // 处理雷达的点云数据　msg->header.frame_id = /laser
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}

void SensorBridge::HandleMultiEchoLaserScanMessage(
    const std::string& sensor_id,
    const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}

void SensorBridge::HandlePointCloud2Message(
    const std::string& sensor_id,
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  HandleRangefinder(sensor_id, time, msg->header.frame_id, point_cloud.points);
}

const TfBridge& SensorBridge::tf_bridge() const { return tf_bridge_; }

// 处理雷达的点云数据，将雷达数据分段，然后处理
void SensorBridge::HandleLaserScan(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id,     //frame_id = /laser
    const carto::sensor::PointCloudWithIntensities& points) {
  if (points.points.empty()) {
    return;
  }
  CHECK_LE(points.points.back().time, 0.f);
  // TODO(gaschler): Use per-point time instead of subdivisions.

  static bool log_out = false;
  if(log_out == false)
  {
    LOG(WARNING)<<"\n 每帧雷达数据划分成　"<<num_subdivisions_per_laser_scan_<<" 段";
    log_out = true;
  }

  for (int i = 0; i != num_subdivisions_per_laser_scan_; ++i) {
    // 获取每一段点云的起始和终止索引
    const size_t start_index =
        points.points.size() * i / num_subdivisions_per_laser_scan_;
    const size_t end_index =
        points.points.size() * (i + 1) / num_subdivisions_per_laser_scan_;
    
    // 获取该段点云数据
    carto::sensor::TimedPointCloud subdivision(
        points.points.begin() + start_index, points.points.begin() + end_index);
    if (start_index == end_index) {
      continue;
    }
    const double time_to_subdivision_end = subdivision.back().time;
    // `subdivision_time` is the end of the measurement so sensor::Collator will
    // send all other sensor data first.

    // time 雷达最后一点的时间戳,也即变换处理后的点云时间戳
    //测量结束时间
    // 获取该段点云最后一个点的时间
    const carto::common::Time subdivision_time =
        time + carto::common::FromSeconds(time_to_subdivision_end);

    // 获取该传感器对应的数据
    auto it = sensor_to_previous_subdivision_time_.find(sensor_id);

    // 有数据,但是数据的时间大于该段时间的.则不处理该段时间,直接continue小,进行下一段点云处理
    if (it != sensor_to_previous_subdivision_time_.end() &&
        it->second >= subdivision_time) {
      LOG(WARNING) << "Ignored subdivision of a LaserScan message from sensor "
                   << sensor_id << " because previous subdivision time "
                   << it->second << " is not before current subdivision time "
                   << subdivision_time;
      continue;
    }
    // 记录该段点云时间戳
    sensor_to_previous_subdivision_time_[sensor_id] = subdivision_time;
    
    // 将该段点云中具体点的时间戳，转换到以该点为基准的时间戳
    for (auto& point : subdivision) {
      //所有点时间，前移 time_to_subdivision_end，为了匹配后面计算获取时间方式
      point.time -= time_to_subdivision_end;
    }
    CHECK_EQ(subdivision.back().time, 0.f);

    // 将每帧雷达数据　分成 10 段来处理,将laser坐标系点云转换到　base_link 坐标系下点云数据，然后处理
    //frame_id = /laser
    HandleRangefinder(sensor_id, subdivision_time, frame_id, subdivision);
  }
}

// 将laser坐标系下点云数据，转换到　base_link 
// 然后调用　trajectory_builder_->AddSensorData() 来向轨迹中添加点云数据
void SensorBridge::HandleRangefinder(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id, const carto::sensor::TimedPointCloud& ranges) {
      // ranges = subdivision
  if (!ranges.empty()) {
    CHECK_LE(ranges.back().time, 0.f);
  }
  //frame_id = /laser
  // 获取对应时刻的转换矩阵 Tbase_laser; time ，该段最后一个点的时间戳
  const auto sensor_to_tracking =
      tf_bridge_.LookupToTracking(time, CheckNoLeadingSlash(frame_id));

  if (sensor_to_tracking != nullptr) {
    /*
    struct TimedPointCloudData {
          common::Time time;      该段点云最后一个点的时间戳
          Eigen::Vector3f origin; 该段点云原点，最后一个点对应时刻下,雷达坐标系laser,在base下的坐标
          TimedPointCloud ranges; 该段点云转换到base下的点云
          // 'intensities' has to be same size as 'ranges', or empty.
          std::vector<float> intensities;  //点云强度
    };
    */ 
  //  collated_trajectory_builder  AddSensorData
    trajectory_builder_->AddSensorData(
        sensor_id, carto::sensor::TimedPointCloudData{
                       time, sensor_to_tracking->translation().cast<float>(),
                       carto::sensor::TransformTimedPointCloud(
                           ranges, sensor_to_tracking->cast<float>())});
  }
}

}  // namespace cartographer_ros
