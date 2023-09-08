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

#include "cartographer_ros/tf_bridge.h"

#include "absl/memory/memory.h"
#include "cartographer_ros/msg_conversion.h"

namespace cartographer_ros {

TfBridge::TfBridge(const std::string& tracking_frame,
                   const double lookup_transform_timeout_sec,
                   const tf2_ros::Buffer* buffer)
    : tracking_frame_(tracking_frame),
      lookup_transform_timeout_sec_(lookup_transform_timeout_sec),
      buffer_(buffer) {

        LOG(WARNING)
          <<"\n TfBridge Init:"
          <<"\n tracking_frame_ = "<<tracking_frame_
          <<"\n lookup_transform_timeout_sec_ = "<<lookup_transform_timeout_sec_;
      }

std::unique_ptr<::cartographer::transform::Rigid3d> TfBridge::LookupToTracking(
    const ::cartographer::common::Time time,
    const std::string& frame_id) const {
  ::ros::Duration timeout(lookup_transform_timeout_sec_);
  std::unique_ptr<::cartographer::transform::Rigid3d> frame_id_to_tracking;
  try {
    /**
     * 雷达数据：
     *    tracking_frame_ = base_link
     *    frame_id = laser
     * 里程计数据:
     *    tracking_frame_ = base_link
     *    frame_id = base_link
     * 陀螺仪数据:
     *    tracking_frame_ = base_link
     *    frame_id = base_link
     * **/ 
    // 获取最新转换时间
    const ::ros::Time latest_tf_time =
        buffer_
            ->lookupTransform(tracking_frame_, frame_id, ::ros::Time(0.),
                              timeout)
            .header.stamp;
    
    // 设置我们需要的转换的时间戳
    const ::ros::Time requested_time = ToRos(time);
    if (latest_tf_time >= requested_time) {
      // We already have newer data, so we do not wait. Otherwise, we would wait
      // for the full 'timeout' even if we ask for data that is too old.
      timeout = ::ros::Duration(0.);
    }
    /**
     *  latest_tf_time > requested_time
     *    最新数据时间大于requested_time，不需要等待，设置timeout = 0
     *  否则
     *    timeout = lookup_transform_timeout_sec_ ,全时等待
     *  返回转换 Tbase_odom（处理里程计） Tbase_laser(雷达数据)
     * **/
    return absl::make_unique<::cartographer::transform::Rigid3d>(
        ToRigid3d(buffer_->lookupTransform(tracking_frame_, frame_id,
                                           requested_time, timeout)));
  } catch (const tf2::TransformException& ex) {
    LOG(WARNING) << ex.what();
  }
  return nullptr;
}

}  // namespace cartographer_ros
