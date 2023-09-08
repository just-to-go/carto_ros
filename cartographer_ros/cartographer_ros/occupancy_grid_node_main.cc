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

#include <cmath>
#include <string>
#include <vector>

#include <std_msgs/Int32.h>
#include <boost/thread.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <fstream>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "absl/synchronization/mutex.h"
#include "cairo/cairo.h"
#include "cartographer/common/port.h"
#include "cartographer/io/image.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/submap.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "gflags/gflags.h"
// #include "jsonoption/json.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"

DEFINE_double(resolution, 0.05,
              "Resolution of a grid cell in the published occupancy grid.");
DEFINE_double(publish_period_sec, 5.0, "OccupancyGrid publishing period.");
DEFINE_bool(include_frozen_submaps, true,
            "Include frozen submaps in the occupancy grid.");
DEFINE_bool(include_unfrozen_submaps, true,
            "Include unfrozen submaps in the occupancy grid.");
DEFINE_string(occupancy_grid_topic, cartographer_ros::kOccupancyGridTopic,
              "Name of the topic on which the occupancy grid is published.");

namespace cartographer_ros {
namespace {

bool CreateFile(const std::string& filepath) {
  if (access(filepath.c_str(), F_OK | R_OK | W_OK) == 0) {
    return true;
  } else {
    std::string Cmd = "mkdir " + filepath;
    if (system(Cmd.c_str())) {
      return true;
    } else {
      return false;
    }
  }
}

std::string GetCmdResult(std::string cmd) {
  FILE* pipe = popen(cmd.c_str(), "r");
  if (!pipe) return "ERROR";
  char buffer[128];
  std::string result = "";
  while (!feof(pipe)) {
    if (fgets(buffer, 128, pipe) != NULL) {
      int index = strlen(buffer) - 1;
      if (buffer[index] == '\n') {
        buffer[index] = '\0';
      }
    }
  }
  result = buffer;

  pclose(pipe);
  return result;
}

void ObtainNewMapInfo(std::string& map_id, std::string& url, int& floor) {
  // Json::Reader reader;
  // Json::Value root;
  // std::ifstream is;
  // is.open("/tmp/new_map.json", std::ios::binary);
  // if (reader.parse(is, root)) {
  //   map_id = root["map_id"].asString();
  //   url = root["url"].asString();
  //   floor = root["floor"].asInt();
  // }
  // is.close();
  // root.clear();
}

void MapUpload(std::string url, std::string map_path, std::string map_id,
               int* flag) {
  std::string map_file = map_path + map_id;
  GetCmdResult("tar -czvf " + map_file + ".tar " + map_file);
  std::string res = GetCmdResult("curl -F 'file=@" + map_file + ".tar' " + url);
  GetCmdResult("rm " + map_file + ".tar");
  if (*flag == 1) {
    *flag = 2;
  }
}

using ::cartographer::io::PaintSubmapSlicesResult;
using ::cartographer::io::SubmapSlice;
using ::cartographer::mapping::SubmapId;

class Node {
 public:
  explicit Node(double resolution, double publish_period_sec);
  ~Node() {}

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

 private:
  void HandleSubmapList(const cartographer_ros_msgs::SubmapList::ConstPtr& msg);
  void DrawAndPublish(const ::ros::WallTimerEvent& timer_event);

  ::ros::NodeHandle node_handle_;
  const double resolution_;

  absl::Mutex mutex_;
  ::ros::ServiceClient client_ GUARDED_BY(mutex_);
  ::ros::Subscriber submap_list_subscriber_ GUARDED_BY(mutex_);
  ::ros::Publisher occupancy_grid_publisher_ GUARDED_BY(mutex_);
  std::map<SubmapId, SubmapSlice> submap_slices_ GUARDED_BY(mutex_);
  ::ros::WallTimer occupancy_grid_publisher_timer_;
  std::string last_frame_id_;
  ros::Time last_timestamp_;

  ros::Publisher mapping_logout_pub_;
  ros::Subscriber mapping_logout_sub_;
  std::string map_path_;
  std::string map_id_;
  std::string url_;
  int floor_;
  int log_out_flag_;
  void MappingLogoutCallback(const std_msgs::Int32::ConstPtr& msg);
};

void Node::MappingLogoutCallback(const std_msgs::Int32::ConstPtr& msg) {
  if (msg->data == 1 && log_out_flag_ == 0) {
    log_out_flag_ = 1;
    LOG(WARNING) << "recevived log out msg";
  }
}

Node::Node(const double resolution, const double publish_period_sec)
    : resolution_(resolution),
      client_(node_handle_.serviceClient<::cartographer_ros_msgs::SubmapQuery>(
          kSubmapQueryServiceName)),
      submap_list_subscriber_(node_handle_.subscribe(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize,
          boost::function<void(
              const cartographer_ros_msgs::SubmapList::ConstPtr&)>(
              [this](const cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
                HandleSubmapList(msg);
              }))),
      occupancy_grid_publisher_(
          node_handle_.advertise<::nav_msgs::OccupancyGrid>(
              FLAGS_occupancy_grid_topic, kLatestOnlyPublisherQueueSize,
              true /* latched */)),
      occupancy_grid_publisher_timer_(node_handle_.createWallTimer(
          ::ros::WallDuration(publish_period_sec), &Node::DrawAndPublish, this))

{
  // mapping_logout_pub_ =
  //     node_handle_.advertise<std_msgs::Int32>("mapping_logout_ack", 1);
  // mapping_logout_sub_ = node_handle_.subscribe(
  //     "mapping_logout_request", 5, &Node::MappingLogoutCallback, this);
  // map_path_ = GetCmdResult("rospack find agv_master") + "/maps/";
  // CreateFile(map_path_);
  // ObtainNewMapInfo(map_id_, url_, floor_);
  // CreateFile(map_path_ + map_id_);
  // log_out_flag_ = 0;
}

void Node::HandleSubmapList(
    const cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
  absl::MutexLock locker(&mutex_);

  // LOG(INFO)<<"debug!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 0 ";

  // We do not do any work if nobody listens.
  // if (occupancy_grid_publisher_.getNumSubscribers() == 0) {
  //   return;
  // }

  // Keep track of submap IDs that don't appear in the message anymore.
  std::set<SubmapId> submap_ids_to_delete;
  for (const auto& pair : submap_slices_) {
    submap_ids_to_delete.insert(pair.first);
  }

  for (const auto& submap_msg : msg->submap) {
    const SubmapId id{submap_msg.trajectory_id, submap_msg.submap_index};
    submap_ids_to_delete.erase(id);
    if ((submap_msg.is_frozen && !FLAGS_include_frozen_submaps) ||
        (!submap_msg.is_frozen && !FLAGS_include_unfrozen_submaps)) {
      continue;
    }
    SubmapSlice& submap_slice = submap_slices_[id];
    submap_slice.pose = ToRigid3d(submap_msg.pose);
    submap_slice.metadata_version = submap_msg.submap_version;
    if (submap_slice.surface != nullptr &&
        submap_slice.version == submap_msg.submap_version) {
      continue;
    }

    auto fetched_textures =
        ::cartographer_ros::FetchSubmapTextures(id, &client_);
    if (fetched_textures == nullptr) {
      continue;
    }
    CHECK(!fetched_textures->textures.empty());
    submap_slice.version = fetched_textures->version;

    // We use the first texture only. By convention this is the highest
    // resolution texture and that is the one we want to use to construct the
    // map for ROS.
    const auto fetched_texture = fetched_textures->textures.begin();
    submap_slice.width = fetched_texture->width;
    submap_slice.height = fetched_texture->height;
    submap_slice.slice_pose = fetched_texture->slice_pose;
    submap_slice.resolution = fetched_texture->resolution;
    submap_slice.cairo_data.clear();
    submap_slice.surface = ::cartographer::io::DrawTexture(
        fetched_texture->pixels.intensity, fetched_texture->pixels.alpha,
        fetched_texture->width, fetched_texture->height,
        &submap_slice.cairo_data);
  }

  // Delete all submaps that didn't appear in the message.
  for (const auto& id : submap_ids_to_delete) {
    submap_slices_.erase(id);
  }

  last_timestamp_ = msg->header.stamp;
  last_frame_id_ = msg->header.frame_id;
}

void Node::DrawAndPublish(const ::ros::WallTimerEvent& unused_timer_event) {
  absl::MutexLock locker(&mutex_);
  // if (submap_slices_.empty() || last_frame_id_.empty()) {
  //   return;
  // }
  // auto painted_slices = PaintSubmapSlices(submap_slices_, resolution_);
  // bool completed = true;

  // std::unique_ptr<nav_msgs::OccupancyGrid> msg_ptr =
  //     CreateOccupancyGridMsg(completed, map_path_, map_id_, floor_, painted_slices,
  //                            resolution_, last_frame_id_, last_timestamp_);
  // occupancy_grid_publisher_.publish(*msg_ptr);

  // boost::thread upload_thread(
  //     boost::bind(MapUpload, url_, map_path_, map_id_, &log_out_flag_));
  // upload_thread.detach();

  // if (log_out_flag_ == 2) {
  //   ros::Duration(5.0).sleep();
  //   std_msgs::Int32 msg;
  //   msg.data = 1;
  //   mapping_logout_pub_.publish(msg);
  //   LOG(WARNING) << "publish mapping log out msg";
  // }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  google::SetStderrLogging(google::GLOG_INFO);
  FLAGS_logbufsecs = 0;
  FLAGS_colorlogtostderr = true;

  CHECK(FLAGS_include_frozen_submaps || FLAGS_include_unfrozen_submaps)
      << "Ignoring both frozen and unfrozen submaps makes no sense.";

  ::ros::init(argc, argv, "cartographer_occupancy_grid_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  ::cartographer_ros::Node node(FLAGS_resolution, FLAGS_publish_period_sec);

  ::ros::spin();
  ::ros::shutdown();
}
