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
#include <string>
#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/TrajectoryOptions.h"
#include "gflags/gflags.h"
#include "ros/ros.h"

// ed: header added
#include <chrono>
#include <thread>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include "cartographer/io/proto_stream.h"
#include "cartographer_ros/submap.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <tf_conversions/tf_kdl.h>

using namespace std;

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(initial_pose, "", "Starting pose of a new trajectory");

// ed: pbstream filename to get last trajectory's position(x,y,z) value 
DEFINE_string(pbstream_filename, "",
              "Filename of a pbstream to draw a map from.");

namespace cartographer_ros {
  namespace {

    TrajectoryOptions LoadOptions() {
      auto file_resolver = cartographer::common::make_unique<
        cartographer::common::ConfigurationFileResolver>(
                                                         std::vector<std::string>{FLAGS_configuration_directory});

      const std::string code =
        file_resolver->GetFileContentOrDie(FLAGS_configuration_basename);
      auto lua_parameter_dictionary =
        cartographer::common::LuaParameterDictionary::NonReferenceCounted(
                                                                          code, std::move(file_resolver));

      // ed: if there is initial_pose value
      if (!FLAGS_initial_pose.empty()) {

        auto initial_trajectory_pose_file_resolver =
          cartographer::common::make_unique<
            cartographer::common::ConfigurationFileResolver>(
                                                             std::vector<std::string>{FLAGS_configuration_directory});

        auto initial_trajectory_pose =
          cartographer::common::LuaParameterDictionary::NonReferenceCounted(
                                                                            "return " + FLAGS_initial_pose,
                                                                            std::move(initial_trajectory_pose_file_resolver));

        return CreateTrajectoryOptions(lua_parameter_dictionary.get(),
                                       initial_trajectory_pose.get());

      }
      else {
        return CreateTrajectoryOptions(lua_parameter_dictionary.get());
      }
    }


    bool Run() {
      ros::NodeHandle node_handle;
      ros::ServiceClient client =
        node_handle.serviceClient<cartographer_ros_msgs::StartTrajectory>(
                                                                          kStartTrajectoryServiceName);
      cartographer_ros_msgs::StartTrajectory srv;
      srv.request.options = ToRosMessage(LoadOptions());
      srv.request.topics.laser_scan_topic = node_handle.resolveName(
                                                                    kLaserScanTopic, true /* apply topic remapping */);
      srv.request.topics.multi_echo_laser_scan_topic =
        node_handle.resolveName(kMultiEchoLaserScanTopic, true);
      srv.request.topics.point_cloud2_topic =
        node_handle.resolveName(kPointCloud2Topic, true);
      srv.request.topics.imu_topic = node_handle.resolveName(kImuTopic, true);
      srv.request.topics.odometry_topic =
        node_handle.resolveName(kOdometryTopic, true);

      if (!client.call(srv)) {
        LOG(ERROR) << "Failed to call " << kStartTrajectoryServiceName << ".";
        return false;
      }
      if (srv.response.status.code != cartographer_ros_msgs::StatusCode::OK) {
        LOG(ERROR) << "Error starting trajectory - message: '"
                   << srv.response.status.message
                   << "' (status code: " << std::to_string(srv.response.status.code)
                   << ").";
        return false;
      }
      LOG(INFO) << "Started trajectory " << srv.response.trajectory_id;
      return true;
    }



  }  // namespace
}  // namespace cartographer_ros

// ed: /move_base_simple/goal (2D Nav Goal in Rviz) subscribe callback function
void move_base_simple_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){

  //TODO: stop the old trajectory

  std::cout << "[+] CB : /move_base_simple/goal data received! " << std::endl;


  /* check the coincidence of frames */
  ::ros::NodeHandle nhp("~");
  std::string initpose_frame_name, cartographer_tracking_frame_name;
  nhp.param("initpose_frame_name", initpose_frame_name, std::string("base_link"));
  nhp.param("cartographer_tracking_frame_name", cartographer_tracking_frame_name, std::string("base_link"));
  tf::Transform tracking_frame_rot_from_initpose_frame(tf::createIdentityQuaternion());
  if(initpose_frame_name != cartographer_tracking_frame_name)
    {
      tf2_ros::Buffer tfBuffer;
      tf2_ros::TransformListener tfListener(tfBuffer);

      try
        {
          geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform(cartographer_tracking_frame_name, initpose_frame_name, ros::Time(0));
          tf::transformMsgToTF(transformStamped.transform, tracking_frame_rot_from_initpose_frame);
        }
      catch (tf2::TransformException &ex)
        {
          ROS_WARN("%s",ex.what());
          return;
        }
    }

  ::cartographer::io::ProtoStreamReader reader(FLAGS_pbstream_filename);
  cartographer_ros::NodeOptions node_options;
  cartographer_ros::TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) =
      cartographer_ros::LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
  auto map_builder =
      cartographer::common::make_unique<cartographer::mapping::MapBuilder>(
          node_options.map_builder_options);

  // load pbstream
  map_builder->LoadState(&reader, true);

  tf::Transform traj_ref_tf;

  // get TrajectoryNodePose
  const auto node_poses = map_builder->pose_graph()->GetTrajectoryNodePoses();
  const auto traj_ref_pose = node_poses.BeginOfTrajectory(0)->data.global_pose;

  traj_ref_tf.getOrigin() = tf::Vector3(traj_ref_pose.translation().x(), traj_ref_pose.translation().y(), traj_ref_pose.translation().z());
  traj_ref_tf.setRotation(tf::Quaternion(traj_ref_pose.rotation().x(), traj_ref_pose.rotation().y(), traj_ref_pose.rotation().z(), traj_ref_pose.rotation().w()));

  // get init pose w.r.t /map
  tf::Transform map_tf;
  tf::poseMsgToTF(msg->pose.pose, map_tf);
  map_tf *= tracking_frame_rot_from_initpose_frame;
  /* have to set the corret z, with the assumption of 2.5D map */
  const auto submap_poses = map_builder->pose_graph()->GetAllSubmapPoses();
  double min_dist = 1e6;
  tf::Transform best_submap_pose;
  for(auto itr = submap_poses.begin(); itr != submap_poses.end(); ++itr)
    {
      tf::Vector3 submap_origin(itr->data.pose.translation().x(),
                                itr->data.pose.translation().y(),
                                itr->data.pose.translation().z());
      tf::Matrix3x3 rot = tf::Matrix3x3(tf::Quaternion(itr->data.pose.rotation().x(),
                                                       itr->data.pose.rotation().y(),
                                                       itr->data.pose.rotation().z(),
                                                       itr->data.pose.rotation().w()));
      double r,p,y;
      rot.getRPY(r, p, y);

      tf::Vector3 delta_pos = submap_origin - map_tf.getOrigin();
      delta_pos.setZ(0); // only check horinzontal location
      if(delta_pos.length() < min_dist)
        {
          map_tf.getOrigin().setZ(submap_origin.z());
          min_dist = delta_pos.length();
          best_submap_pose.getOrigin() = submap_origin;
        }
    }
  ROS_INFO("best submap for initpose: [%f, %f, %f]",
           best_submap_pose.getOrigin().x(),
           best_submap_pose.getOrigin().y(),
           best_submap_pose.getOrigin().z());


  tf::Transform relative_initpose_tf = traj_ref_tf.inverse() * map_tf;

  double roll, pitch, yaw;
  relative_initpose_tf.getBasis().getRPY(roll, pitch, yaw);

  int to_trajectory_id = 0;   // ed: offline map's trajectory

  ROS_INFO("init pose w.r.t map: [%f, %f, %f], yaw: %f",
           map_tf.getOrigin().x(),
           map_tf.getOrigin().y(),
           map_tf.getOrigin().z(),
           tf::getYaw(map_tf.getRotation()));

  ROS_INFO("relative pose: [%f, %f, %f], [%f, %f, %f]",
           relative_initpose_tf.getOrigin().x(),
           relative_initpose_tf.getOrigin().y(),
           relative_initpose_tf.getOrigin().z(),
           roll, pitch, yaw);

  // ed: pitch isn't used
  std::string parsed_string ="{to_trajectory_id = "    \
                             + std::to_string(to_trajectory_id)         \
                             + ", relative_pose = { translation = { "   \
                             + std::to_string(relative_initpose_tf.getOrigin().x()) + ", " \
                             + std::to_string(relative_initpose_tf.getOrigin().y()) + ", " \
                             + std::to_string(relative_initpose_tf.getOrigin().z()) + \
                             + "}, rotation = { "\
                             + std::to_string(roll) + ","       \
                             + std::to_string(pitch) + ","      \
                             + std::to_string(yaw) + "}}}";


  FLAGS_initial_pose = parsed_string;

  //return;

  {
    cartographer_ros::ScopedRosLogSink ros_log_sink;
    cartographer_ros::Run();

  }
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  google::SetUsageMessage(
      "\n\n"
      "Convenience tool around the start_trajectory service. This takes a Lua "
      "file that is accepted by the node as well and starts a new trajectory "
      "using its settings.\n");

  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";

  ::ros::init(argc, argv, "cartographer_start_trajectory");
  ::ros::start();

  // ed: code added
  ::ros::NodeHandle nh;
  ::ros::Subscriber sub_move_base_simple = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, &move_base_simple_callback);

  ::ros::Rate loop_rate(1);

  // test:
  ::cartographer::io::ProtoStreamReader reader(FLAGS_pbstream_filename);
  cartographer_ros::NodeOptions node_options;
  cartographer_ros::TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) = cartographer_ros::LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
  auto map_builder = cartographer::common::make_unique<cartographer::mapping::MapBuilder>(node_options.map_builder_options);

  map_builder->LoadState(&reader, true);
  const auto node_poses = map_builder->pose_graph()->GetTrajectoryNodePoses();

  double pos_x = (node_poses.BeginOfTrajectory(0))->data.global_pose.translation().x();
  double pos_y = (node_poses.BeginOfTrajectory(0))->data.global_pose.translation().y();
  double pos_z = (node_poses.BeginOfTrajectory(0))->data.global_pose.translation().z();
  double quat_x = ((node_poses.BeginOfTrajectory(0)))->data.global_pose.rotation().x();
  double quat_y = ((node_poses.BeginOfTrajectory(0)))->data.global_pose.rotation().y();
  double quat_z = ((node_poses.BeginOfTrajectory(0)))->data.global_pose.rotation().z();
  double quat_w = ((node_poses.BeginOfTrajectory(0)))->data.global_pose.rotation().w();

  tf::Matrix3x3 rot(tf::Quaternion(quat_x, quat_y, quat_z, quat_w));
  double r,p,y;
  rot.getRPY(r, p, y);
  ROS_INFO("start pos: [%f, %f, %f], [%f, %f, %f]", pos_x, pos_y, pos_z, r, p, y);

  pos_x = std::prev(node_poses.EndOfTrajectory(0))->data.global_pose.translation().x();
  pos_y = std::prev(node_poses.EndOfTrajectory(0))->data.global_pose.translation().y();
  pos_z = std::prev(node_poses.EndOfTrajectory(0))->data.global_pose.translation().z();
  quat_x = (std::prev(node_poses.EndOfTrajectory(0)))->data.global_pose.rotation().x();
  quat_y = (std::prev(node_poses.EndOfTrajectory(0)))->data.global_pose.rotation().y();
  quat_z = (std::prev(node_poses.EndOfTrajectory(0)))->data.global_pose.rotation().z();
  quat_w = (std::prev(node_poses.EndOfTrajectory(0)))->data.global_pose.rotation().w();

  rot = tf::Matrix3x3(tf::Quaternion(quat_x, quat_y, quat_z, quat_w));
  rot.getRPY(r, p, y);
  ROS_INFO("end pos: [%f, %f, %f], [%f, %f, %f]", pos_x, pos_y, pos_z, r, p, y);


  const auto submap_poses = map_builder->pose_graph()->GetAllSubmapPoses();
  int cnt = 0;
  for(auto itr = submap_poses.begin(); itr != submap_poses.end(); ++itr)
    {
      cnt ++;
      pos_x = itr->data.pose.translation().x();
      pos_y = itr->data.pose.translation().y();
      pos_z = itr->data.pose.translation().z();
      quat_x = itr->data.pose.rotation().x();
      quat_y = itr->data.pose.rotation().y();
      quat_z = itr->data.pose.rotation().z();
      quat_w = itr->data.pose.rotation().w();
      rot = tf::Matrix3x3(tf::Quaternion(quat_x, quat_y, quat_z, quat_w));
      rot.getRPY(r, p, y);

      ROS_INFO("submap%d pose: [%f, %f, %f], [%f, %f, %f]", cnt, pos_x, pos_y, pos_z, r, p, y);
    }

  ros::spin();
}

