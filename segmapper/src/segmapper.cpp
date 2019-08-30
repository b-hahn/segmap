#include "segmapper/segmapper.hpp"

#include <stdlib.h>

#include <laser_slam/benchmarker.hpp>
#include <laser_slam/common.hpp>
#include <laser_slam_ros/common.hpp>
#include <ros/ros.h>
#include <segmatch/utilities.hpp>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <chrono>

using namespace laser_slam;
using namespace laser_slam_ros;
using namespace segmatch;
using namespace segmatch_ros;

SegMapper::SegMapper(ros::NodeHandle& n) : nh_(n) {
  // pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);

  // Load ROS parameters from server.
  getParameters();

  // TODO: it would be great to have a cleaner check here, e.g. by having the segmenter interface
  // telling us if normals are needed or not. Unfortunately, at the moment the segmenters are
  // created much later ...
  const std::string& segmenter_type =
      segmatch_worker_params_.segmatch_params.segmenter_params.segmenter_type;
  const bool needs_normal_estimation =
      (segmenter_type == "SimpleSmoothnessConstraints") ||
      (segmenter_type == "IncrementalSmoothnessConstraints");

  // Configure benchmarker
  Benchmarker::setParameters(benchmarker_params_);

  // Create an incremental estimator.
  std::shared_ptr<IncrementalEstimator> incremental_estimator(
      new IncrementalEstimator(params_.online_estimator_params, params_.number_of_robots));

  incremental_estimator_ = incremental_estimator;

  // Create local map publisher
  local_maps_mutexes_ = std::vector<std::mutex>(params_.number_of_robots);
  if (laser_slam_worker_params_.publish_local_map) {
    local_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        laser_slam_worker_params_.local_map_pub_topic,
        kPublisherQueueSize);
  }

  // Setup the laser_slam workers.
  ROS_INFO_STREAM("Number of laser_slam workers: " << params_.number_of_robots);
  for (unsigned int i = 0u; i < params_.number_of_robots; ++i) {
    // Adjust the topics and frames for that laser_slam worker.
    LaserSlamWorkerParams params = laser_slam_worker_params_;

    // Create a local map for each robot.
    std::unique_ptr<NormalEstimator> normal_estimator = nullptr;
    if (needs_normal_estimation) {
      normal_estimator = NormalEstimator::create(
          segmatch_worker_params_.segmatch_params.normal_estimator_type,
          segmatch_worker_params_.segmatch_params.radius_for_normal_estimation_m);
    }
    local_maps_.emplace_back(
        segmatch_worker_params_.segmatch_params.local_map_params, std::move(normal_estimator));

    // TODO rm offset when updating mr_foundry.
    const unsigned int offset = 0;
    if (params_.number_of_robots > 1) {
      // Subscribers.

      params.assembled_cloud_sub_topic = "/" + params_.robot_prefix + std::to_string(i + offset) +
          "/" + laser_slam_worker_params_.assembled_cloud_sub_topic;

      // TF frames.
      params.odom_frame =  params_.robot_prefix + std::to_string(i + offset) +
          "/" + laser_slam_worker_params_.odom_frame;
      params.sensor_frame =  params_.robot_prefix + std::to_string(i + offset) +
          "/" + laser_slam_worker_params_.sensor_frame;

      // Publishers.
      params.trajectory_pub_topic = params_.robot_prefix + std::to_string(i + offset) + "/" +
          laser_slam_worker_params_.trajectory_pub_topic;

      params.local_map_pub_topic = params_.robot_prefix + std::to_string(i + offset) + "/" +
          laser_slam_worker_params_.local_map_pub_topic;
    }

    LOG(INFO) << "Robot " << i << " subscribes to " << params.assembled_cloud_sub_topic << " "
        << params.odom_frame << " and " << params.sensor_frame;

    LOG(INFO) << "Robot " << i << " publishes to " << params.trajectory_pub_topic << " and "
        << params.local_map_pub_topic;

    std::unique_ptr<LaserSlamWorker> laser_slam_worker(new LaserSlamWorker());
    laser_slam_worker->init(nh_, params, incremental_estimator_, i);
    laser_slam_workers_.push_back(std::move(laser_slam_worker));
  }

  // Advertise the save_map service.
  save_map_ = nh_.advertiseService("save_map", &SegMapper::saveMapServiceCall, this);
  save_local_map_ = nh_.advertiseService("save_local_map", &SegMapper::saveLocalMapServiceCall, this);

  // Initialize the SegMatchWorker.
  if (segmatch_worker_params_.localize || segmatch_worker_params_.close_loops) {
    segmatch_worker_.init(n, segmatch_worker_params_, params_.number_of_robots);
  }

  for (size_t i = 0u; i < laser_slam_workers_.size(); ++i) {
      skip_counters_.push_back(0u);
      first_points_received_.push_back(false);
  }

  boost::posix_time::ptime my_posix_time = boost::posix_time::microsec_clock::universal_time();
  std::string current_datetime = boost::posix_time::to_iso_extended_string(my_posix_time);
  pose_file_name = "localization_log_" + current_datetime + ".txt";
  LOG(INFO) << "Logging localizations to " << pose_file_name;

  segment_plot_file_name = "segment_log_" + current_datetime + ".txt";
  LOG(INFO) << "Logging segment data to " << segment_plot_file_name;

  source_centroids_file_name = "source_centroids_log_" + current_datetime + ".txt";
  LOG(INFO) << "Logging source centroid data to " << segment_plot_file_name;

  odometry_file_name = "odometry_log_" + current_datetime + ".txt";
  LOG(INFO) << "Logging odometry data to " << odometry_file_name;
  }

SegMapper::~SegMapper() {}

void SegMapper::publishMapThread() {
  // Check if map publication is required.
  if (!laser_slam_worker_params_.publish_local_map)
    return;

  ros::Rate thread_rate(laser_slam_worker_params_.map_publication_rate_hz);
  while (ros::ok()) {
    LOG(INFO) << "publishing local maps";
    MapCloud local_maps;
    for (size_t i = 0u; i < local_maps_.size(); ++i) {
      std::unique_lock<std::mutex> map_lock(local_maps_mutexes_[i]);
      local_maps += local_maps_[i].getFilteredPoints();
    //   if (local_maps.size() > 0) {
    //     LOG(INFO) << "local_maps[0].rgb: " << std::to_string(local_maps[0].rgb);
    //   }
      map_lock.unlock();
    }
    sensor_msgs::PointCloud2 msg;
    laser_slam_ros::convert_to_point_cloud_2_msg(
        local_maps,
        params_.world_frame, &msg);
    local_map_pub_.publish(msg);
    thread_rate.sleep();
  }
}

void SegMapper::publishTfThread() {
  if (params_.publish_world_to_odom) {
    ros::Rate thread_rate(params_.tf_publication_rate_hz);
    while (ros::ok()) {
      for (size_t i = 0u; i < laser_slam_workers_.size(); ++i) {
        tf::StampedTransform world_to_odom = laser_slam_workers_[i]->getWorldToOdom();
        world_to_odom.stamp_ = ros::Time::now();
        tf_broadcaster_.sendTransform(world_to_odom);
      }
      thread_rate.sleep();
    }
  }
}

void SegMapper::segMatchThread() {
  // Terminate the thread if localization and loop closure are not needed.
  if ((!segmatch_worker_params_.localize &&
      !segmatch_worker_params_.close_loops) ||
      laser_slam_workers_.empty())
    return;

  unsigned int track_id = laser_slam_workers_.size() - 1u;
  // Number of tracks skipped because waiting for new voxels to activate.
  unsigned int skipped_tracks_count = 0u;
  ros::Duration sleep_duration(kSegMatchSleepTime_s);

  unsigned int n_loops = 0u;
  int loc_counter = 0;

  while (ros::ok()) {
    // If all the tracks have been skipped consecutively, sleep for a bit to
    // free some CPU time.
    // commented out to ensure localization happens whenever possible
    // if (skipped_tracks_count == laser_slam_workers_.size()) {
    //   skipped_tracks_count = 0u;
    //   sleep_duration.sleep();
    // }

    // Make sure that all the measurements in this loop iteration will get the same timestamp. This
    // makes it easier to plot the data.
    BENCHMARK_START_NEW_STEP();
    // No, we don't include sleeping in the timing, as it is an intended delay.
    BENCHMARK_START("SM");
    // Set the next source cloud to process.
    track_id = (track_id + 1u) % laser_slam_workers_.size();

    // Get the queued points.
    auto new_points = laser_slam_workers_[track_id]->getQueuedPoints();
    if (new_points.empty()) {
      BENCHMARK_STOP_AND_IGNORE("SM");
      ++skipped_tracks_count;
      // Keep asking for publishing to increase the publishing counter.
      segmatch_worker_.publish();
      continue;
    } else {
        if (!first_points_received_[track_id]) {
            first_points_received_[track_id] = true;
            skip_counters_[track_id] = 0u;
        }
    }

    // Update the local map with the new points and the new pose.
    Pose current_pose = incremental_estimator_->getCurrentPose(track_id);
    {
      std::lock_guard<std::mutex> map_lock(local_maps_mutexes_[track_id]);
      local_maps_[track_id].updatePoseAndAddPoints(new_points, current_pose);
    }

    // LOG(INFO) << "ts = " << current_pose.time_ns << "at scan_cb_counter = "
    //           << laser_slam_workers_[0]->scan_cb_counter << " and ros time: "
    //           << ros::Time::now() << "and addition:" << current_pose.time_ns + laser_slam_workers_[0]->returnBaseTimeNs();

// store segment size, local map size, time, and whether there was a localization and store loc dist.
    segment_plot_file.open(segment_plot_file_name, std::ios_base::app);
    segment_plot_file << current_pose.time_ns +
                             laser_slam_workers_[0]->returnBaseTimeNs()
                      << " ";
    segment_plot_file << local_maps_[track_id].getFilteredPointsPtr()->size()
                      << " "; // local map size
    segment_plot_file << segmatch_worker_.returnSourceSegmentsPtr()->size()
                      << "\n"; // num segments
    segment_plot_file.close();
    LOG(INFO) << current_pose.time_ns +
                     laser_slam_workers_[0]->returnBaseTimeNs()
              << " " << local_maps_[track_id].getFilteredPointsPtr()->size()
              << " " << segmatch_worker_.returnTargetSegmentsPtr()->size()
              << " " << segmatch_worker_.returnSourceSegmentsPtr()->size();

    // write segment centroids to file
    std::vector<Id> ids;
    auto source_centroids = segmatch_worker_.returnSourceSegmentsPtr()->centroidsAsPointCloud(ids);
    source_centroids_file.open(source_centroids_file_name, std::ios_base::app);
    for (uint16_t i = 0; i < source_centroids.size(); ++i) {
        source_centroids_file << current_pose.time_ns + laser_slam_workers_[0]->returnBaseTimeNs() << " "
                              << laser_slam_workers_[0]->scan_cb_counter << " ";
        source_centroids_file << std::to_string(ids[i]) << " ";
        source_centroids_file << source_centroids[i].x << " " << source_centroids[i].y << " " << source_centroids[i].z
                              << "\n"; // target segment centroid x, y and z coords
    }
    source_centroids_file.close();

    // write current pose to file
    odometry_file.open(odometry_file_name, std::ios_base::app);
    odometry_file << current_pose.time_ns + laser_slam_workers_[0]->returnBaseTimeNs() << " "
                  << laser_slam_workers_[0]->scan_cb_counter << " ";
    odometry_file << current_pose.T_w.getPosition()[0] << " " << current_pose.T_w.getPosition()[1] << " "
                  << current_pose.T_w.getPosition()[2] << "\n"; // current pose
    odometry_file.close();

    std::string red = "\u001b[31;1m";
    std::string reset = "\u001b[0m";
    // Process the source cloud.
    if (segmatch_worker_params_.localize) {
      if (segmatch_worker_.processLocalMap(local_maps_[track_id], current_pose, track_id)) {
          // TODO: check here to see if localized - count how often this happens, this should be the same as total number of clouds.
          // might have to run bag at half speed
          // num clouds = probably num calls of ScanCallback
          // TODO: fix this to work with multiple robots/ laser_slam_workers (need to know which at what frame number
          // the localization occurred for each robot)
          CHECK(laser_slam_workers_.size() == 1) << "Number of robots (or laser_slam_workers) not equal to 1; no clear "
                                                    "definition of current number of scans/frames.";
          auto matches = segmatch_worker_.getMatches();
          float distance = 0;
          float dist_x, dist_y, dist_z;
          uint16_t num_matches = matches.size();
          for (uint8_t i = 0; i < num_matches; ++i) {
            dist_x = (matches[i].getCentroids().second.x - matches[i].getCentroids().first.x);
            dist_y = (matches[i].getCentroids().second.y - matches[i].getCentroids().first.y);
            dist_z = (matches[i].getCentroids().second.z - matches[i].getCentroids().first.z);
              distance += sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z);
          }
          distance /= num_matches;
          BENCHMARK_RECORD_VALUE("SM.MeanMatchesDistance", distance);

          // compute mean distance of matches - should be low for correct matches
          BENCHMARK_RECORD_VALUE("SM.NumMatchesPerLocalization", num_matches);
          LOG(INFO) << "Successful Localization with " << std::to_string(num_matches)
                    << red << " matches with mean distance = " << std::to_string(distance) << "!" << reset << " loc_counter = " << loc_counter
                    << ", scan_cb_counter = " << laser_slam_workers_[0]->scan_cb_counter << "("
                    << static_cast<float>(loc_counter) / laser_slam_workers_[0]->scan_cb_counter * 100
                    << "% frames localized), ts = " << current_pose.time_ns;
          // draw localization TF
          Eigen::Matrix4f localization_pose = segmatch_worker_.getLatestLocalizationTransformation(track_id);
          Eigen::Matrix4f map_pose = localization_pose * current_pose.T_w.getTransformationMatrix().cast<float>();
        //   for (uint8_t r = 0; r < localization_pose.rows(); ++r) {
        //     std::cout << std::setprecision(4) << localization_pose.block<1,4>(r, 0) << "\tand\t\t" << current_pose.T_w.getTransformationMatrix().block<1,4>(r, 0) << "\t-->\t\t"
        //               << map_pose.block<1,4>(r, 0) << std::endl;
        //   }

          pose_file.open(pose_file_name, std::ios_base::app);
          pose_file << laser_slam_workers_[0]->scan_cb_counter << "\n";
          pose_file << current_pose.time_ns + laser_slam_workers_[0]->returnBaseTimeNs() << "\n";
          // pose_file << current_pose.T_w.getPosition() << "\n\n";
          pose_file << map_pose.block<3,1>(0, 3) << "\n\n";
          pose_file.close();

          loc_counter++;
        if (!pose_at_last_localization_set_) {
          pose_at_last_localization_set_ = true;
          pose_at_last_localization_ = current_pose.T_w;
        } else {
          BENCHMARK_RECORD_VALUE("SM.LocalizationDistances", distanceBetweenTwoSE3(
              pose_at_last_localization_, current_pose.T_w));
          pose_at_last_localization_ = current_pose.T_w;
        }
        // count number of segments on localization
        // TODO: implement
        // BENCHMARK_RECORD_VALUE("SM.NumSegmentsOnLocalization", laser_slam_workers_[0]-> "Get num Segments!1");

        // count mean number of points per segment on loclaization
        // TODO: implement
        // BENCHMARK_RECORD_VALUE("SM.NumSegmentsOnLocalization", laser_slam_workers_[0]-> "Get num pts for all Segments!!");

        // count number of localizations
        // TODO: fix
        // BENCHMARK_RECORD_VALUE("SM.NumLocalizations", 1);
      }
    } else {
      double max_time_for_lc = 10000;
      if (current_pose.time_ns < max_time_for_lc*1e9) {

      RelativePose loop_closure;

      // If there is a loop closure.
      if (segmatch_worker_.processLocalMap(local_maps_[track_id], current_pose,
                                           track_id, &loop_closure)) {
        uint16_t num_matches = segmatch_worker_.getMatches().size();
        BENCHMARK_BLOCK("SM.ProcessLoopClosure");
        LOG(INFO) << "Found loop closure! track_id_a: " << loop_closure.track_id_a
                  << " time_a_ns: " << loop_closure.time_a_ns << " track_id_b: " << loop_closure.track_id_b
                  << " time_b_ns: " << loop_closure.time_b_ns << red << " num_matches: " << std::to_string(num_matches)
                  << reset;

        // Prevent the workers to process further scans (and add variables to the graph).
        BENCHMARK_START("SM.ProcessLoopClosure.WaitingForLockOnLaserSlamWorkers");
        for (auto& worker: laser_slam_workers_) {
          worker->setLockScanCallback(true);
        }
        BENCHMARK_STOP("SM.ProcessLoopClosure.WaitingForLockOnLaserSlamWorkers");

        // Save last poses for updating the local maps.
        BENCHMARK_START("SM.ProcessLoopClosure.GettingLastPoseOfTrajectories");
        Trajectory trajectory;
        std::vector<SE3> last_poses_before_update;
        std::vector<laser_slam::Time> last_poses_timestamp_before_update_ns;
        if (!params_.clear_local_map_after_loop_closure) {
          for (const auto& worker: laser_slam_workers_) {
            worker->getTrajectory(&trajectory);
            last_poses_before_update.push_back(trajectory.rbegin()->second);
            last_poses_timestamp_before_update_ns.push_back(trajectory.rbegin()->first);
          }
        }
        BENCHMARK_STOP("SM.ProcessLoopClosure.GettingLastPoseOfTrajectories");

        BENCHMARK_START("SM.ProcessLoopClosure.UpdateIncrementalEstimator");
        incremental_estimator_->processLoopClosure(loop_closure);
        BENCHMARK_STOP("SM.ProcessLoopClosure.UpdateIncrementalEstimator");

        BENCHMARK_START("SM.ProcessLoopClosure.ProcessLocalMap");
        for (size_t i = 0u; i < laser_slam_workers_.size(); ++i) {
          if (!params_.clear_local_map_after_loop_closure) {
            laser_slam::SE3 local_map_update_transform =
                laser_slam_workers_[i]->getTransformBetweenPoses(
                    last_poses_before_update[i], last_poses_timestamp_before_update_ns[i]);
            std::unique_lock<std::mutex> map_lock2(local_maps_mutexes_[i]);
            local_maps_[i].transform(local_map_update_transform.cast<float>());
            map_lock2.unlock();
          } else {
            std::unique_lock<std::mutex> map_lock2(local_maps_mutexes_[i]);
            local_maps_[i].clear();
            map_lock2.unlock();
          }
        }
        BENCHMARK_STOP("SM.ProcessLoopClosure.ProcessLocalMap");

        MapCloud local_maps;
        for (size_t i = 0u; i < local_maps_.size(); ++i) {
          std::unique_lock<std::mutex> map_lock(local_maps_mutexes_[i]);
          local_maps += local_maps_[i].getFilteredPoints();
          map_lock.unlock();
        }
        sensor_msgs::PointCloud2 msg;
        laser_slam_ros::convert_to_point_cloud_2_msg(
            local_maps,
            params_.world_frame, &msg);
        local_map_pub_.publish(msg);

        // Update the Segmatch object.
        std::vector<Trajectory> updated_trajectories;
        for (const auto& worker: laser_slam_workers_) {
          worker->getTrajectory(&trajectory);
          updated_trajectories.push_back(trajectory);
        }

        BENCHMARK_START("SM.ProcessLoopClosure.UpdateSegMatch");
        segmatch_worker_.update(updated_trajectories);
        BENCHMARK_STOP("SM.ProcessLoopClosure.UpdateSegMatch");

        //Publish the trajectories.
        for (const auto& worker : laser_slam_workers_) {
          worker->publishTrajectories();
        }

        // Unlock the workers.
        for (auto& worker: laser_slam_workers_) {
          worker->setLockScanCallback(false);
        }

        n_loops++;
        LOG(INFO) << "That was the loop number " << n_loops << ".";
      }

      for (const auto& worker : laser_slam_workers_) {
        worker->publishTrajectories();
      }
    }
    else
    {
        LOG(INFO) << "Time greater than " << std::to_string(max_time_for_lc)  << "s; ignoring loop closures!";
    }
    }

    // The track was processed, reset the counter.
    skipped_tracks_count = 0;
    skip_counters_[track_id] = 0u;
    BENCHMARK_STOP("SM");
  }

  Benchmarker::logStatistics(LOG(INFO));
  Benchmarker::saveData();
}

bool SegMapper::saveMapServiceCall(segmapper::SaveMap::Request& request,
                                   segmapper::SaveMap::Response& response) {
  try {
    pcl::io::savePCDFileASCII(request.filename.data,
                              local_maps_.front().getFilteredPoints());
  }
  catch (const std::runtime_error& e) {
    ROS_ERROR_STREAM("Unable to save: " << e.what());
    return false;
  }
  return true;
}

bool SegMapper::saveLocalMapServiceCall(segmapper::SaveMap::Request& request,
                                        segmapper::SaveMap::Response& response) {
  // TODO this is saving only the local map of worker ID 0.
  std::unique_lock<std::mutex> map_lock(local_maps_mutexes_[0]);
  MapCloud local_map;
  local_map += local_maps_[0].getFilteredPoints();
  map_lock.unlock();
  try {
    pcl::io::savePCDFileASCII(request.filename.data, mapPoint2PointCloud(local_map));
  }
  catch (const std::runtime_error& e) {
    ROS_ERROR_STREAM("Unable to save: " << e.what());
    return false;
  }
  return true;
}

void SegMapper::getParameters() {
  // SegMapper parameters.
  const std::string ns = "/SegMapper";
  nh_.getParam(ns + "/number_of_robots",
               params_.number_of_robots);
  nh_.getParam(ns + "/robot_prefix",
               params_.robot_prefix);

  CHECK_GE(params_.number_of_robots, 0u);

  nh_.getParam(ns + "/publish_world_to_odom",
               params_.publish_world_to_odom);
  nh_.getParam(ns + "/world_frame",
               params_.world_frame);
  nh_.getParam(ns + "/tf_publication_rate_hz",
               params_.tf_publication_rate_hz);

  nh_.getParam(ns + "/clear_local_map_after_loop_closure",
               params_.clear_local_map_after_loop_closure);

  // laser_slam worker parameters.
  laser_slam_worker_params_ = laser_slam_ros::getLaserSlamWorkerParams(nh_, ns);
  laser_slam_worker_params_.world_frame = params_.world_frame;

  // Online estimator parameters.
  params_.online_estimator_params = laser_slam_ros::getOnlineEstimatorParams(nh_, ns);

  // Benchmarker parameters.
  benchmarker_params_ = laser_slam_ros::getBenchmarkerParams(nh_, ns);

  // ICP configuration files.
  nh_.getParam("icp_configuration_file",
               params_.online_estimator_params.laser_track_params.icp_configuration_file);
  nh_.getParam("icp_input_filters_file",
               params_.online_estimator_params.laser_track_params.icp_input_filters_file);

  // SegMatchWorker parameters.
  segmatch_worker_params_ = segmatch_ros::getSegMatchWorkerParams(nh_, ns);
  segmatch_worker_params_.world_frame = params_.world_frame;
}
