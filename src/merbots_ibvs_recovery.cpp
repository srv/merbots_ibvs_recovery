//  Copyright (c) 2017 Universitat de les Illes Balears
//  This file is part of Merbots IBVS Recovery.
//
//  Merbots IBVS Recovery is free software: you can redistribute it and/or
//  modify it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  Merbots IBVS Recovery is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with Merbots IBVS Recovery. If not, see
//  <http://www.gnu.org/licenses/>.

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_srvs/Empty.h>
#include <merbots_ibvs/IBVSInfo.h>
#include <auv_msgs/NavSts.h>
#include <cola2_msgs/Goto.h>

// Topic sync
typedef message_filters::sync_policies::ApproximateTime <auv_msgs::NavSts,
  merbots_ibvs::IBVSInfo> SyncPool;

/**
 * @brief      Class for merbots ibvs recovery.
 */
class MerbotsIbvsRecovery {
 public:
  /**
   * @brief      Empty class constructor
   */
  MerbotsIbvsRecovery() : nh_(""), nh_private_("~"), srv_running_(false),
    last_time_seen_(-1.0) {}

  /**
   * @brief      Reads parameters.
   */
  void ReadParams() {
    nh_private_.param("srv_name", srv_name_,
      std::string("/cola2_control/enable_goto"));
    nh_private_.param("disable_srv_name", disable_srv_name_,
      std::string("/cola2_control/disable_goto"));
    nh_private_.param("filter_size", filter_size_, 10);
    nh_private_.param("max_time_diff", max_time_diff_, 30.0);
    nh_private_.param("min_lost_time", min_lost_time_, 5.0);
    nh_private_.param("go_to_linear_velocity_x", go_to_linear_velocity_x_, 0.1);
    nh_private_.param("go_to_linear_velocity_y", go_to_linear_velocity_y_, 0.1);
    nh_private_.param("go_to_linear_velocity_z", go_to_linear_velocity_z_, 0.1);
    nh_private_.param("go_go_angular_velocity_yaw", go_go_angular_velocity_yaw_, 0.05);
    nh_private_.param("go_to_position_tolerance", go_to_position_tolerance_, 0.4);
    nh_private_.param("go_to_orientation_tolerance", go_to_orientation_tolerance_, 0.3);
  }

  /**
   * @brief      Receives synchronized messages of nav_status and ibvs_info
   *
   * @param[in]  nav_sts    The navigation status message
   * @param[in]  ibvs_info  The ibvs information message
   */
  void MsgsCallback(const auv_msgs::NavSts::ConstPtr& nav_sts,
                    const merbots_ibvs::IBVSInfo::ConstPtr& ibvs_info) {
    if (ibvs_info->target_found == true) {
      // Target found
      last_time_seen_ = ros::Time::now().toSec();

      // If service is running, disable it
      if (srv_running_) {
        if (!DisableGoTo()) {
          ROS_ERROR_STREAM("[MerbotsIbvsRecovery]: Impossible to disable " <<
            "go to service!");
        }
      }

      // Remove old measures
      RemoveOldMeasures();

      // Add new measure
      AddNewMeasure(nav_sts, ibvs_info);
    } else {
      // Target not found

      // Do nothing if no measures have been received yet
      if (last_time_seen_ < 0) return;

      // Do nothing if the lost period is small
      if (fabs(ros::Time::now().toSec() - last_time_seen_) < min_lost_time_)
        return;

      // Do nothing if service is already running
      if (srv_running_) return;

      // Call service
      if (!EnableGoTo(nav_sts))
        ROS_ERROR_STREAM("[MerbotsIbvsRecovery]: Impossible to enable " <<
          "go to service!");
    }
  }

 protected:
  /**
   * @brief      Removes old measures.
   */
  void RemoveOldMeasures() {
    double current_stamp = ros::Time::now().toSec();
    for (std::vector< std::pair<auv_msgs::NavSts, double> >::iterator it =
      ibvs_poses_.begin(); it != ibvs_poses_.end();) {
      double it_stamp = it->first.header.stamp.toSec();
      if (fabs(it_stamp - current_stamp) > max_time_diff_) {
        it = ibvs_poses_.erase(it);
      } else {
        ++it;
      }
    }
  }

  /**
   * @brief      Adds a new measure.
   *
   * @param[in]  nav_sts    The navigation status message
   * @param[in]  ibvs_info  The ibvs information message
   */
  void AddNewMeasure(const auv_msgs::NavSts::ConstPtr& nav_sts,
                     const merbots_ibvs::IBVSInfo::ConstPtr& ibvs_info) {
    if (ibvs_poses_.size() < filter_size_) {
      // The filter is not full, so add the new measure
      ibvs_poses_.push_back(std::make_pair(*nav_sts, ibvs_info->error));
    } else {
      // The filter is full, remove the worst measure and add the new one

      // Get the maximum value of the filter
      double max_error = 0.0;
      std::vector< std::pair<auv_msgs::NavSts, double> >::iterator it_max =
        ibvs_poses_.begin();
      for (std::vector< std::pair<auv_msgs::NavSts, double> >::iterator it =
        ibvs_poses_.begin(); it != ibvs_poses_.end(); ++it) {
        if (it->second > max_error) {
          max_error = it->second;
          it_max = it;
        }
      }

      if (ibvs_info->error < max_error) {
        ibvs_poses_.erase(it_max);
        ibvs_poses_.push_back(std::make_pair(*nav_sts, ibvs_info->error));
      }
    }
  }

  /**
   * @brief      Check if service goto_holonomic exists
   *
   * @return     True if service exists, false otherwise
   */
  bool CheckForService(const std::string& service) {
    ROS_INFO_STREAM("[MerbotsIbvsRecovery]: Waiting for service: " <<
      service);
    if (ros::service::waitForService(service, ros::Duration(5))) {
      return true;
    } else {
      ROS_INFO_STREAM("[MerbotsIbvsRecovery]: Service " <<
      service << " not found. Shutting down node.");
      return false;
    }
  }

  /**
   * @brief      Enable go to service
   *
   * @param[in]  nav_sts  The navigation status
   *
   * @return     True if service was called successfully, false otherwise
   */
  bool EnableGoTo(const auv_msgs::NavSts::ConstPtr& nav_sts) {
    // Check if service exists
    if (!CheckForService(srv_name_))
      return false;

    // Remove old measures
    RemoveOldMeasures();

    // Check
    if (ibvs_poses_.size() == 0) {
      ROS_INFO_STREAM_THROTTLE(10, "[MerbotsIbvsRecovery]: Zero recent " <<
        "measures to compute a go to position. Please, increase the " <<
        "max_time_diff");
    }

    // Compute median positions
    double north = 0.0;
    double east = 0.0;
    double depth = 0.0;
    double yaw = 0.0;
    for (std::vector< std::pair<auv_msgs::NavSts, double> >::iterator it =
      ibvs_poses_.begin(); it != ibvs_poses_.end(); ++it) {
      north += it->first.position.north;
      east += it->first.position.east;
      depth += it->first.position.depth;
      double tmp_yaw = it->first.orientation.yaw;
      tmp_yaw = fmod(tmp_yaw, 2*M_PI);
      while (tmp_yaw < 0) {
        tmp_yaw += 2*M_PI;
      }
      yaw += tmp_yaw;
    }
    north = north / ibvs_poses_.size();
    east = east / ibvs_poses_.size();
    depth = depth / ibvs_poses_.size();
    yaw = yaw / ibvs_poses_.size();

    double curr_north = nav_sts->position.north;
    double curr_east = nav_sts->position.east;
    double dist = sqrt((north - curr_north)*(north - curr_north) +
                       (east - curr_east)*(east - curr_east));
    if (dist > 5.0) {
      ROS_WARN_STREAM("[MerbotsIbvsRecovery]: Distance to target too big. " <<
        "Omitting");
      return false;
    }
    ROS_INFO_STREAM("[MerbotsIbvsRecovery]: Distance to target: " << dist <<
      "m.");


    // Call service
    ros::ServiceClient client =
      nh_.serviceClient<cola2_msgs::Goto>(srv_name_);
    cola2_msgs::Goto srv;
    srv.request.reference = cola2_msgs::Goto::Request::REFERENCE_NED;
    srv.request.position.x = north;
    srv.request.position.y = east;
    srv.request.position.z = depth;
    srv.request.altitude_mode = false;  // depth mode
    srv.request.yaw = yaw;
    srv.request.linear_velocity.x = go_to_linear_velocity_x_;
    srv.request.linear_velocity.y = go_to_linear_velocity_y_;
    srv.request.linear_velocity.z = go_to_linear_velocity_z_;
    srv.request.angular_velocity.roll = 0;
    srv.request.angular_velocity.pitch = 0;
    srv.request.angular_velocity.yaw = go_go_angular_velocity_yaw_;
    srv.request.position_tolerance.x = go_to_position_tolerance_;
    srv.request.position_tolerance.y = go_to_position_tolerance_;
    srv.request.position_tolerance.z = go_to_position_tolerance_;
    srv.request.orientation_tolerance.roll = go_to_orientation_tolerance_;
    srv.request.orientation_tolerance.pitch = go_to_orientation_tolerance_;
    srv.request.orientation_tolerance.yaw = go_to_orientation_tolerance_;
    // HOLONOMIC GOTO:   False,  False,  ?,      True,   True,   ?
    srv.request.disable_axis.x = false;
    srv.request.disable_axis.y = false;
    srv.request.disable_axis.z = true;
    srv.request.disable_axis.roll = true;
    srv.request.disable_axis.pitch = true;
    srv.request.disable_axis.yaw = false;
    if (!client.call(srv)) {
      ROS_ERROR_STREAM("[MerbotsIbvsRecovery]: Failed to call service " <<
        srv_name_);
      return false;
    }
    ROS_INFO_STREAM("[MerbotsIbvsRecovery]: Go to service called!");
    srv_running_ = true;
    return true;
  }

  /**
   * @brief     Disables go to service
   *
   * @return    True if service was called successfully, false otherwise
   */
  bool DisableGoTo() {
    // Check if service exists
    if (!CheckForService(disable_srv_name_))
      return false;

    ros::ServiceClient client =
      nh_.serviceClient<std_srvs::Empty>(disable_srv_name_);
      std_srvs::Empty empty;
    if (!client.call(empty)) {
      ROS_ERROR_STREAM("[MerbotsIbvsRecovery]: Failed to call service " <<
        disable_srv_name_);
      return false;
    }
    srv_running_ = false;
    return true;
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  std::string srv_name_;
  std::string disable_srv_name_;
  double go_to_linear_velocity_x_;
  double go_to_linear_velocity_y_;
  double go_to_linear_velocity_z_;
  double go_go_angular_velocity_yaw_;
  double go_to_position_tolerance_;
  double go_to_orientation_tolerance_;
  bool srv_running_;

  int filter_size_;
  double max_time_diff_;
  double min_lost_time_;
  double last_time_seen_;

  std::vector< std::pair<auv_msgs::NavSts, double> > ibvs_poses_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "merbots_ibvs_recovery");

  // The recovery node
  MerbotsIbvsRecovery ibvs_recovery_node;

  // Read params
  ibvs_recovery_node.ReadParams();

  // Sync topics
  ros::NodeHandle nh;
  message_filters::Subscriber<auv_msgs::NavSts> nav_sts_sub(nh,
                                                            "nav_sts",
                                                            10);
  message_filters::Subscriber<merbots_ibvs::IBVSInfo> ibvs_info_sub(nh,
                                                            "ibvs_info",
                                                            10);
  message_filters::Synchronizer<SyncPool> sync(SyncPool(5),
                                               nav_sts_sub,
                                               ibvs_info_sub);
  sync.registerCallback(boost::bind(&MerbotsIbvsRecovery::MsgsCallback,
                                    &ibvs_recovery_node, _1, _2));

  // Ros spin
  ros::spin();
  return 0;
}
