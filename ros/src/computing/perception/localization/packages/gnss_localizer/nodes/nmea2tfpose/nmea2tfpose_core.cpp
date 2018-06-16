/*
 *  Copyright (c) 2015, Nagoya University

 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "nmea2tfpose_core.h"

namespace gnss_localizer
{
// Constructor
Nmea2TFPoseNode::Nmea2TFPoseNode()
  : private_nh_("~")
  , MAP_FRAME_("map")
  , GPS_FRAME_("gps")
  , roll_(0)
  , pitch_(0)
  , yaw_(0)
  , orientation_time_(0)
  , position_time_(0)
  , current_time_(0)
  , orientation_stamp_(0)
{
  initForROS();
  geo_.set_plane(plane_number_);
}

// Destructor
Nmea2TFPoseNode::~Nmea2TFPoseNode()
{
}

void Nmea2TFPoseNode::initForROS()
{
  // ros parameter settings
  private_nh_.getParam("plane", plane_number_);

  // setup subscriber
  sub1_ = nh_.subscribe("nmea_sentence", 100, &Nmea2TFPoseNode::callbackFromNmeaSentence, this);

  // setup publisher
  pub1_ = nh_.advertise<geometry_msgs::PoseStamped>("gnss_pose", 10);
  pub2_ = nh_.advertise<geometry_msgs::TwistStamped>("gnss_twist", 10);
}

void Nmea2TFPoseNode::run()
{
  ros::spin();
}

void Nmea2TFPoseNode::publishPoseStamped()
{
  geometry_msgs::PoseStamped pose;

  /*
  prev_pose_ = current_pose_;
  prev_pose_.header.frame_id = current_pose_.header.frame_id;
  prev_pose_.header.stamp = current_pose_.header.stamp;

  prev_pose_.pose.position.x = current_pose_.pose.position.x;
  prev_pose_.pose.position.y = current_pose_.pose.position.y;
  prev_pose_.pose.position.z = current_pose_.pose.position.z;
  prev_pose_.pose.orientation.z = current_pose_.pose.orientation.z;
  */

//  prev_time_ = current_time_;

  prev_pose_.x = current_pose_.x;
  prev_pose_.y = current_pose_.y;
  prev_pose_.z = current_pose_.z;
  prev_pose_.yaw = current_pose_.yaw;

  //***
  pose.header.frame_id = MAP_FRAME_;
  pose.header.stamp = current_time_;

  pose.pose.position.x = geo_.y();
  pose.pose.position.y = geo_.x();
  pose.pose.position.z = geo_.z();
  pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_, pitch_, yaw_);
  //***

  /*
  current_pose_ = pose;
  current_pose_.header.frame_id = pose.header.frame_id;
  current_pose_.header.stamp = pose.header.stamp;

  current_pose_.pose.position.x = pose.pose.position.x;
  current_pose_.pose.position.y = pose.pose.position.y;
  current_pose_.pose.position.z = pose.pose.position.z;
  current_pose_.pose.orientation.z = pose.pose.orientation.z;
  */
  current_time_ = pose.header.stamp;
  current_pose_.x = pose.pose.position.x;
  current_pose_.y = pose.pose.position.y;
  current_pose_.z = pose.pose.position.z;
  current_pose_.yaw = yaw_;

//  ROS_WARN("prev_time: %f, current_time: %f", prev_pose_.header.stamp.toSec(), current_pose_.header.stamp.toSec());
  pub1_.publish(pose);
}

void Nmea2TFPoseNode::publishGnssTwistStamped()
{
  geometry_msgs::TwistStamped twist;
//  ros::Duration duration = (current_pose_.header.stamp - prev_pose_.header.stamp);

//  ros::Duration duration = current_time_ - prev_time_;
//  double t_secs = duration.toSec();

  double t_secs = current_time_.toSec() - prev_time_.toSec();

//  ROS_WARN("nmea2tfpose_core.cpp:99 : current_pose_.header.stamp = %f", current_pose_.header.stamp);
//  ROS_WARN("nmea2tfpose_core.cpp:100 : prev_pose_.header.stamp = %f", prev_pose_.header.stamp);

  double diff_x = (current_pose_.x - prev_pose_.x) / 10000;
  double diff_y = (current_pose_.y - prev_pose_.y) / 10000;
  double diff_z = (current_pose_.z - prev_pose_.z) / 10000;
  double diff_yaw = (current_pose_.yaw - prev_pose_.yaw) / 1000;
  double diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

  ROS_WARN("nmea2tfpose_core.cpp:98 : t_secs = %f, diff_x = %f, diff_y = %f, diff_z = %f", t_secs, diff_x, diff_y, diff_z);

  double current_velocity = diff / t_secs;
  double angular_velocity = diff_yaw / t_secs;

  twist.header.frame_id = "/base_link";
  twist.twist.linear.x = current_velocity;
  twist.twist.linear.y = 0;
  twist.twist.linear.z = 0;
  twist.twist.angular.z = angular_velocity;

  pub2_.publish(twist);


  /*
  diff_x = current_pose.x - previous_pose.x;
  diff_y = current_pose.y - previous_pose.y;
  diff_z = current_pose.z - previous_pose.z;
  diff_yaw = current_pose.yaw - previous_pose.yaw;
  diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

  current_velocity = diff / secs;
  current_velocity_x = diff_x / secs;
  current_velocity_y = diff_y / secs;
  current_velocity_z = diff_z / secs;
  angular_velocity = diff_yaw / secs;
  */

  /*
  estimate_twist_msg.header.frame_id = "/base_link";
  estimate_twist_msg.twist.linear.x = current_velocity;
  estimate_twist_msg.twist.linear.y = 0.0;
  estimate_twist_msg.twist.linear.z = 0.0;
  estimate_twist_msg.twist.angular.x = 0.0;
  estimate_twist_msg.twist.angular.y = 0.0;
  estimate_twist_msg.twist.angular.z = angular_velocity;
  */
//  pub2_.publish(twist);
}
/*
static void gnss_callback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
  tf::Quaternion gnss_q(input->pose.orientation.x, input->pose.orientation.y, input->pose.orientation.z,
                        input->pose.orientation.w);
  tf::Matrix3x3 gnss_m(gnss_q);
  current_gnss_pose.x = input->pose.position.x;
  current_gnss_pose.y = input->pose.position.y;
  current_gnss_pose.z = input->pose.position.z;
  gnss_m.getRPY(current_gnss_pose.roll, current_gnss_pose.pitch, current_gnss_pose.yaw);

  if ((_use_gnss == 1 && init_pos_set == 0) || fitness_score >= 500.0)
  {
    previous_pose.x = previous_gnss_pose.x;
    previous_pose.y = previous_gnss_pose.y;
    previous_pose.z = previous_gnss_pose.z;
    previous_pose.roll = previous_gnss_pose.roll;
    previous_pose.pitch = previous_gnss_pose.pitch;
    previous_pose.yaw = previous_gnss_pose.yaw;

    current_pose.x = current_gnss_pose.x;
    current_pose.y = current_gnss_pose.y;
    current_pose.z = current_gnss_pose.z;
    current_pose.roll = current_gnss_pose.roll;
    current_pose.pitch = current_gnss_pose.pitch;
    current_pose.yaw = current_gnss_pose.yaw;

    current_pose_imu = current_pose_odom = current_pose_imu_odom = current_pose;

    offset_x = current_pose.x - previous_pose.x;
    offset_y = current_pose.y - previous_pose.y;
    offset_z = current_pose.z - previous_pose.z;
    offset_yaw = current_pose.yaw - previous_pose.yaw;

    init_pos_set = 1;
  }

  previous_gnss_pose.x = current_gnss_pose.x;
  previous_gnss_pose.y = current_gnss_pose.y;
  previous_gnss_pose.z = current_gnss_pose.z;
  previous_gnss_pose.roll = current_gnss_pose.roll;
  previous_gnss_pose.pitch = current_gnss_pose.pitch;
  previous_gnss_pose.yaw = current_gnss_pose.yaw;
}
*/

void Nmea2TFPoseNode::publishTF()
{
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(geo_.y(), geo_.x(), geo_.z()));
  tf::Quaternion quaternion;
  quaternion.setRPY(roll_, pitch_, yaw_);
  transform.setRotation(quaternion);
  br_.sendTransform(tf::StampedTransform(transform, current_time_, MAP_FRAME_, GPS_FRAME_));
}

void Nmea2TFPoseNode::createOrientation()
{
  yaw_ = atan2(geo_.x() - last_geo_.x(), geo_.y() - last_geo_.y());
  roll_ = 0;
  pitch_ = 0;
}

void Nmea2TFPoseNode::convert(std::vector<std::string> nmea, ros::Time current_stamp)
{
  try
  {
    if (nmea.at(0).compare(0, 2, "QQ") == 0)
    {
      orientation_time_ = stod(nmea.at(3));
      roll_ = stod(nmea.at(4)) * M_PI / 180.;
      pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
      yaw_ = -1 * stod(nmea.at(6)) * M_PI / 180. + M_PI / 2;
      orientation_stamp_ = current_stamp;
      ROS_INFO("QQ is subscribed.");
    }
    else if (nmea.at(0) == "$PASHR")
    {
      orientation_time_ = stod(nmea.at(1));
      roll_ = stod(nmea.at(4)) * M_PI / 180.;
      pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
      yaw_ = -1 * stod(nmea.at(2)) * M_PI / 180. + M_PI / 2;
      ROS_INFO("PASHR is subscribed.");
    }
    else if(nmea.at(0).compare(3, 3, "GGA") == 0)
    {
      position_time_ = stod(nmea.at(1));
      double lat = stod(nmea.at(2));
      if (nmea.at(3)=="S") {
          lat = -lat;
      }

      double lon = stod(nmea.at(4));
      if (nmea.at(5)=="W") {
          lon = -lon;
      }
      double h = stod(nmea.at(9));
      geo_.set_llh_nmea_degrees(lat, lon, h);
      ROS_INFO("GGA is subscribed.");
    }
    else if(nmea.at(0) == "$GPRMC")
    {
      position_time_ = stoi(nmea.at(1));
      double lat = stod(nmea.at(3));
      double lon = stod(nmea.at(5));
      double h = 0.0;
      geo_.set_llh_nmea_degrees(lat, lon, h);
      ROS_INFO("GPRMC is subscribed.");
    }
  }catch (const std::exception &e)
  {
    ROS_WARN_STREAM("Message is invalid : " << e.what());
  }
}

void Nmea2TFPoseNode::callbackFromNmeaSentence(const nmea_msgs::Sentence::ConstPtr &msg)
{
  prev_time_ = current_time_;
  current_time_ = msg->header.stamp;
  convert(split(msg->sentence), msg->header.stamp);

  double timeout = 10.0;
  if (fabs(orientation_stamp_.toSec() - msg->header.stamp.toSec()) > timeout)
  {
    double dt = sqrt(pow(geo_.x() - last_geo_.x(), 2) + pow(geo_.y() - last_geo_.y(), 2));
    double threshold = 0.2;
    if (dt > threshold)
    {
      ROS_INFO("QQ is not subscribed. Orientation is created by atan2");
      createOrientation();
      publishPoseStamped();
      publishGnssTwistStamped();
      publishTF();
      last_geo_ = geo_;
    }
    return;
  }

  double e = 1e-2;
  if (fabs(orientation_time_ - position_time_) < e)
  {
    publishPoseStamped();
    publishGnssTwistStamped();
    publishTF();
    return;
  }
}

std::vector<std::string> split(const std::string &string)
{
  std::vector<std::string> str_vec_ptr;
  std::string token;
  std::stringstream ss(string);

  while (getline(ss, token, ','))
    str_vec_ptr.push_back(token);

  return str_vec_ptr;
}

}  // gnss_localizer
