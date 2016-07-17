/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, George Kouros.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author:  George Kouros
*********************************************************************/

#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include "reeds_shepp_paths_ros/reeds_shepp_paths_ros.h"
#include <math.h>

class StartGoalUpdater
{
  public:

    StartGoalUpdater(
      ros::NodeHandle* nh,
      geometry_msgs::PoseStamped* start,
      geometry_msgs::PoseStamped* goal)
      : nh_(nh), start_(start), goal_(goal),
        receivedStart_(false), receivedGoal_(false)
    {
      startSub_ = nh_->subscribe(
        "/initialpose", 1, &StartGoalUpdater::startCallback, this);
      goalSub_ = nh_->subscribe(
        "/move_base_simple/goal", 1, &StartGoalUpdater::goalCallback, this);
    }

    void startCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
    {
      if (!receivedStart_)
        receivedStart_ = true;

      start_->header = msg.header;
      start_->pose = msg.pose.pose;
    }

    void goalCallback(const geometry_msgs::PoseStamped& msg)
    {
      if (!receivedGoal_)
        receivedGoal_ = true;

      *goal_ = msg;
    }

    bool receivedStart() {return receivedStart_;}
    bool receivedGoal() {return receivedGoal_;}
    bool receivedStartAndGoal() {return (receivedStart_ && receivedGoal_);}

  private:

    ros::NodeHandle* nh_;
    ros::Subscriber startSub_;
    ros::Subscriber goalSub_;

    geometry_msgs::PoseStamped* start_;
    geometry_msgs::PoseStamped* goal_;

    bool receivedStart_;
    bool receivedGoal_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "reeds_shepp_paths_ros_demo");
  ros::NodeHandle nh("~");

  // create path publisher
  ros::Publisher pathPub = nh.advertise<nav_msgs::Path>("path", 1);
  ros::Publisher startPosePub = nh.advertise<geometry_msgs::PoseStamped>("start_pose", 1);
  ros::Publisher goalPosePub = nh.advertise<geometry_msgs::PoseStamped>("goal_pose", 1);

  geometry_msgs::PoseStamped start, goal;
  start.header.frame_id = goal.header.frame_id = "map";

  StartGoalUpdater sgu(&nh, &start, &goal);

  // initialize ReedsSheppPathsROS
  reeds_shepp::RSPathsROS RSPlanner("demo", NULL, NULL);

  while (ros::ok())
  {
    if (sgu.receivedStart())
      startPosePub.publish(start);
    if (sgu.receivedGoal())
      goalPosePub.publish(goal);

    if (sgu.receivedStartAndGoal())
    {
      // plan path from start to goal pose
      std::vector<geometry_msgs::PoseStamped> pathPoses;
      RSPlanner.planPath(start, goal, pathPoses);

      std_msgs::Header header =
        (start.header.stamp > goal.header.stamp) ? start.header : goal.header;

      // create path msg from path states
      nav_msgs::Path path;
      path.header = header;
      path.poses = pathPoses;

      // publish path
      pathPub.publish(path);
    }
    else
    {
      ROS_INFO("Waiting for start and goal poses...");
    }


    ros::spinOnce();

    ros::Duration(1.0).sleep();
  }

  return 0;
}
