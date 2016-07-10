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

#include "reeds_shepp_paths_ros/reeds_shepp_paths_ros.h"
#include <tf/tf.h>

namespace reeds_shepp
{
  RSPathsROS::RSPathsROS(double minTurningRadius)
    : minTurningRadius_(minTurningRadius),
    reedsSheppStateSpace_(new ompl::base::ReedsSheppStateSpace),
    simpleSetup_(new ompl::geometric::SimpleSetup(reedsSheppStateSpace_))
  {
  }


  RSPathsROS::~RSPathsROS()
  {
  }


  // void RSPathsROS::pose2state(
    // const geometry_msgs::Pose& pose,
    // ompl::base::ReedsSheppStateSpace::StateType& state)
  // {
    // state.setXY(pose.position.x, pose.position.y);
    // state.setYaw(tf::getYaw(pose.orientation));
  // }


  // ompl::base::ReedsSheppStateSpace::StateType
    // RSPathsROS::pose2state(const geometry_msgs::Pose& pose)
  // {
    // ompl::base::ReedsSheppStateSpace::StateType* state
    // state.setXY(pose.position.x, pose.position.y);
    // state.setYaw(tf::getYaw(pose.orientation))
    // return state;
  // }


  // void RSPathsROS::state2pose(
    // const ompl::base::ReedsSheppStateSpace::StateType& state,
    // geometry_msgs::Pose& pose)
  // {
    // pose.position.x = state.getX();
    // pose.position.y = state.getY();
    // pose.orientation = tf::createQuaternionMsgFromYaw(state.getYaw());
  // }


  // geometry_msgs::Pose RSPathsROS::state2pose(
    // const ompl::base::ReedsSheppStateSpace::StateType& state)
  // {
    // geometry_msgs::Pose pose;
    // pose.position.x = state.getX();
    // pose.position.y = state.getY();
    // pose.orientation = tf::createQuaternionMsgFromYaw(state.getYaw());
    // return pose;
  // }


  bool RSPathsROS::planPath(
    const geometry_msgs::Pose& startPose,
    const geometry_msgs::Pose& endPose,
    std::vector<geometry_msgs::Pose>& pathPoses)
  {
    ompl::base::ScopedState<> start(reedsSheppStateSpace_), goal(reedsSheppStateSpace_);
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(18);
    reedsSheppStateSpace_->as<ompl::base::SE2StateSpace>()->setBounds(bounds);

    // define a simple setup class
    ompl::geometric::SimpleSetup simpleSetup_(reedsSheppStateSpace_);

    // set state validity checking for this space
    ompl::base::SpaceInformationPtr
      spaceInformation(simpleSetup_.getSpaceInformation());
    reedsSheppStateSpace_.setStateValidityChecker(
      std::bind(
        true ? &isStateValidEasy : &isStateValidHard, si.get(),
        std::placeholders::_1));

    start[0] = startPose.position.x;
    start[1] = startPose.position.y;
    start[2] = tf::getYaw(startPose.orientation);

    goal[0] = endPose.position.x;
    goal[1] = endPose.position.y;
    goal[2] = tf::getYaw(endPose.orientation);

    simpleSetup_.setStartAndGoalStates(start, goal);

    // print space and planning info
    // simpleSetup_.getSpaceInformation()->setStateValidityCheckingResolution(0.1);
    // simpleSetup_.setup();
    // simpleSetup_.print();

    // attempt to solve the problem within 30 seconds of planning time
    ompl::base::PlannerStatus
      solved = simpleSetup_.solve(30.0);

    if (!solved)
    {
      ROS_ERROR("No solution found");
      return false;
    }

    ROS_INFO("Found solution");

    simpleSetup_.simplifySolution();
    ompl::geometric::PathGeometric path = simpleSetup_.getSolutionPath();
    path.interpolate(50);
    path.printAsMatrix(std::cout);

    return true;
  }

}  // namespace reeds_shepp
