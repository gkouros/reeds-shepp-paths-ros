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
#include <boost/foreach.hpp>

namespace reeds_shepp
{
  RSPathsROS::RSPathsROS(double minTurningRadius, double maxPlanningDuration)
    : minTurningRadius_(minTurningRadius), maxPlanningDuration_(maxPlanningDuration),
    reedsSheppStateSpace_(new ompl::base::ReedsSheppStateSpace),
    simpleSetup_(new ompl::geometric::SimpleSetup(reedsSheppStateSpace_))
  {
  }


  RSPathsROS::~RSPathsROS()
  {
  }


  void RSPathsROS::state2pose(
    const ompl::base::State* state, geometry_msgs::Pose& pose)
  {
    const ompl::base::SE2StateSpace::StateType *s =
      state->as<ompl::base::SE2StateSpace::StateType>();
    pose.position.x = s->getX();
    pose.position.y = s->getY();
    pose.orientation = tf::createQuaternionMsgFromYaw(s->getYaw());
  }

  void RSPathsROS::pose2state(
    const geometry_msgs::Pose& pose, ompl::base::State* state)
  {
    ompl::base::SE2StateSpace::StateType *s =
      state->as<ompl::base::SE2StateSpace::StateType>();
    s->setX(pose.position.x);
    s->setY(pose.position.y);
    s->setYaw(tf::getYaw(pose.orientation));
  }

  bool RSPathsROS::planPath(
    const geometry_msgs::Pose& startPose,
    const geometry_msgs::Pose& goalPose,
    std::vector<geometry_msgs::Pose>& pathPoses,
    double bubbleRadius)
  {
    ompl::base::ScopedState<> start(reedsSheppStateSpace_), goal(reedsSheppStateSpace_);
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-bubbleRadius);
    bounds.setHigh(bubbleRadius);
    reedsSheppStateSpace_->as<ompl::base::SE2StateSpace>()->setBounds(bounds);

    // TODO set state validity checking
    // ompl::base::SpaceInformationPtr
      // spaceInformation(simpleSetup_->.getSpaceInformation());
    // reedsSheppStateSpace_.setStateValidityChecker(
      // std::bind(TODO customStateValidityChecker, spaceInformation.get(),
        // std::placeholders::_1));

    pose2state(startPose, start());
    pose2state(goalPose, goal());

    // clear all planning data
    simpleSetup_->clear();
    // set new start and goal states
    simpleSetup_->setStartAndGoalStates(start, goal);

    // print space and planning info
    // simpleSetup_->getSpaceInformation()->setStateValidityCheckingResolution(0.1);
    // simpleSetup_->setup();  // called by solve automatically
    // simpleSetup_->print();

    if (!simpleSetup_->solve(maxPlanningDuration_))
    {
      ROS_ERROR("No solution found");
      return false;
    }
    else
      ROS_INFO("Found solution");

    // simplify solution
    simpleSetup_->simplifySolution();
    // get solution path
    ompl::geometric::PathGeometric path = simpleSetup_->getSolutionPath();
    // interpolate between poses using 50 intermediate poses
    path.interpolate(50);

    // clear pathPoses vector in case it's not empty
    pathPoses.clear();

    // convert each state to a pose and store it in pathPoses vector
    for (unsigned int i = 0; i < path.getStateCount(); i++)
    {
      const ompl::base::State* pathState = path.getState(i);
      geometry_msgs::Pose pathPose;
      state2pose(pathState, pathPose);
      pathPoses.push_back(pathPose);
    }

    // path.printAsMatrix(std::cout);  // prints poses in matrix format

    return true;
  }

}  // namespace reeds_shepp
