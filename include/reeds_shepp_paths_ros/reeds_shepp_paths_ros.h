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

#ifndef REEDS_SHEPP_PATHS_REEDS_SHEPP_PATHS_H
#define REEDS_SHEPP_PATHS_REEDS_SHEPP_PATHS_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

namespace reeds_shepp
{

  class RSPathsROS
  {
    public:

      RSPathsROS(double minTurningRadius);

      ~RSPathsROS();

      void state2pose(
        const ompl::base::State* state, geometry_msgs::Pose& pose);

      void pose2state(
        const geometry_msgs::Pose& pose, ompl::base::State* state);

      bool planPath(
        const geometry_msgs::Pose& startPose,
        const geometry_msgs::Pose& goalPose,
        std::vector<geometry_msgs::Pose>& pathPoses,
        double bubbleRadius);

    private:
      ompl::base::StateSpacePtr reedsSheppStateSpace_;
      ompl::geometric::SimpleSetupPtr simpleSetup_;

      double minTurningRadius_;
      std::vector<geometry_msgs::Pose> path_;

  };  // class RSPathsROS
}  // namespace reeds_shepp

#endif  // REEDS_SHEPP_PATHS_REEDS_SHEPP_PATHS_H
