/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2019, Joydeep Biswas joydeepb@cs.umass.edu
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef HITL_SLAM_TOOL_H
#define HITL_SLAM_TOOL_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/tool.h>
#endif

namespace rviz {
  class ViewportMouseEvent;
}  // namespace rviz

namespace Ogre {
  class ManualObject;
}  // namespace Ogre

namespace rviz_hitl_slam {

class HitlSlamTool : public rviz::Tool {
Q_OBJECT
public:
  ros::NodeHandle n_;
  ros::Publisher mouse_publisher_;

  HitlSlamTool();
  ~HitlSlamTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

  Ogre::ManualObject* myManualObject;
  float start_x;
  float start_y;
  bool selection_active;

};

}  // namespace rviz_hitl_slam

#endif // HITL_SLAM_TOOL_H