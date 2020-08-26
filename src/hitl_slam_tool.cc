/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

#include <iostream>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreManualObject.h>

#include <ros/console.h>

#include <geometry_msgs/Point32.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>

#include "rviz_hitl_slam_tool/HitlSlamInputMsg.h"

#include "hitl_slam_tool.h"

#include <eigen3/Eigen/Dense>

using Eigen::Vector2f;
using rviz_hitl_slam::State;
using rviz_hitl_slam_tool::HitlSlamInputMsg;

namespace rviz_hitl_slam {

HitlSlamTool::HitlSlamTool() :
    line_a_object_ (NULL),
    line_b_object_ (NULL),
    state(State::kDisabled) {
  shortcut_key_ = 's';
  ros::NodeHandle n_;
  publisher_ = n_.advertise<HitlSlamInputMsg>("hitl_slam_input", 1);
}

HitlSlamTool::~HitlSlamTool() {
}

void HitlSlamTool::onInitialize() {
  std::cout << "Initializing" << std::endl;
  if (!Ogre::ResourceGroupManager::getSingleton().resourceGroupExists("rviz")) {
    Ogre::ResourceGroupManager::getSingleton().createResourceGroup("rviz");
    Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup("rviz");
    std::cout << "Is Rviz Initialized: " << Ogre::ResourceGroupManager::getSingleton().isResourceGroupInitialised("rviz") << std::endl;
  }
  std::cout << "Running" << std::endl;
  line_a_object_ = scene_manager_->createManualObject("hitl_line_a");
  Ogre::SceneNode* line_a_node =
      scene_manager_->getRootSceneNode()->createChildSceneNode(
        "hitl_line_a_node");

  Ogre::MaterialPtr line_a_material =
      Ogre::MaterialManager::getSingleton().create(
        "hitl_line_a_material", "rviz");
  line_a_material->setReceiveShadows(false);
  line_a_material->getTechnique(0)->setLightingEnabled(true);
  line_a_material->getTechnique(0)->getPass(0)->setDiffuse(0,0,1,0);
  line_a_material->getTechnique(0)->getPass(0)->setAmbient(0,0,1);
  line_a_material->getTechnique(0)->getPass(0)->setSelfIllumination(0,0,1);
  line_a_object_->setDynamic(true);
  line_a_object_->begin("hitl_line_a_material",
                      Ogre::RenderOperation::OT_LINE_LIST);
  line_a_object_->position(3, 2, 1);
  line_a_object_->position(4, 1, 0);
  line_a_object_->end();
  line_a_object_->setVisible(false);
  line_a_node->attachObject( line_a_object_ );

  line_b_object_ = scene_manager_->createManualObject("hitl_line_b");
  Ogre::SceneNode* line_b_node =
      scene_manager_->getRootSceneNode()->createChildSceneNode(
          "hitl_line_b_node");

  Ogre::MaterialPtr line_b_material =
      Ogre::MaterialManager::getSingleton().create(
          "hitl_line_b_material", "rviz");
  line_b_material->setReceiveShadows(false);
  line_b_material->getTechnique(0)->setLightingEnabled(true);
  line_b_material->getTechnique(0)->getPass(0)->setDiffuse(1,0,0,0);
  line_b_material->getTechnique(0)->getPass(0)->setAmbient(1,0,0);
  line_b_material->getTechnique(0)->getPass(0)->setSelfIllumination(1,0,0);
  line_b_object_->setDynamic(true);
  line_b_object_->begin("hitl_line_b_material",
                        Ogre::RenderOperation::OT_LINE_LIST);
  line_b_object_->position(3, 2, 1);
  line_b_object_->position(4, 1, 0);
  line_b_object_->end();
  line_b_object_->setVisible(false);
  line_b_node->attachObject( line_b_object_ );
}

void HitlSlamTool::activate() {
  state = State::kStartLineA;
}

void HitlSlamTool::deactivate() {
  state = State::kDisabled;
}

void HitlSlamTool::Publish() {
  HitlSlamInputMsg msg;
  msg.line_a_start.x = line_a_start.x();
  msg.line_a_start.y = line_a_start.y();
  msg.line_a_start.z = 0;
  msg.line_a_end.x = line_a_end.x();
  msg.line_a_end.y = line_a_end.y();
  msg.line_a_end.z = 0;
  msg.line_b_start.x = line_b_start.x();
  msg.line_b_start.y = line_b_start.y();
  msg.line_b_start.z = 0;
  msg.line_b_end.x = line_b_end.x();
  msg.line_b_end.y = line_b_end.y();
  msg.line_b_end.z = 0;
  publisher_.publish(msg);
}

int HitlSlamTool::processMouseEvent(rviz::ViewportMouseEvent& event) {
  static const bool kDebug = false;
  Ogre::Vector3 intersection;
  Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
  if (rviz::getPointOnPlaneFromWindowXY(
        event.viewport, ground_plane, event.x, event.y, intersection)) {
    if (kDebug) {
      printf("State: %d\n", static_cast<int>(state));
    }
    switch (state) {
      case State::kDisabled: {

      } break;

      case State::kStartLineA: {
        if (event.leftDown()) {
          line_a_start.x() = intersection.x;
          line_a_start.y() = intersection.y;
          state = State::kLineA;
          line_a_object_->setVisible(true);
        }
      } break;

      case State::kLineA: {
        line_a_end.x() = intersection.x;
        line_a_end.y() = intersection.y;
        line_a_object_->beginUpdate(0);
        line_a_object_->position(line_a_start.x(), line_a_start.y(), 0);
        line_a_object_->position(line_a_end.x(), line_a_end.y(), 0);
        line_a_object_->end();
        if (event.leftUp()) {
          state = State::kStartLineB;
        }
      } break;

      case State::kStartLineB: {
        if (event.leftDown()) {
          line_b_start.x() = intersection.x;
          line_b_start.y() = intersection.y;
          state = State::kLineB;
          line_b_object_->setVisible(true);
        }
      } break;

      case State::kLineB: {
        line_b_end.x() = intersection.x;
        line_b_end.y() = intersection.y;
        line_b_object_->beginUpdate(0);
        line_b_object_->position(line_b_start.x(), line_b_start.y(), 0);
        line_b_object_->position(line_b_end.x(), line_b_end.y(), 0);
        line_b_object_->end();
        if (event.leftUp()) {
          state = State::kDisabled;
          line_a_object_->setVisible(false);
          line_b_object_->setVisible(false);
          Publish();
          return Render | Finished;
        }
      } break;

      default: {
        fprintf(stderr, "ERROR: Unknown state %d\n", static_cast<int>(state));
      }
    }

  } else {
    if (kDebug) printf("Sky\n");
  }
  return Render;
}

}  // namespace rviz_hitl_slam

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_hitl_slam::HitlSlamTool, rviz::Tool)
