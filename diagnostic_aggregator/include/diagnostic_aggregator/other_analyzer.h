// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef OTHER_ANALYZER_H
#define OTHER_ANALYZER_H

#include <string>
/*#include <ros/ros.h>*/
#include "rclcpp/rclcpp.hpp"
#include "diagnostic_aggregator/generic_analyzer_base.h"

//TODO(tfoote replace these terrible macros)
#define ROS_ERROR printf
#define ROS_FATAL printf
#define ROS_WARN printf
#define ROS_INFO printf

namespace diagnostic_aggregator {

/*
 *\brief OtherAnalyzer analyzes any messages that haven't been analyzed by other Analyzers
 *
 * OtherAnalyzer is not loaded as a plugin. It is created by the Aggregator, and called 
 * seperately. The aggregator will call analyze() on any message not handled by other
 * analyzers.
 *
 * Stale items will be discarded after 5 seconds of no updates.
 * 
 * OtherAnalyzer is designed to be used internally by the Aggregator only.
 *
 */
class OtherAnalyzer : public GenericAnalyzerBase
{
public:
  /*!
   *\brief Default constructor. OtherAnalyzer isn't loaded by pluginlib
   */
 /* explicit OtherAnalyzer(bool other_as_errors = false)
  : other_as_errors_(other_as_errors)
  { }*/
   OtherAnalyzer() { }

  ~OtherAnalyzer() { }

  /*
   *\brief Initialized with the base path only.
   *
   *\param path Base path of Aggregator
   */
  bool init(std::string path)
  {
    return GenericAnalyzerBase::init_v(path + "/Other", "Other", 5.0, -1, true);
  }

  /*
   *\brief OtherAnalyzer cannot be initialized with a NodeHandle
   *
   *\return False, since NodeHandle initialization isn't valid
   */
  //bool init(const std::string base_path, const rclcpp::Node::SharedPtr &n)
  bool init(const std::string base_path, const char *nsp,const rclcpp::Node::SharedPtr &n,const char *)
  {
    ROS_ERROR("OtherAnalyzer was attempted to initialize with a NodeHandle. This analyzer cannot be used as a plugin.");
    return false;
  }

  /*
   *\brief match() isn't called by aggregator for OtherAnalyzer
   *
   *\return True, since match() will never by called by Aggregator
   */
  bool match(std::string name) { return true; }

  /*
   *\brief Reports diagnostics, but doesn't report anything if it doesn't have data
   *
   */
  std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus> > report()
  {
    std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus> > processed = GenericAnalyzerBase::report();

    // We don't report anything if there's no "Other" items
    if (processed.size() == 1)
    {
      processed.clear();
    }
    // "Other" items are considered an error.
    else if (other_as_errors_ && processed.size() > 1)
    {
      std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus> >::iterator it = processed.begin();
      for (; it != processed.end(); ++it)
      {
        if ((*it)->name == path_)
        {
          (*it)->level = 2;
          (*it)->message = "Unanalyzed items found in \"Other\"";
          break;
        }
      }
    }
    
    return processed;
  }

private:
  bool other_as_errors_;
};

}


#endif // OTHER_ANALYZER_H
