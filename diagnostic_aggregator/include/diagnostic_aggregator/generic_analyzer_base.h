/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 *********************************************************************/

/**!< \author Kevin Watts */

#ifndef GENERIC_ANALYZER_BASE_H
#define GENERIC_ANALYZER_BASE_H

#include <map>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include <boost/regex.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/KeyValue.h"
#include "diagnostic_aggregator/analyzer.h"
#include "diagnostic_aggregator/status_item.h"

namespace diagnostic_aggregator {

/*!
 *\brief GenericAnalyzerBase is the base class for GenericAnalyzer and OtherAnalyzer
 *
 * GenericAnalyzerBase contains the getPath(), getName(), analyze() and report() functions of
 * the Generic and Other Analyzers. It is a virtual class, and cannot be instantiated or loaded 
 * as a plugin. Subclasses are responsible for implementing the init() and match() functions.
 *
 * The GenericAnalyzerBase holds the state of the analyzer, and tracks if items are stale, and
 * if the user has the correct number of items.
 */
class GenericAnalyzerBase : public Analyzer
{
public:
  GenericAnalyzerBase() : 
    nice_name_(""), path_(""), timeout_(-1.0), num_items_expected_(-1),
    discard_stale_(false), has_initialized_(false), has_warned_(false) 
  { }
  
  virtual ~GenericAnalyzerBase() { items_.clear(); }
  
  /*
   *\brief Cannot be initialized from (string, NodeHandle) like defined Analyzers
   */
  bool init(const std::string path, const ros::NodeHandle &n) = 0;

  /*
   *\brief Must be initialized with path, and a "nice name"
   *
   * Must be initialized in order to prepend the path to all outgoing status messages.
   */
  bool init(const std::string path, const std::string nice_name, 
            double timeout = -1.0, int num_items_expected = -1, bool discard_stale = false)
  {
    num_items_expected_ = num_items_expected;
    timeout_ = timeout;
    nice_name_ = nice_name;
    path_ = path;
    discard_stale_ = discard_stale;

    if (discard_stale_ and timeout <= 0)
    {
      ROS_WARN("Cannot discard stale items if no timeout specified. No items will be discarded");
      discard_stale_ = false;
    }

    has_initialized_ = true;
    
    return true;
  }

  /*!
   *\brief Update state with new StatusItem
   */
  virtual bool analyze(const boost::shared_ptr<StatusItem> item)
  {
    if (!has_initialized_ && !has_warned_)
    {
      has_warned_ = true;
      ROS_ERROR("GenericAnalyzerBase is asked to analyze diagnostics without being initialized. init() must be called in order to correctly use this class.");
    }

    if (!has_initialized_)
      return false;

    items_[item->getName()] = item;

    return has_initialized_;
  }
  
  /*!
   *\brief Reports current state, returns vector of formatted status messages
   *
   *\return Vector of DiagnosticStatus messages. They must have the correct prefix for all names.
   */
  virtual std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > report()
  {
    if (!has_initialized_ && !has_warned_)
    {
      has_warned_ = true;
      ROS_ERROR("GenericAnalyzerBase is asked to report diagnostics without being initialized. init() must be called in order to correctly use this class.");
    }
    if (!has_initialized_)
    {
      std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > vec;
      return vec;
    }

    boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> header_status(new diagnostic_msgs::DiagnosticStatus());
    header_status->name = path_;
    header_status->level = 0;
    header_status->message = "OK";
    
    std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > processed;
    processed.push_back(header_status);
    
    bool all_stale = true;

    std::map<std::string, boost::shared_ptr<StatusItem> >::iterator it = items_.begin();
    while(it != items_.end())
    {
      std::string name = it->first;
      boost::shared_ptr<StatusItem> item = it->second;

      bool stale = false;
      if (timeout_ > 0)
        stale = (ros::Time::now() - item->getLastUpdateTime()).toSec() > timeout_;

      // Erase item if its stale and we're discarding items
      if (discard_stale_ and stale)
      {
        items_.erase(it++);
        continue;
      }
      
      int8_t level = item->getLevel();
      header_status->level = std::max(header_status->level, level);
      
      diagnostic_msgs::KeyValue kv;
      kv.key = name;
      kv.value = item->getMessage();
      
      header_status->values.push_back(kv);
      
      all_stale = all_stale && ((level == 3) || stale);
      
      //boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> stat = item->toStatusMsg(path_, stale);
      
      processed.push_back(item->toStatusMsg(path_, stale));

      if (stale)
        header_status->level = 3;

      ++it;
    }
    
    // Header is not stale unless all subs are
    if (all_stale)
      header_status->level = 3;
    else if (header_status->level == 3)
      header_status->level = 2;
    
    header_status->message = valToMsg(header_status->level);
    
    // If we expect a given number of items, check that we have this number
    if (num_items_expected_ == 0 && items_.size() == 0)
    {
      header_status->level = 0;
      header_status->message = "OK";
    }
    else if (num_items_expected_ > 0 and int(items_.size()) != num_items_expected_)
    {
      int8_t lvl = 2;
      header_status->level = std::max(lvl, header_status->level);

      std::stringstream expec, item;
      expec << num_items_expected_;
      item << items_.size();

      if (items_.size() > 0)
        header_status->message = "Expected " + expec.str() + ", found " + item.str();
      else
        header_status->message = "No items found, expected " + expec.str();
    }
    
    return processed;
  }

  /*!
   *\brief Match function isn't implemented by GenericAnalyzerBase
   */
  virtual bool match(const std::string name) = 0;
  
  /*!
   *\brief Returns full prefix (ex: "/Robot/Power System")
   */
  virtual std::string getPath() const { return path_; }

  /*!
   *\brief Returns nice name (ex: "Power System")
   */
  virtual std::string getName() const { return nice_name_; }

protected:
  std::string nice_name_;
  std::string path_;

  double timeout_;
  int num_items_expected_;

  /*!
   *\brief Subclasses can add items to analyze 
   */
  void addItem(std::string name, boost::shared_ptr<StatusItem> item)  { items_[name] = item; }

private:
  /*!
   *\brief Stores items by name. State of analyzer
   */
  std::map<std::string, boost::shared_ptr<StatusItem> > items_;

  bool discard_stale_, has_initialized_, has_warned_;
};

}
#endif //GENERIC_ANALYZER_BASE_H
