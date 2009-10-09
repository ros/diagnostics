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

// Author: Blaise Gassend
#ifndef __DIAGNOSTIC_UPDATER__DRIVER_H__
#define __DIAGNOSTIC_UPDATER__DRIVER_H__

#include <ros/publisher.h>
#include <ros/subscription.h>
#include <diagnostic_updater/update_functions.h>

namespace diagnostic_updater
{
                                   
class HeaderlessTopicDiagnostic : public CombinationDiagnosticTask
{
public:
  HeaderlessTopicDiagnostic(
      std::string name,
      diagnostic_updater::Updater &diag,
      const diagnostic_updater::FrequencyStatusParam &freq) :
    CombinationDiagnosticTask(name + " topic status"), 
    freq_(freq)
  {
    addTask(&freq_);
    diag.add(*this);
  }

  virtual ~HeaderlessTopicDiagnostic()
  {}
  
  virtual void tick()
  {
    freq_.tick();
  }
  
  virtual void clear_window()
  {
    freq_.clear();
  }

private:
  diagnostic_updater::FrequencyStatus freq_;
};

class HeaderlessDiagnosedPublisher : public HeaderlessTopicDiagnostic
{
public:
  HeaderlessDiagnosedPublisher(const ros::Publisher &pub,
      diagnostic_updater::Updater &diag,
      const diagnostic_updater::FrequencyStatusParam &freq) :
    HeaderlessTopicDiagnostic(pub.getTopic(), diag, freq),
    publisher_(pub)
  {}

  virtual ~HeaderlessDiagnosedPublisher()
  {}
  
  virtual void publish(const ros::MessageConstPtr& message)
  {
    tick();
    publisher_.publish(message);
  }
 
  virtual void publish(const ros::Message& message)
  {
    tick();
    publisher_.publish(message);
  }

  ros::Publisher publisher() const
  {
    return publisher_;
  }

  void set_publisher(ros::Publisher pub)
  {
    publisher_ = pub;
  }

private:
  ros::Publisher publisher_;
};

class TopicDiagnostic : public HeaderlessTopicDiagnostic
{
public:
  TopicDiagnostic(
      std::string name,
      diagnostic_updater::Updater &diag,
      const diagnostic_updater::FrequencyStatusParam &freq,
      const diagnostic_updater::TimeStampStatusParam &stamp) : 
    HeaderlessTopicDiagnostic(name, diag, freq), 
    stamp_(stamp)
  {
    addTask(&stamp_);
  }
  
  virtual ~TopicDiagnostic()
  {}
  
  virtual void tick()
  {
    ROS_FATAL("tick(void) has been called on a TopicDiagnostic. This is never correct. Use tick(ros::Time &) instead.");
  }

  virtual void tick(const ros::Time &stamp)
  {
    stamp_.tick(stamp);
    HeaderlessTopicDiagnostic::tick();
  }
  
private:
  TimeStampStatus stamp_;
};

template<class T>
class DiagnosedPublisher : public TopicDiagnostic
{
public:
  DiagnosedPublisher(const ros::Publisher &pub,
      diagnostic_updater::Updater &diag, 
      const diagnostic_updater::FrequencyStatusParam &freq, 
      const diagnostic_updater::TimeStampStatusParam &stamp) : 
    TopicDiagnostic(pub.getTopic(), diag, freq, stamp),
    publisher_(pub)
  {}

  virtual ~DiagnosedPublisher()
  {}
  
  virtual void publish(const boost::shared_ptr<T>& message)
  {
    tick(message->header.stamp);
    publisher_.publish(message);
  }
 
  virtual void publish(const T& message)
  {
    tick(message.header.stamp);
    publisher_.publish(message);
  }

  ros::Publisher publisher() const
  {
    return publisher_;
  }

  void set_publisher(ros::Publisher pub)
  {
    publisher_ = pub;
  }

private:
  ros::Publisher publisher_;
};


};

#endif
