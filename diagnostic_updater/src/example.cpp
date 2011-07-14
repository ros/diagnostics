/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>

double time_to_launch;

/*
 *\brief Used as a tutorial for loading and using diagnostic updater
 *
 * DummyClass and dummy_diagnostics show how to use a diagnostic_updater
 * class.
 */

void dummy_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  // DiagnosticStatusWrapper are a derived class of 
  // diagnostic_msgs::DiagnosticStatus provides a set of convenience
  // methods.
  
  // summary and summaryf set the level and message.
  if (time_to_launch < 10)
    // summaryf for formatted text.
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Buckle your seat belt. Launch in %f seconds!", time_to_launch);
  else
    // summary for unformatted text.
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Launch is in a long time. Have a soda.");

  // add and addf are used to append key-value pairs.
  stat.add("Diagnostic Name", "dummy");
  // add transparently handles conversion to string (using a string_stream).
  stat.add("Time to Launch", time_to_launch);
  // addf allows arbitrary printf style formatting.
  stat.addf("Geeky thing to say", "The square of the time to launch %f is %f", 
      time_to_launch, time_to_launch * time_to_launch);
}

class DummyClass
{
public:
  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "This is a silly updater.");

    stat.add("Stupidicity of this updater", 1000.);
  }
};

class DummyTask : public diagnostic_updater::DiagnosticTask
{
public:
  DummyTask() : DiagnosticTask("Updater Derived from DiagnosticTask")
  {}

  void run(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "This is another silly updater.");
    stat.add("Stupidicity of this updater", 2000.);
  }
};

void check_lower_bound(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (time_to_launch > 5)
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Lower-bound OK");
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Too low");

  stat.add("Low-Side Margin", time_to_launch - 5);
}

void check_upper_bound(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (time_to_launch < 10)
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Upper-bound OK");
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Too high");

  stat.add("Top-Side Margin", 10 - time_to_launch);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "diagnostic_updater_example");
  
  ros::NodeHandle nh;
  
  // The Updater class advertises to /diagnostics, and has a
  // ~diagnostic_period parameter that says how often the diagnostics
  // should be published.
  diagnostic_updater::Updater updater;

  // The diagnostic_updater::Updater class will fill out the hardware_id
  // field of the diagnostic_msgs::DiagnosticStatus message. You need to
  // use the setHardwareID() or setHardwareIDf() methods to set the
  // hardware ID. 
  //
  // The hardware ID should be able to identify the specific device you are
  // working with.  If it is not appropriate to fill out a hardware ID in
  // your case, you should call setHardwareIDf("none") to avoid warnings.
  // (A warning will be generated as soon as your node updates with no
  // non-OK statuses.)
  updater.setHardwareID("none"); 
  // Or...
  updater.setHardwareIDf("Device-%i-%i", 27, 46);

  // Diagnostic tasks are added to the Updater. They will later be run when
  // the updater decides to update. The add method is heavily overloaded
  // for convenienc. Check doxygen for the full list of add methods.
  updater.add("Function updater", dummy_diagnostic);
  DummyClass dc;
  updater.add("Method updater", &dc, &DummyClass::produce_diagnostics);
  
  // Internally, updater.add converts its arguments into a DiagnosticTask.
  // Sometimes it can be useful to work directly with DiagnosticTasks. Look
  // at FrequencyStatus and TimestampStatus in update_functions.h for a
  // real-life example of how to make a DiagnosticTask by deriving from
  // DiagnosticTask.
  
  // Alternatively, a FunctionDiagnosticTask is a derived class from
  // DiagnosticTask that can be used to create a DiagnosticTask from
  // a function. This will be useful when combining multiple diagnostic
  // tasks using a CompositeDiagnosticTask.
  diagnostic_updater::FunctionDiagnosticTask lower("Lower-bound check",
      boost::bind(&check_lower_bound, _1));
  diagnostic_updater::FunctionDiagnosticTask upper("Upper-bound check",
      boost::bind(&check_upper_bound, _1));

  // If you want to merge the outputs of two diagnostic tasks together, you
  // can create a CompositeDiagnosticTask, also a derived class from
  // DiagnosticTask. For example, we could combine the upper and lower
  // bounds check into a single DiagnosticTask.
  diagnostic_updater::CompositeDiagnosticTask bounds("Bound check");
  bounds.addTask(&lower);
  bounds.addTask(&upper);

  // We can then add the CompositeDiagnosticTask to our Updater. When it is
  // run, the overall name will be the name of the composite task, i.e., 
  // "Bound check". The summary will be a combination of the summary of the
  // lower and upper tasks (see \ref
  // DiagnosticStatusWrapper::mergeSummarSummary for details on how the
  // merging is done). The lists of key-value pairs will be concatenated.
  updater.add(bounds);

  // You can broadcast a message in all the DiagnosticStatus if your node
  // is in a special state.
  updater.broadcast(0, "Doing important initialization stuff.");

  ros::Publisher pub1 = nh.advertise<std_msgs::Bool>("topic1", 1);
  ros::Publisher pub2_temp = nh.advertise<std_msgs::Bool>("topic2", 1);
  ros::Duration(2).sleep(); // It isn't important if it doesn't take time.

  // Some diagnostic tasks are very common, such as checking the rate
  // at which a topic is publishing, or checking that timestamps are
  // sufficiently recent. FrequencyStatus and TimestampStatus can do these
  // checks for you. 
  //
  // Usually you would instantiate them via a HeaderlessTopicDiagnostic
  // (FrequencyStatus only, for topics that do not contain a header) or a
  // TopicDiagnostic (FrequencyStatus and TimestampStatus, for topics that
  // do contain a header). 
  //
  // Some values are passed to the constructor as pointers. If these values
  // are changed, the FrequencyStatus/TimestampStatus will start operating
  // with the new values. 
  //
  // Refer to diagnostic_updater::FrequencyStatusParam and
  // diagnostic_updater::TimestampStatusParam documentation for details on
  // what the parameters mean:
  double min_freq = 0.5; // If you update these values, the
  double max_freq = 2; // HeaderlessTopicDiagnostic will use the new values.
  diagnostic_updater::HeaderlessTopicDiagnostic pub1_freq("topic1", updater,
      diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10));

  // Note that TopicDiagnostic, HeaderlessDiagnosedPublisher,
  // HeaderlessDiagnosedPublisher and DiagnosedPublisher all descend from
  // CompositeDiagnosticTask, so you can add your own fields to them using
  // the addTask method.
  //
  // Each time pub1_freq is updated, lower will also get updated and its
  // output will be merged with the output from pub1_freq.
  pub1_freq.addTask(&lower); // (This wouldn't work if lower was stateful).

  // If we know that the state of the node just changed, we can force an
  // immediate update.
  updater.force_update();

  // We can remove a task by refering to its name.
  if (!updater.removeByName("Bound check"))
    ROS_ERROR("The Bound check task was not found when trying to remove it.");

  while (nh.ok())
  {
    std_msgs::Bool msg;
    ros::Duration(0.1).sleep();
    
    // Calls to pub1 have to be accompanied by calls to pub1_freq to keep
    // the statistics up to date.
    msg.data = false;
    pub1.publish(msg);
    pub1_freq.tick();

    // We can call updater.update whenever is convenient. It will take care
    // of rate-limiting the updates.
    updater.update();
  }

  return 0; 
}
