// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <std_msgs/msg/bool.hpp>

double time_to_launch;

/*
 *\brief Used as a tutorial for loading and using diagnostic updater
 *
 * DummyClass and dummy_diagnostics show how to use a diagnostic_updater
 * class.
 */

void dummy_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  // DiagnosticStatusWrapper are a derived class of
  // diagnostic_msgs::DiagnosticStatus provides a set of convenience
  // methods.

  // summary and summaryf set the level and message.
  if (time_to_launch < 10)
    // summaryf for formatted text.
    stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                  "Buckle your seat belt. Launch in %f seconds!",
                  time_to_launch);
  else
    // summary for unformatted text.
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                 "Launch is in a long time. Have a soda.");

  // add and addf are used to append key-value pairs.
  stat.add("Diagnostic Name", "dummy");
  // add transparently handles conversion to string (using a string_stream).
  stat.add("Time to Launch", time_to_launch);
  // addf allows arbitrary printf style formatting.
  stat.addf("Geeky thing to say", "The square of the time to launch %f is %f",
            time_to_launch, time_to_launch * time_to_launch);
}

class DummyClass {
public:
  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                 "This is a silly updater.");

    stat.add("Stupidicity of this updater", 1000.);
  }
};

class DummyTask : public diagnostic_updater::DiagnosticTask {
public:
  DummyTask() : DiagnosticTask("Updater Derived from DiagnosticTask") {}

  void run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                 "This is another silly updater.");
    stat.add("Stupidicity of this updater", 2000.);
  }
};

void check_lower_bound(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  if (time_to_launch > 5)
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Lower-bound OK");
  else
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Too low");

  stat.add("Low-Side Margin", time_to_launch - 5);
}

void check_upper_bound(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  if (time_to_launch < 10)
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Upper-bound OK");
  else
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Too high");

  stat.add("Top-Side Margin", 10 - time_to_launch);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node =
      rclcpp::Node::make_shared("diagnostic_updater_example");

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
  diagnostic_updater::FunctionDiagnosticTask lower(
      "Lower-bound check", std::bind(&check_lower_bound, std::placeholders::_1));
  diagnostic_updater::FunctionDiagnosticTask upper(
      "Upper-bound check", std::bind(&check_upper_bound, std::placeholders::_1));

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

  auto pub1 = node->create_publisher<std_msgs::msg::Bool>("topic1", 1);
  auto pub2_temp = node->create_publisher<std_msgs::msg::Bool>("topic2", 1);
  rclcpp::Rate(2).sleep();  // It isn't important if it doesn't take time.

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
  double min_freq = 0.5;  // If you update these values, the
  double max_freq = 2;  // HeaderlessTopicDiagnostic will use the new values.
  diagnostic_updater::HeaderlessTopicDiagnostic pub1_freq(
      "topic1", updater,
      diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10));

  // Note that TopicDiagnostic, HeaderlessDiagnosedPublisher,
  // HeaderlessDiagnosedPublisher and DiagnosedPublisher all descend from
  // CompositeDiagnosticTask, so you can add your own fields to them using
  // the addTask method.
  //
  // Each time pub1_freq is updated, lower will also get updated and its
  // output will be merged with the output from pub1_freq.
  pub1_freq.addTask(&lower);  // (This wouldn't work if lower was stateful).

  // If we know that the state of the node just changed, we can force an
  // immediate update.
  updater.force_update();

  // We can remove a task by refering to its name.
  if (!updater.removeByName("Bound check"))
    ;  //  ROS_ERROR("The Bound check task was not found when trying to remove
      //  it.");

  while (rclcpp::ok()) {
    std_msgs::msg::Bool msg;
    rclcpp::Rate(0.1).sleep();

    // Calls to pub1 have to be accompanied by calls to pub1_freq to keep
    // the statistics up to date.
    msg.data = false;
    pub1->publish(msg);
    pub1_freq.tick();

    // We can call updater.update whenever is convenient. It will take care
    // of rate-limiting the updates.
    updater.update();
  }

  return 0;
}
