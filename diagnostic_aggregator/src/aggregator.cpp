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

#include <string>
#include <vector>
#include <memory>
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "diagnostic_aggregator/aggregator.hpp"

//  using namespace std;
//  using namespace diagnostic_aggregator;

diagnostic_aggregator::Aggregator::Aggregator()
: pub_rate_(1.0), analyzer_group_(NULL), other_analyzer_(NULL),
  base_path_("")
{
  auto context =
    rclcpp::contexts::default_context::get_global_default_context();
  const std::vector<std::string> arguments = {};
  const std::vector<rclcpp::Parameter> initial_values = {
    rclcpp::Parameter("base_path", "/"),
    rclcpp::Parameter("pub_rate", pub_rate_),
  };
  const bool use_global_arguments = true;
  const bool use_intra_process = true;

  if (base_path_.size() > 0 && base_path_.find("/") != 0) {
    base_path_ = "/" + base_path_;
  } else {
    base_path_ = "/";
  }

  nh = std::make_shared<rclcpp::Node>("analyzers", "/", context, arguments,
      initial_values, use_global_arguments,
      use_intra_process);
  nh_an = std::make_shared<rclcpp::Node>(
    "diagnostic_aggregator", "/", context, arguments, initial_values,
    use_global_arguments, use_intra_process);
  //  Listing parameter using syncronous paramter serrvice
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(nh);
  using namespace std::chrono_literals;
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(nh->get_logger(),
        "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(nh->get_logger(), "service not available, waiting again...");
  }

  RCLCPP_INFO(nh->get_logger(), "service is  available Now ");
  std::stringstream ss, ss1;

  analyzer_group_ = new AnalyzerGroup();
  //  Analyzer initialisation: parameter passed is base path, name of node which create analyzer
  //  node share pointer
  if (!analyzer_group_->init(base_path_, nh_an->get_name(), nh_an, NULL)) {
    // if (!analyzer_group_->init(base_path_,"analyzers",nh,"gen_analyzers"))
    ROS_ERROR("Analyzer group for diagnostic aggregator failed to initialize!");
  }
  // Last analyzer handles remaining data
  other_analyzer_ = new OtherAnalyzer();
  other_analyzer_->init(base_path_);  //  This always returns true
  //  Callback for service adding analyzer
  auto handle_add_agreegator =
    [this](
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<diagnostic_msgs::srv::AddDiagnostics::Request>
    req,
    std::shared_ptr<diagnostic_msgs::srv::AddDiagnostics::Response> res)
    -> bool

    {
      (void)request_header;
      // Don't currently support relative or private namespace definitions
      if (req->load_namespace[0] != '/') {
        res->message = "Requested load from non-global namespace. Private and "
          "relative namespaces are not supported.";
        res->success = false;
        return true;
      }
      std::shared_ptr<Analyzer> group = std::make_shared<AnalyzerGroup>();

      { // lock here ensures that bonds from the same namespace aren't added
        // twice. Without it, possibility of two simultaneous calls adding two
        // objects.
        // boost::mutex::scoped_lock lock(mutex_);
        std::unique_lock<std::mutex> lock(mutex_);
        // rebuff attempts to add things from the same namespace twice
        if (std::find_if(bonds_.begin(), bonds_.end(),
          BondIDMatch(req->load_namespace)) != bonds_.end())
        {
          res->message = "Requested load from namespace " + req->load_namespace +
            " which is already in use";
          res->success = false;
          return true;
        }
        std::shared_ptr<bond::Bond> req_bond = std::make_shared<bond::Bond>(
          "/diagnostics_agg/bond", req->load_namespace, nh,
          std::function<void(void)>(std::bind(&Aggregator::bondBroken, this,
          req->load_namespace, group)),
          std::function<void(void)>(
            std::bind(&Aggregator::bondFormed, this, group)));
        req_bond->start();
        bonds_.push_back(req_bond);  // bond formed, keep track of it
      }

      rclcpp::Node::SharedPtr nh_temp;
      if (group->init(base_path_, req->load_namespace.c_str(), nh_an, NULL)) {
        // if (group->init(base_path_,
        // req->load_namespace.c_str(),nh_temp,req->load_namespace.c_str()))
        res->message =
          "Successfully initialised AnalyzerGroup. Waiting for bond to form.";
        res->success = true;
        return true;
      } else {
        res->message = "Failed to initialise AnalyzerGroup.";
        res->success = false;
        return true;
      }
    };

  std::function<void(diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr)>
  cb_std_function =
    std::bind(&Aggregator::diagCallback, this, std::placeholders::_1);
  add_srv_ = nh->create_service<diagnostic_msgs::srv::AddDiagnostics>(
    "/diagnostics_agg/add_diagnostics", handle_add_agreegator);
  diag_sub_ = nh->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", cb_std_function, rmw_qos_profile_default);
  agg_pub_ = nh->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics_agg");
  toplevel_state_pub_ =
    nh->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
    "/diagnostics_toplevel_state");
}

void diagnostic_aggregator::Aggregator::checkTimestamp(
  const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr & diag_msg)
{
  if (((diag_msg->header.stamp.nanosec) * 1e-9) != 0) {
    return;
  }

  std::string stamp_warn =
    "No timestamp set for diagnostic message. Message names: ";
  std::vector<diagnostic_msgs::msg::DiagnosticStatus>::const_iterator it;
  for (it = diag_msg->status.begin(); it != diag_msg->status.end(); ++it) {
    if (it != diag_msg->status.begin()) {
      stamp_warn += ", ";
    }
    stamp_warn += it->name;
  }

  if (!ros_warnings_.count(stamp_warn)) {
    ROS_WARN("%s", stamp_warn.c_str());
    ros_warnings_.insert(stamp_warn);
  }
}

void diagnostic_aggregator::Aggregator::diagCallback(
  const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr & diag_msg)
{
  checkTimestamp(diag_msg);
  bool analyzed = false;
  { // lock the whole loop to ensure nothing in the analyzer group changes
    // during it.
    // std::mutex::scoped_lock lock(mutex_);
    std::unique_lock<std::mutex> lock(mutex_);
    for (unsigned int j = 0; j < diag_msg->status.size(); ++j) {
      analyzed = false;
      std::shared_ptr<StatusItem> item(new StatusItem(&diag_msg->status[j]));

      if (analyzer_group_->match(item->getName())) {
        analyzed = analyzer_group_->analyze(item);

      } else {
        std::cout << "No match found for " << item->getName() << std::endl;
      }
      if (!analyzed) {
        other_analyzer_->analyze(item);
      }
    }
  }
}

diagnostic_aggregator::Aggregator::~Aggregator()
{
  if (analyzer_group_) {
    delete analyzer_group_;
  }

  if (other_analyzer_) {
    delete other_analyzer_;
  }
}

void diagnostic_aggregator::Aggregator::bondBroken(
  std::string bond_id,
  std::shared_ptr<Analyzer> analyzer)
{
  // boost::mutex::scoped_lock lock(mutex_); // Possibility of multiple bonds
  // breaking at once
  std::unique_lock<std::mutex> lock(mutex_);
  ROS_DEBUG("Bond for namespace %s was broken", bond_id.c_str());
  std::vector<std::shared_ptr<bond::Bond>>::iterator elem;
  elem = std::find_if(bonds_.begin(), bonds_.end(), BondIDMatch(bond_id));
  if (elem == bonds_.end()) {
    ROS_WARN("Broken bond tried to erase a bond which didn't exist.");
  } else {
    bonds_.erase(elem);
  }
  if (!analyzer_group_->removeAnalyzer(analyzer)) {
    ROS_WARN("Broken bond tried to remove an analyzer which didn't exist.");
  }

  analyzer_group_->resetMatches();
}

void diagnostic_aggregator::Aggregator::bondFormed(std::shared_ptr<Analyzer> group)
{
  ROS_DEBUG("Bond formed");
  // boost::mutex::scoped_lock lock(mutex_);
  std::unique_lock<std::mutex> lock(mutex_);
  analyzer_group_->addAnalyzer(group);
  analyzer_group_->resetMatches();
}

void diagnostic_aggregator::Aggregator::publishData()
{
  // diagnostic_msgs::msg::DiagnosticArray diag_array;

  diagnostic_msgs::msg::DiagnosticStatus diag_toplevel_state;
  diag_toplevel_state.name = "toplevel_state";
  diag_toplevel_state.level = -1;
  int min_level = 255;

  std::shared_ptr<diagnostic_msgs::msg::DiagnosticArray> diag_array =
    std::make_shared<diagnostic_msgs::msg::DiagnosticArray>();

  std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> processed;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    processed = analyzer_group_->report();
  }
  for (unsigned int i = 0; i < processed.size(); ++i) {
    diag_array->status.push_back(*processed[i]);

    if (processed[i]->level > diag_toplevel_state.level) {
      diag_toplevel_state.level = processed[i]->level;
    }
    if (processed[i]->level < min_level) {
      min_level = processed[i]->level;
    }
  }

  std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>>
  processed_other = other_analyzer_->report();
  for (unsigned int i = 0; i < processed_other.size(); ++i) {
    diag_array->status.push_back(*processed_other[i]);

    if (processed_other[i]->level > diag_toplevel_state.level) {
      diag_toplevel_state.level = processed_other[i]->level;
    }
    if (processed_other[i]->level < min_level) {
      min_level = processed_other[i]->level;
    }
  }

  /*  diag_array.header.stamp = ros::Time::now();*/
  rclcpp::Clock ros_clock(RCL_ROS_TIME);
  using builtin_interfaces::msg::Time;
  Time ros_now = ros_clock.now();
  diag_array->header.stamp.sec = ros_now.sec;
  diag_array->header.stamp.nanosec = ros_now.nanosec;
  agg_pub_->publish(diag_array);
  // Top level is error if we have stale items, unless all stale
  if (diag_toplevel_state.level > 2 && min_level <= 2) {
    diag_toplevel_state.level = 2;
  }

  toplevel_state_pub_->publish(diag_toplevel_state);
}
