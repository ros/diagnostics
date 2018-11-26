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

/**! \author Kevin Watts */

#include <diagnostic_aggregator/aggregator.h>
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std;
using namespace diagnostic_aggregator;

Aggregator::Aggregator() :
  pub_rate_(1.0),
  analyzer_group_(NULL),
  other_analyzer_(NULL),
  base_path_("")
{
	auto context = rclcpp::contexts::default_context::get_global_default_context();
	const std::vector<std::string> arguments = {};
	const std::vector<rclcpp::Parameter> initial_values = {
		rclcpp::Parameter("base_path","/"),
		rclcpp::Parameter("pub_rate", pub_rate_),
	};
	const bool use_global_arguments = true;
	const bool use_intra_process = true;


	if (base_path_.size() > 0 && base_path_.find("/") != 0)
		base_path_ = "/" + base_path_;
	else
		base_path_ = "/";


	nh = std::make_shared<rclcpp::Node>("diagnostic_aggregator","/", context, arguments, initial_values, use_global_arguments, use_intra_process);
	auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(nh);
	while (!parameters_client->wait_for_service(1s)) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(nh->get_logger(), "Interrupted while waiting for the service. Exiting.")
		}
		RCLCPP_INFO(nh->get_logger(), "service not available, waiting again...")
	}

	RCLCPP_INFO(nh->get_logger(), "service is  available Now ")
		std::stringstream ss,ss1;



	analyzer_group_ = new AnalyzerGroup();




	if (!analyzer_group_->init(base_path_,"analyzers",nh,"gen_analyzers"))
	{
		ROS_ERROR("Analyzer group for diagnostic aggregator failed to initialize!");
	}
	// Last analyzer handles remaining data
	other_analyzer_ = new OtherAnalyzer();
	other_analyzer_->init(base_path_); // This always returns true
	auto handle_add_agreegator =
		[this](const std::shared_ptr<rmw_request_id_t> request_header,
				const std::shared_ptr<diagnostic_msgs::srv::AddDiagnostics::Request> req,
				std::shared_ptr<diagnostic_msgs::srv::AddDiagnostics::Response> res) -> bool
		{
			(void)request_header;
			// Don't currently support relative or private namespace definitions
			if (req->load_namespace[0] != '/')
			{
				res->message = "Requested load from non-global namespace. Private and relative namespaces are not supported.";
				res->success = false;
				return true;
			}
			std::shared_ptr<Analyzer> group = std::make_shared<AnalyzerGroup>();

			rclcpp::Node::SharedPtr nh_temp;
			if (group->init(base_path_, req->load_namespace.c_str(),nh_temp,req->load_namespace.c_str()))
			{
				res->message = "Successfully initialised AnalyzerGroup. Waiting for bond to form.";
				res->success = true;
				return true;
			}
			else
			{
				res->message = "Failed to initialise AnalyzerGroup.";
				res->success = false;
				return true;
			}

		};




	std::function<void(diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr)> cb_std_function= std::bind( &Aggregator::diagCallback,this,std::placeholders::_1);
	add_srv_ = nh->create_service<diagnostic_msgs::srv::AddDiagnostics>("/diagnostics_agg/add_diagnostics", handle_add_agreegator);
	diag_sub_ = nh->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", cb_std_function,rmw_qos_profile_default);
	agg_pub_ = nh->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics_agg");
	toplevel_state_pub_ = nh->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/diagnostics_toplevel_state");
}

void Aggregator::checkTimestamp(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr& diag_msg)
{
	if (((diag_msg->header.stamp.nanosec)*1e-9) != 0)
		return;

	string stamp_warn = "No timestamp set for diagnostic message. Message names: ";
	vector<diagnostic_msgs::msg::DiagnosticStatus>::const_iterator it;
	for (it = diag_msg->status.begin(); it != diag_msg->status.end(); ++it)
	{
		if (it != diag_msg->status.begin())
			stamp_warn += ", ";
		stamp_warn += it->name;
	}

	if (!ros_warnings_.count(stamp_warn))
	{
		ROS_WARN("%s", stamp_warn.c_str());
		ros_warnings_.insert(stamp_warn);
	}
}

void Aggregator::diagCallback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr& diag_msg)
{
	checkTimestamp(diag_msg);

	bool analyzed = false;
	{ // lock the whole loop to ensure nothing in the analyzer group changes
		// during it.
		//std::mutex::scoped_lock lock(mutex_);
		std::unique_lock<std::mutex> lock(mutex_);
		for (unsigned int j = 0; j < diag_msg->status.size(); ++j)
		{
			analyzed = false;
			std::shared_ptr<StatusItem> item(new StatusItem(&diag_msg->status[j]));

			if (analyzer_group_->match(item->getName())){
				analyzed = analyzer_group_->analyze(item);

			}else{
				cout<< "No match found for "<<item->getName() <<endl;
			}
			if (!analyzed)
				other_analyzer_->analyze(item);
		}
	}
}

Aggregator::~Aggregator()
{
	if (analyzer_group_) delete analyzer_group_;

	if (other_analyzer_) delete other_analyzer_;
}


void Aggregator::publishData()
{
	// diagnostic_msgs::msg::DiagnosticArray diag_array;

	diagnostic_msgs::msg::DiagnosticStatus diag_toplevel_state;
	diag_toplevel_state.name = "toplevel_state";
	diag_toplevel_state.level = -1;
	int min_level = 255;

	std::shared_ptr<diagnostic_msgs::msg::DiagnosticArray> diag_array = std::make_shared<diagnostic_msgs::msg::DiagnosticArray>();



	vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus> > processed;
	{
		std::unique_lock<std::mutex> lock(mutex_);
		processed = analyzer_group_->report();
	}
	for (unsigned int i = 0; i < processed.size(); ++i)
	{
		diag_array->status.push_back(*processed[i]);

		if (processed[i]->level > diag_toplevel_state.level)
			diag_toplevel_state.level = processed[i]->level;
		if (processed[i]->level < min_level)
			min_level = processed[i]->level;
	}

	vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus> > processed_other = other_analyzer_->report();
	for (unsigned int i = 0; i < processed_other.size(); ++i)
	{
		diag_array->status.push_back(*processed_other[i]);

		if (processed_other[i]->level > diag_toplevel_state.level)
			diag_toplevel_state.level = processed_other[i]->level;
		if (processed_other[i]->level < min_level)
			min_level = processed_other[i]->level;
	}

	/*  diag_array.header.stamp = ros::Time::now();*/
	rclcpp::Clock ros_clock(RCL_ROS_TIME);
	using builtin_interfaces::msg::Time;
	Time ros_now = ros_clock.now();
	diag_array->header.stamp.sec = ros_now.sec;
	diag_array->header.stamp.nanosec = ros_now.nanosec;
	agg_pub_->publish(diag_array);
	// Top level is error if we have stale items, unless all stale
	if (diag_toplevel_state.level > 2 && min_level <= 2)
		diag_toplevel_state.level = 2;

	toplevel_state_pub_->publish(diag_toplevel_state);
}
