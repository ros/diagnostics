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

#include <diagnostic_aggregator/analyzer_group.h>
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <map>
#include <iterator>
#include <iostream> 

using namespace std;
using namespace diagnostic_aggregator;

PLUGINLIB_EXPORT_CLASS(diagnostic_aggregator::AnalyzerGroup, 
                        diagnostic_aggregator::Analyzer)

AnalyzerGroup::AnalyzerGroup() :
  path_(""),
  nice_name_(""),
  analyzer_loader_("diagnostic_aggregator", "diagnostic_aggregator::Analyzer")
{ }

bool AnalyzerGroup::init(const string base_path, const char * nsp,const rclcpp::Node::SharedPtr &nh,const char * rnsp)
{
   auto context = rclcpp::contexts::default_context::get_global_default_context();
  const std::vector<std::string> arguments = {};
  const std::vector<rclcpp::Parameter> initial_values = {
       rclcpp::Parameter("an_base_path",""),
       rclcpp::Parameter("an_pub_rate", 1.0),
     };
   const bool use_global_arguments = true;
   const bool use_intra_process = true;

 	

  if (base_path.size() > 0 && base_path != "/")
    path_ = base_path + "/" + nice_name_;
  else
    path_ = nice_name_;


  if (path_.find("/") != 0)
    path_ = "/" + path_;

  string anz_name;
  string an_name = nsp; 

  if (an_name.compare("analyzers") !=0)
	  path_ = nsp;
  
 
    cout<< "ANALYZER to be created with name = " << anz_name  << "and path is " << path_ << endl;
   // analyzers_nh  = std::make_shared<rclcpp::Node>("analyzers",path_, context, arguments, initial_values, use_global_arguments, use_intra_process);
      analyzers_nh = nh;


   // rclcpp::Parameter bp = analyzers_nh->get_parameter("an_base_path");
   // cout << "rclcpp::Parameter bp =" << bp <<endl;
   // rclcpp::Parameter bp1 = analyzers_nh->get_parameter("an_pub_rate");
   // cout << "rclcpp::Parameter bp =" << bp1 <<endl;
  //ros::NodeHandle analyzers_nh = ros::NodeHandle(n, "analyzers");
 // analyzers_nh.getParam("", analyzer_params);
//  rclcpp::SyncParametersClient::SharedPtr parameters_client = std::make_shared<rclcpp::SyncParametersClient>(analyzers_nh,nsp);
  rclcpp::SyncParametersClient::SharedPtr parameters_client = std::make_shared<rclcpp::SyncParametersClient>(nh);
    cout<<"analyzers Node created  "<<  path_  << "and name is "  << nh->get_name() << endl;
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(analyzers_nh->get_logger(), "Interrupted while waiting for the service. Exiting.")
      return 0;
    }
    RCLCPP_INFO(analyzers_nh->get_logger(), "service not available, waiting again...")
  }

    RCLCPP_INFO(analyzers_nh->get_logger(), "service is  available Now ")
                 std::stringstream ss;
                 std::stringstream ss1;
	  
  bool init_ok = true;

   map <string, string> anl_param;
  auto parameters_and_prefixes = parameters_client->list_parameters({"analyzers_params"}, 0);

  cout<<"parameters_client->get_parameters called for analyzer group"<<endl;
  ss << "\nParameter names:";
  for (auto & name : parameters_and_prefixes.names) {
  //  ss << "\n " << name;
    for (auto & parameter : parameters_client->get_parameters({name})) {
    ss1 << "\nParameter name: " << parameter.get_name();
    ss1 << "\nParameter value (" << parameter.get_type_name() << "): " <<
      parameter.value_to_string();
      anl_param[parameter.get_name()]=parameter.value_to_string();
  }
    //anl_param.insert(std::pair<string, string>(ss,ss1)); 
  }

    RCLCPP_INFO(analyzers_nh->get_logger(), ss1.str().c_str())


 	


 
   for(map<string, string>::iterator anl_it = anl_param.begin(); anl_it != anl_param.end(); ++anl_it ){
       string analyzer_name = anl_it->first;
       string ns = anl_it->second;
       cout<< analyzer_name  << "::"<<ns <<endl;
	       std::shared_ptr<Analyzer> analyzer;
	       string an_type= anl_it->second ;
#if 0
       	if(std::string::npos != analyzer_name.find("path")){
 	    	   
		if (base_path.size() > 0 && base_path != "/")
			path_ = base_path + "/" + anl_it->second;
		else
			path_ = anl_it->second;
		

		if (path_.find("/") != 0)
			path_ = "/" + path_;
		cout<< "Name of path is set to " << path_ << endl;
	}   
#endif     	
       if(std::string::npos != analyzer_name.find("type"))
       {
	       try
	       {
		       // Look for non-fully qualified class name for Analyzer type
		       if (!analyzer_loader_.isClassAvailable(an_type))
		       {

			       bool have_class = false;
			       vector<string> classes = analyzer_loader_.getDeclaredClasses();
			       for(unsigned int i = 0; i < classes.size(); ++i)
			       {
				       if(an_type == analyzer_loader_.getName(classes[i]))
				       {
					       //if we've found a match... we'll get the fully qualified name and break out of the loop
					       ROS_WARN("Analyzer specification should now include the package name. You are using a deprecated API. Please switch from %s to %s in your Analyzer specification.",
							       an_type.c_str(), classes[i].c_str());
					       an_type = classes[i];
					       have_class = true;
					       break;
				       }
					       cout<< "available clase is " << classes[i] << endl;
			       }
			       if (!have_class)
			       {
				       ROS_ERROR("Unable to find Analyzer class %s. Check that Analyzer is fully declared.", an_type.c_str());
				       continue;
			       }
		       }

		       //analyzer = analyzer_loader_.createInstance(an_type);
		       analyzer = analyzer_loader_.createSharedInstance(an_type);
		        cout<<"analyzer loader class is available and instance created of "<< an_type << endl;
	       }
	       catch (pluginlib::LibraryLoadException& e)
	       {
		       ROS_ERROR("Failed to load analyzer %s, type %s. Caught exception. %s", ns.c_str(), an_type.c_str(), e.what());
		       cout<<"failed to load analyzer";
		       std::shared_ptr<StatusItem> item(new StatusItem(ns, "Pluginlib exception loading analyzer"));
		       aux_items_.push_back(item);
		       init_ok = false;
		       continue;
	       }

       }else{

	continue;
       }       
       if (!analyzer)
       {
	       //ROS_ERROR("Pluginlib returned a null analyzer for %s, namespace %s.", an_type.c_str(), analyzers_nh.getNamespace().c_str());
	       cout<<"Pluginlib returned a null analyzer for , namespace."<< an_type.c_str();
	       std::shared_ptr<StatusItem> item(new StatusItem(ns, "Pluginlib return NULL Analyzer for " + an_type));
	       aux_items_.push_back(item);
	       init_ok = false;
	       continue;
       }

       //if (!analyzer->init(path_, ros::NodeHandle(analyzers_nh, ns)))
//       if( an_type.compare("diagnostic_aggregator/AnalyzerGroup") != 0) {
       //if (!analyzer->init(path_, analyzer_name.c_str(),analyzers_nh,rnsp))
       if (!analyzer->init(path_, analyzer_name.c_str(),nh,rnsp))
       {
	       //ROS_ERROR("Unable to initialize analyzer NS: %s, type: %s", analyzers_nh.getNamespace().c_str(), an_type.c_str());
	       cout<<" Vaibhav: Unable to initialize analyzer NS"<<  analyzer_name.c_str()  <<endl;
	       std::shared_ptr<StatusItem> item(new StatusItem(ns, "Analyzer init failed"));
	       aux_items_.push_back(item);
	       init_ok = false;
	       continue;
       }
  //     }
       analyzers_.push_back(analyzer);

   }	   
  if (analyzers_.size() == 0)
  {
    init_ok = false;
    //ROS_ERROR(" Vaibhav: No analyzers initialized in AnalyzerGroups");
    cout<<" Vaibhav: No analyzers initialized in AnalyzerGroup " << endl;
  }else {

     cout<<"analyzer created and push back " << analyzers_.size() << endl; 
  }
  return init_ok;
}

AnalyzerGroup::~AnalyzerGroup()
{
  analyzers_.clear();
}

bool AnalyzerGroup::addAnalyzer(std::shared_ptr<Analyzer>& analyzer)
{
  analyzers_.push_back(analyzer);
  return true;
}

bool AnalyzerGroup::removeAnalyzer(std::shared_ptr<Analyzer>& analyzer)
{
  vector<std::shared_ptr<Analyzer> >::iterator it = find(analyzers_.begin(), analyzers_.end(), analyzer);
  if (it != analyzers_.end())
  {
    analyzers_.erase(it);
    return true;
  }
  return false;
}

bool AnalyzerGroup::match(const string name)
{
  if (analyzers_.size() == 0)
    return false;

  bool match_name = false;
  if (matched_.count(name))
  {
    vector<bool> &mtch_vec = matched_[name];
    for (unsigned int i = 0; i < mtch_vec.size(); ++i)
    {
      if (mtch_vec[i])
        return true;
    }
    return false;
  }
  
  matched_[name].resize(analyzers_.size());
  for (unsigned int i = 0; i < analyzers_.size(); ++i)
  {
    bool mtch = analyzers_[i]->match(name);
    match_name = mtch || match_name;
    matched_[name].at(i) = mtch;
  }

  return match_name;
}

void AnalyzerGroup::resetMatches()
{
  matched_.clear();
}


bool AnalyzerGroup::analyze(const std::shared_ptr<StatusItem> item)
{
  //ROS_ASSERT_MSG(matched_.count(item->getName()), "AnalyzerGroup was asked to analyze an item it hadn't matched.");

  bool analyzed = false;
  vector<bool> &mtch_vec = matched_[item->getName()];
  for (unsigned int i = 0; i < mtch_vec.size(); ++i)
  {
    if (mtch_vec[i])
      analyzed = analyzers_[i]->analyze(item) || analyzed;
  }
  
  return analyzed;
}

vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus> > AnalyzerGroup::report()
{
  vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus> > output;

  std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus> header_status(new diagnostic_msgs::msg::DiagnosticStatus);
  header_status->name = path_;
  header_status->level = 0;
  header_status->message = "OK";

  if (analyzers_.size() == 0)
  {
    header_status->level = 2;
    header_status->message = "No analyzers";
    output.push_back(header_status);
    
    if (header_status->name == "" || header_status->name == "/")
      header_status->name = "/AnalyzerGroup";

    return output;
  }

  bool all_stale = true;

  for (unsigned int j = 0; j < analyzers_.size(); ++j)
  {
    string path = analyzers_[j]->getPath();
    string nice_name = analyzers_[j]->getName();

    vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus> > processed = analyzers_[j]->report();

    // Do not report anything in the header values for analyzers that don't report
    if (processed.size() == 0)
      continue;

    // Look through processed data for header, append it to header_status
    // Ex: Look for /Robot/Power and append (Power, OK) to header
    for (unsigned int i = 0; i < processed.size(); ++i)
    {
      output.push_back(processed[i]);

      // Add to header status
      if (processed[i]->name == path)
      {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = nice_name;
        kv.value = processed[i]->message;
        
        all_stale = all_stale && (processed[i]->level == 3);
        header_status->level = max(header_status->level, processed[i]->level);
        header_status->values.push_back(kv);
      }
    }
  }

  // Report stale as errors unless all stale
  if (header_status->level == 3 && !all_stale)
    header_status->level = 2;

  header_status->message = valToMsg(header_status->level);

  if (path_ != "" && path_ != "/") // No header if we don't have a base path
  {
    output.push_back(header_status);
  }

  for (unsigned int i = 0; i < aux_items_.size(); ++i)
  {
    output.push_back(aux_items_[i]->toStatusMsg(path_, true));
  }

  return output;
}
