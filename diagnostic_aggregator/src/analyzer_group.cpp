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

using namespace std;
using namespace diagnostic_aggregator;

PLUGINLIB_EXPORT_CLASS(diagnostic_aggregator::AnalyzerGroup, 
                        diagnostic_aggregator::Analyzer)

AnalyzerGroup::AnalyzerGroup() :
  path_(""),
  nice_name_(""),
  analyzer_loader_("diagnostic_aggregator", "diagnostic_aggregator::Analyzer")
{ }

bool AnalyzerGroup::init(const string base_path, const ros::NodeHandle &n)
{
  n.param("path", nice_name_, string(""));
  
  if (base_path.size() > 0 && base_path != "/")
    if (nice_name_.size() > 0)
      path_ = base_path + "/" + nice_name_;
    else
      path_ = base_path;
  else
    path_ = nice_name_;


  if (path_.find("/") != 0)
    path_ = "/" + path_;

  ros::NodeHandle analyzers_nh = ros::NodeHandle(n, "analyzers");
    
  XmlRpc::XmlRpcValue analyzer_params;
  analyzers_nh.getParam("", analyzer_params);
  ROS_DEBUG("Analyzer params: %s.", analyzer_params.toXml().c_str());

  bool init_ok = true;

  XmlRpc::XmlRpcValue::iterator xml_it;
  for (xml_it = analyzer_params.begin(); xml_it != analyzer_params.end(); ++xml_it)
  {
    XmlRpc::XmlRpcValue analyzer_name = xml_it->first;
    ROS_DEBUG("Got analyzer name: %s", analyzer_name.toXml().c_str());
    XmlRpc::XmlRpcValue analyzer_value = xml_it->second;

    string ns = analyzer_name;
    
    if (!analyzer_value.hasMember("type"))
    {
      ROS_ERROR("Namespace %s has no member 'type', unable to initialize analyzer for this namespace.", analyzers_nh.getNamespace().c_str());
      boost::shared_ptr<StatusItem> item(new StatusItem(ns, "No Analyzer type given"));
      aux_items_.push_back(item);
      init_ok = false;
      continue;
    }

    XmlRpc::XmlRpcValue analyzer_type = analyzer_value["type"];
    string an_type = analyzer_type;
    
    boost::shared_ptr<Analyzer> analyzer;
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
        }
        if (!have_class)
        {
          ROS_ERROR("Unable to find Analyzer class %s. Check that Analyzer is fully declared.", an_type.c_str());
          continue;
        }
      }

      analyzer = analyzer_loader_.createInstance(an_type);
    }
    catch (pluginlib::LibraryLoadException& e)
    {
      ROS_ERROR("Failed to load analyzer %s, type %s. Caught exception. %s", ns.c_str(), an_type.c_str(), e.what());
      boost::shared_ptr<StatusItem> item(new StatusItem(ns, "Pluginlib exception loading analyzer"));
      aux_items_.push_back(item);
      init_ok = false;
      continue;
    }

    if (!analyzer)
    {
      ROS_ERROR("Pluginlib returned a null analyzer for %s, namespace %s.", an_type.c_str(), analyzers_nh.getNamespace().c_str());
      boost::shared_ptr<StatusItem> item(new StatusItem(ns, "Pluginlib return NULL Analyzer for " + an_type));
      aux_items_.push_back(item);
      init_ok = false;
      continue;
    }
    
    if (!analyzer->init(path_, ros::NodeHandle(analyzers_nh, ns)))
    {
      ROS_ERROR("Unable to initialize analyzer NS: %s, type: %s", analyzers_nh.getNamespace().c_str(), an_type.c_str());
      boost::shared_ptr<StatusItem> item(new StatusItem(ns, "Analyzer init failed"));
      aux_items_.push_back(item);
      init_ok = false;
      continue;
    }
    
    analyzers_.push_back(analyzer);
  }

  if (analyzers_.size() == 0)
  {
    init_ok = false;
    ROS_ERROR("No analyzers initialized in AnalyzerGroup %s", analyzers_nh.getNamespace().c_str());
  }

  return init_ok;
}

AnalyzerGroup::~AnalyzerGroup()
{
  analyzers_.clear();
}

bool AnalyzerGroup::addAnalyzer(boost::shared_ptr<Analyzer>& analyzer)
{
  analyzers_.push_back(analyzer);
  return true;
}

bool AnalyzerGroup::removeAnalyzer(boost::shared_ptr<Analyzer>& analyzer)
{
  vector<boost::shared_ptr<Analyzer> >::iterator it = find(analyzers_.begin(), analyzers_.end(), analyzer);
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


bool AnalyzerGroup::analyze(const boost::shared_ptr<StatusItem> item)
{
  ROS_ASSERT_MSG(matched_.count(item->getName()), "AnalyzerGroup was asked to analyze an item it hadn't matched.");

  bool analyzed = false;
  vector<bool> &mtch_vec = matched_[item->getName()];
  for (unsigned int i = 0; i < mtch_vec.size(); ++i)
  {
    if (mtch_vec[i])
      analyzed = analyzers_[i]->analyze(item) || analyzed;
  }
  
  return analyzed;
}

vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > AnalyzerGroup::report()
{
  vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > output;

  boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> header_status(new diagnostic_msgs::DiagnosticStatus);
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

    vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > processed = analyzers_[j]->report();

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
        diagnostic_msgs::KeyValue kv;
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
