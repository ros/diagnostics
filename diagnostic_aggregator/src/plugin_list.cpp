#include <pluginlib/class_list_macros.h>
#include <diagnostic_aggregator/analyzer.h>
#include <diagnostic_aggregator/generic_analyzer.h>

PLUGINLIB_REGISTER_CLASS(GenericAnalyzer, diagnostic_aggregator::GenericAnalyzer, diagnostic_aggregator::Analyzer)
