#include <pluginlib/plugin_macros.h>
#include <diagnostic_analyzer/diagnostic_analyzer.h>
#include <generic_analyzer/generic_analyzer.h>

BEGIN_PLUGIN_LIST(diagnostic_analyzer::DiagnosticAnalyzer)
REGISTER_PLUGIN(diagnostic_analyzer::GenericAnalyzer)
END_PLUGIN_LIST
