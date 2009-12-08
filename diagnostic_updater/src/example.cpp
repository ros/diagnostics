#include <diagnostic_updater/diagnostic_updater.h>

double time_to_launch;

void dummy_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  // DiagnosticStatusWrapper are a derived class of 
  // diagnostic_msgs::DiagnosticStatus provides a set of convenience
  // methods.
  
  // summary and summaryf set the level and message.
  if (time_to_launch < 10)
    // summaryf for formatted text.
    stat.summaryf(2, "Buckle your seat belt. Launch in %f seconds!", time_to_launch);
  else
    // summary for unformatted text.
    stat.summary(0, "Launch is in a long time. Have a soda.");

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
  produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    stat.summary(1, "This is a silly updater.");

    stat.add("Stupidicity of this updater", 1000.);
  }
}

int main()
{
  DummyClass dc;
  ros::NodeHandle nh;
  
  // The Updater class advertises to /diagnostics, and has a
  // ~diagnostic_period parameter that says how often the diagnostics
  // should be published.
  diagnostic_updater::Updater updater;

  // Diagnostic tasks are added to the Updater. They will later be run when
  // the updater decides to update. Many different things can be added.
  // Check doxygen for the full list:
  updater.add("Function updater", dummy_diagnostic);
  updater.add("Method updater", &dc, &DummyClass::produce_diagnostics);
  
  // You can broadcast a message in all the DiagnosticStatus if your node
  // is in a special state.
  updater.broadcast(0, "Doing important initialization stuff.");

  ros::Publisher pub1 = nh.advertise<std_msgs::Bool>("topic1", 1);
  ros::Publisher pub2 = nh.advertise<std_msgs::Bool>("topic2", 1);
  ros::Duration(2).sleep(); // It isn't important if it doesn't take time.

  // If we know that the state of the node just changed, we can force an
  // immediate update.
  updater.force_update();

  while (nh.ok())
  {
    ros::Duration(0.1).sleep();

    // We can call updater.update whenever is convenient. It will take care
    // of rate-limiting the updates.
    updater.update();
  }

  return 0; 
}

	 - A hierarchy of descendents of \ref diagnostic_updater::DiagnosticTask,
		 which each have a name and a way of producing a
		 diagnostic_msgs::DiagnosticStatusWrapper, and that can be used to
		 facilitate the creation of DiagnosticTasks:

		 - \ref diagnostic_updater::DiagnosticTask : The abstract base class
		 
		 - \ref diagnostic_updater::GenericFunctionDiagnosticTask : A diagnostic
			 Task based off a boost::function.

		 - \ref diagnostic_updater::CombinationDiagnosticTask : A diagnostic
			 task that merges the results from a list of
			 diagnostic_updater::DiagnosticTask classes into a single
			 DiagnosticStatusWrapper.

		 @b Note: Most of the time, user code can just use the appropriate add
		 method for diagnostic_updater::Updater with a pointer to a function, a
		 pointer to a method or a boost::function.
  
 - Frequency and Interval statistics in \ref update_functions.h and \ref
	 publisher.h

   - \ref diagnostic_updater::FrequencyStatus and 
     \ref diagnostic_updater::TimeStampStatus, two sublcasses of 
     diagnostic_updater::DiagnosticTask that respectively
  	 compute frequency statistics, and do checks on intervals between events.
  
   - \ref diagnostic_updater::HeaderlessTopicDiagnostic and
  	 \ref diagnostic_updater::TopicDiagnostic, two subclasses of
  	 diagnostic_updater::CombinationDiagnosticTask that allow
  	 diagnostic_updaters::FrequencyStatus and
  	 diagnostic_updaters::TimeStampStatus to be easily applied to
  	 publications of a topic.
  
   - \ref diagnostic_updaters::HeaderlessDiagnosedPublisher and
     \ref diagnostic_updaters::DiagnosedPublisher, two classes that make
  	 diagnostic_updater::HeaderlessTopicDiagnostic and
  	 diagnostic_updater::TopicDiagnostic even easier to use with a standard
  	 ros::Publisher.

