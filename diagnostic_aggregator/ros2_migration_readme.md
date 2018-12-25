Overview

Aggregator is a node that subscribes to /diagnostics, processes it
and republishes aggregated data on /diagnostics_agg. The aggregator
creates a series of analyzers according to the specifications of its
private parameters. The aggregated diagnostics data is organized

ROS2 Migration changes 

1> Basic design and concept of aggregator node is same as per ros,this
node contain two analyzers GenericAnalyzer and AnalyzerGroup to hold 
and categorise messsages. 

2> Defination of base anlyzer class has been changed and extra parameter 
added to pass paramters to analyzers 

3> base analyzers will have following fuctionalities as per ros 

    init() -  Loads parameters from yaml file

    match() - Returns true/false if interested in viewing a status message

    analyze() - Analyze a new message

    report() - Report results or state

    getPath() - Get complete path (anything prepended onto status names)

    getName() - Get nice name, like "Motors" or "Sensors" 


4> To configure the analyzer we give analyzer paramter in private namespace 
'analyzers_params' as below :

                analyzers_params:
                          prefix1:
                            type: diagnostic_aggregator/GenericAnalyzer
                            path: First
                            remove_prefix: [ 'prefix1' ]
                            find_and_remove_prefix: [ 'find1_items']
                            startswith: [
                              'pref1a' ]
                            contains: [
                              'contains1a']
                            name: [
                              'name1' ]
                            expected: [
                              'prefix1: expected1a'] 


5> Xmlrpc variables replacced by string tyes and required modification/changes incorporated. 

6> Parametres for analyzers taken from yaml file only so analyzers created using paramter 
service and corresponding code and logical parsing changes added to devired class of analyzers 

7> Rest of the features of aggregator is same as previous.  
