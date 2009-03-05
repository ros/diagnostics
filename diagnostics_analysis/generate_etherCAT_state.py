PKG = 'diagnostics_analysis'
import roslib; roslib.load_manifest(PKG)
import rosrecord
import std_msgs.msg

def init():
  print "Starting to gather statistics"
  stats = {}
  stats['tracked_values'] = ['Programmed current', 'Measured current', 'Board temperature', 'Bridge temperature', 'Supply voltage', 'Drops', 'Max Consecutive Drops']
  return stats

def update(stats, topic, msg):
  if(not (topic == '/diagnostics')):
    print "discarding message on topic " + topic
    return
  tracked_values = stats['tracked_values']
  for status in msg.status:
    name = status.name
    if(name.startswith('EtherCAT Device')):
      if(not stats.has_key(name)):
        stats[name] = {} 
        for l in tracked_values:
          stats[name][l] = []
      for value in status.strings:
        if value.label in tracked_values:
          stats[name][value.label].append(float(value.value))

def output(stats):
  import pickle
  pickle.dump(stats, file('stats.out', 'w'))
  print "Wrote file 'stats.out'"

def process_bag(stats, update, bagfile):
  for (topic, msg, t) in rosrecord.logplayer(bagfile, raw=False):
    update(stats, topic, msg)

if __name__ == '__main__':
  import sys
  stats = init()
  for i, f in enumerate(sys.argv[1:]): 
    print "(%d) processing file: %s" %(i, f) 
    try:
      process_bag(stats, update, f)
    except:
      print "Failed to process bag " + f
  output(stats)
