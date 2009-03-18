#!/usr/bin/python
PKG = 'diagnostics_analysis'
import roslib; roslib.load_manifest(PKG)
import rosrecord
import std_msgs.msg
import time

def init():
  print "Starting to gather statistics"
  stats = {}
  return stats

def update(stats, topic, msg):
  if(not (topic == '/diagnostics')):
    print "discarding message on topic " + topic
    return
  t = time.localtime(float(str(msg.header.stamp)) / 1000000000.0)
   
  for status in msg.status:
    name = status.name
    if(not stats.has_key(name)):
      stats[name] = {}
      stats[name]['string_fields'] = [s.label for s in status.strings]
      stats[name]['float_fields'] = [s.label for s in status.values]
      stats[name]['file'] = file('output/' + status.name + '.csv', 'w')
      fields = stats[name]['string_fields'] + stats[name]['float_fields'];
      stats[name]['file'].write(', '.join(['timestamp'] + [f.replace(',','') for f in fields]) + '\n')
    
    if (not [s.label for s in status.strings] == stats[name]['string_fields']):
      print "ERROR, mismatched field names in component %s" %(name)
      return stats
    if (not [s.label for s in status.values] == stats[name]['float_fields']):
      print "ERROR, mismatched field names in component %s" %(name)
      return stats
    stats[name]['file'].write(', '.join([time.asctime(t)] + [s.value.replace('\n', ' ').replace(',','') for s in status.strings] + [str(s.value) for s in status.values]) + '\n')


def output(stats):
  for name in stats:
    stats[name]['file'].close()

def process_bag(stats, update, bagfile):
  for (topic, msg, t) in rosrecord.logplayer(bagfile, raw=False):
    update(stats, topic, msg)

if __name__ == '__main__':
  import sys
  stats = init()
  for i, f in enumerate(sys.argv[1:]): 
    print "(%d) processing file: %s" %(i, f) 
    #try:
    process_bag(stats, update, f)
    #except:
    #  print "Failed to process bag " + f
  output(stats)
