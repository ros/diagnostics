PKG = 'diagnostics_analysis'
import roslib; roslib.load_manifest(PKG)
import rosrecord
import std_msgs.msg

b_stats = {}
p_stats = {}
#stats['tracked_values'] = ['Programmed current', 'Measured current', 'Board temperature', 'Bridge temperature', 'Supply voltage', 'Drops', 'Max Consecutive Drops']
b_stats['tracked_values'] = ['voltage (V)', 'current (A)', 'average current (1 minute average A)', 'temperature (K)', 'relative charge (%)']
p_stats['tracked_values'] = ['Input Current', 'Board Temperature', 'Min Voltage', 'Max Current' ]
b_tokens = b_stats['tracked_values']
p_tokens = p_stats['tracked_values']

def update(topic, msg):
  global b_stats
  global p_stats
  global b_tokens
  global p_tokens

  if(not (topic == '/diagnostics')):
    print "discarding message on topic " + topic
    return

  for status in msg.status:
    name = status.name
    if(name.startswith('Power board')):
      #print "name=" + name
      if(not p_stats.has_key(name)):
        p_stats[name] = {} 
        for l in p_tokens:
          p_stats[name][l] = []
      for value in status.values:
        #print "value=%s" %(value)
        if value.label in p_tokens:
          #print "%s label=%s  value=%s" %(name, value.label, value.value)
          p_stats[name][value.label].append(float(value.value))
    elif(name.startswith('Smart Battery')):
      #print "name=" + name
      if(not b_stats.has_key(name)):
        b_stats[name] = {} 
        for l in b_tokens:
          b_stats[name][l] = []
      for value in status.values:
        #print "value=%s" %(value)
        if value.label in b_tokens:
          #print "%s label=%s  value=%s" %(name, value.label, value.value)
          b_stats[name][value.label].append(float(value.value))

def output():
  import cPickle
  cPickle.dump(b_stats, file('b_stats.out', 'wb'), -1)
  cPickle.dump(p_stats, file('p_stats.out', 'wb'), -1)
  print "Wrote files"

def process_bag(update, bagfile):
  for (topic, msg, t) in rosrecord.logplayer(bagfile, raw=False):
    update(topic, msg)

if __name__ == '__main__':
  import sys
  #stats = init()
  for i, f in enumerate(sys.argv[1:]): 
    print "(%d) processing file: %s" %(i, f) 
    try:
      process_bag(update, f)
    except:
      print "Failed to process bag " + f
  output()
