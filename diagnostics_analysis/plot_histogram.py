import pylab
import pickle

stats = pickle.load(file('stats.out'))

def plot_key(label, stats):

  boards = [b for b in stats.keys() if b.count('motor') > 0]

  for index, board in enumerate(boards):
    print board

    pylab.subplot(6, 6, index + 1)
    pylab.hist(stats[board][key], 20)
    pylab.title(board)

  pylab.show()

keys = stats['tracked_values']
while(1):
  print "available keys:" + str(keys)
  key = input("what value would you like to plot?")
  if key in keys:
    plot_key(key, stats)
  else:
    print "Error, key not found"
