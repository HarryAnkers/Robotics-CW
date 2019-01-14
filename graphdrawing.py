import logfileplot

from os import listdir
from os.path import isfile, join
logfiles = [f for f in listdir("logfiles") if isfile(join("logfiles", f))]

#logfilepath = raw_input("Give a logfile name: ")

for logfile in logfiles:
  try:
    logfileplot.plotgraph('logfiles/' + logfile)
  except IndexError:
    print("Oops!  Logfile:%s in wrong format!  Try again..." % logfile)
  
