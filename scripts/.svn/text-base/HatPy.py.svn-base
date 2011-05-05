#! /usr/bin/env python
from pylab import *
import numpy, time, sys



window = 1000
yscale = 1250

if (len(sys.argv) > 1) :
    window = int(sys.argv[1])
    if (len(sys.argv) > 2) :
        yscale = int(sys.argv[2])

# set interactive plot mode
ion()

# clear previous plot
clf()
sys.stdin.readline()
x = arange(0,window,1)

#data = float(sys.stdin.readline().split()[0])
data = -1250
y = zeros(window)
y[0] = yscale # scaling y axis
y[window-1] = data

pltline, = plot(x,map(None,y))
y = [0.0] * window

try:
    while(1):
        # Read measured values
        timeout=time.time()+0.5
        while (time.time() < timeout):
            y = y[1:]
            y.append(float(sys.stdin.readline().split()[0]))
        pltline.set_ydata(array(y))
        # Update plot
        draw()

except:
    print "End"
