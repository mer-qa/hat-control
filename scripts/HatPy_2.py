#! /usr/bin/env python
from pylab import *
import numpy, time, sys


samplerate = 200
window = 1000
y1scale = 50
y2scale = 1250

if (len(sys.argv) > 1) :
    window = int(sys.argv[1])
    if (len(sys.argv) > 2) :
        y1scale = int(sys.argv[2])

# set interactive plot mode
ion()

# clear previous plot
clf()
sys.stdin.readline()
x = arange(0.0,window/samplerate,1.0/samplerate)

data = float(sys.stdin.readline().split()[0])

y1 = zeros(window)
y1[0] = y1scale # scaling y axis
y1[window-1] = 0

y2 = zeros(window)
y2[0] = y2scale # scaling y axis
y2[window-1] = -y2scale #data

ax1 = subplot(211)
pltline1, = plot(x,map(None,y1))
y1 = [0.0] * window
setp( pltline1, color='r')
setp( ax1.get_xticklabels(), fontsize=10)
setp( ax1.get_yticklabels(), fontsize=10)
ylabel('Temp (C)')
xlabel('Time (s)')

ax2 = subplot(212,sharex=ax1)
pltline2, = plot(x,map(None,y2))
y2 = [0.0] * window
setp( ax2.get_xticklabels(), fontsize=10)
setp( ax2.get_yticklabels(), fontsize=10)
ylabel('Audio (mV)')
xlabel('Time (s)')

try:
    while(1):
        # Read measured values
        timeout=time.time()+0.2
        while (time.time() < timeout):
            line = sys.stdin.readline()
            y1.insert(0,float(line.split()[0]))
            y1 = y1[:(len(y1)-1)]
            y2.insert(0,float(line.split()[1]))
            y2 = y2[:(len(y2)-1)]
        pltline1.set_ydata(array(y1))
        pltline2.set_ydata(array(y2))
        # Update plot
        draw()

except:
    print "End"
