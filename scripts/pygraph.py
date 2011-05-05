#!/usr/bin/env python
"""
Example: simple line plot.
Show how to make and save a simple line plot with labels, title and grid
"""
import numpy
import pylab
import wave
import sys
import struct

if (len(sys.argv) < 2):
    print "Data file needed\n"
    sys.exit()

f = open(sys.argv[1])

if (len(sys.argv) == 3):
    fs = int(sys.argv[2])
    print "Samplerate: " + str(fs)
else:
    fs = 0

graphmode = ''

s=[[],[],[],[]]

line = f.readline()
line = line.split(' ')
chs = 1
c = 0

while (line != []):
    c = c + 1
    for i in range(0,chs,1):
        line = f.readline().split()
        #print "line:" + str(line)
        if (line != []):
            #print "->" + str(line[0])
            s[i].append(float(line[0]))

t=[[],[],[],[]]
#print "fs:" + str(fs) + " l:" + str(len(s[i])) + "  c:" + str(c)

for i in range(0,chs,1):
    if (fs == 0):
        pylab.xlabel('samples')
        t[i] = numpy.arange(0.0, len(s[i]), 1)
    else:
        pylab.xlabel('time (ms)')
        t[i] = numpy.arange(0.0, float(len(s[i]))/fs, float(1.0/fs))


for i in range(0,chs,1):
    pylab.plot(t[i],s[i],graphmode)

pylab.ylabel('voltage (mV)')
pylab.title('Graph')
pylab.grid(True)
pylab.savefig('simple_plot') 
pylab.show()

