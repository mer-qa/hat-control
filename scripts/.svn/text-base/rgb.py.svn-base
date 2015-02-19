import sys
import numpy
import matplotlib.pyplot as plt

plt.ion()
dataLine = sys.stdin.readline()

while True:
    dataLine = sys.stdin.readline()
    dataList = dataLine.split()
    print dataList
    if len( dataList ) > 0:
        data = map( lambda x: float(x), dataList )
        #print data
        data[2] = data[2] * 1.5 * 2.1
        data[1] = data[1] * 1   * 2.1
        data[0] = data[0] * 0.8 * 2.1
        if data[0] < 0 :
            data[0] = 0
        if data[1] < 0 :
            data[1] = 0
        if data[2] < 0 :
            data[2] = 0

        #print data
        bgcolor = map( lambda x: x/255.0, data )
        
        labels = ["Red", "Green", "Blue" ]
        xlocations = numpy.array([ 0, 1, 2 ])
        plt.subplot( 111, axisbg=bgcolor )
        plt.plot(xlocations,map(None,data), "wD" )
        plt.xticks(xlocations, labels)
        plt.title("OptoFidelity HAT Color Sensor")
        plt.ylim(0,256)

        plt.draw()

