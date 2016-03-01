"""
Created on Thu Dec 12 08:38:21 2013
 
@author: Sukhbinder Singh
 
Simple QTpy and MatplotLib example with Zoom/Pan
 
Built on the example provided at
How to embed matplotib in pyqt - for Dummies
http://stackoverflow.com/questions/12459811/how-to-embed-matplotib-in-pyqt-for-dummies
 
"""
import sys
from PyQt4 import QtGui
 
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as NavigationToolbar
import matplotlib.pyplot as plt
 
import random


import time
import os
from time import gmtime, strftime
import threading
import platform
import signal
import timeit

import serial

import sys
import pdb

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button

 
 # initialize plots
numplots = 2
fig = plt.figure()
axes = [plt.subplot(201 + 10 * np.ceil(numplots/2.0) + x) for x in range(0, numplots)]

# data sets
data_size = 1000 # the max number of points to plot at one time
xdata = np.linspace(0, 9.99, num = data_size)
ydata0 = np.zeros(data_size) - 1
ydata1 = np.zeros(data_size) - 1

xdata = [0]
ydata0 = [0]
ydata1 = [0]

# set axis limits
lines = [axis.plot(ydata0, '-')[0] for axis in axes]
for axis in axes:
    axis.set_ylim(-2.0,2.0)

xmin = 0 
update_rate = 30


# initial state for blitting
def init():
    for j in range(0,numplots):
        lines[j].set_ydata([])
        lines[j].set_xdata([])
        axes[j].set_xlim(xmin, xmin + 10)
    return tuple([line for line in lines])

def update(data):
    global xdata 
    global ydata0
    global ydata1

    # these should be the only lines needed for live update
    lines[0].set_xdata(xdata)
    lines[1].set_xdata(xdata)
    lines[0].set_ydata(ydata0)
    lines[1].set_ydata(ydata1)

    axes[0].set_xlim(0,10)
    axes[1].set_xlim(0,10)
    xmax = max(xdata)
    axes[0].set_xlim(xmax-10, xmax) # plot last ten seconds
    axes[1].set_xlim(xmax-10, xmax) # plot last ten seconds
    return tuple([line for line in lines])


def main():
    global device
    global reading  # while this is true, thread_read is still alive
    global continue_reading  # If this is true, then the thread_read thread will not exit

    reading = False 
    device = serial_initialize(port, baudrate, timeout_=2, write_timeout_=2)

    ani = animation.FuncAnimation(fig, update, init_func = init, interval=update_rate, blit=blit)
    plt.show()

    continue_reading = False  # tell thread_read to exit
    while reading: pass  # wait until thread_read has finished
    device.close()
    print 'Exit Succesful'

# if __name__ == '__main__': main()


class Window(QtGui.QDialog):
    def __init__(self, parent=None):
        super(Window, self).__init__(parent)
 
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
 
         
        self.toolbar = NavigationToolbar(self.canvas, self)
        self.toolbar.hide()

 
 
        # set the layout
        layout = QtGui.QVBoxLayout()
        self.setLayout(layout)
 
    def home(self):
        self.toolbar.home()
    def zoom(self):
        self.toolbar.zoom()
    def pan(self):
        self.toolbar.pan()
         
    def plot(self):
        ''' plot some random stuff '''
        data = [random.random() for i in range(25)]
        ax = self.figure.add_subplot(111)
        ax.hold(False)
        ax.plot(data, '*-')
        self.canvas.draw()
 
if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
 
    main = Window()
    main.setWindowTitle('Simple QTpy and MatplotLib example with Zoom/Pan')
    main.show()
 
    sys.exit(app.exec_())