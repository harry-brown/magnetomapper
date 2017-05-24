#!/usr/bin/python 
""" 
Dynamically plots single axis (X) accelerometer data recieved over serial from sensorTag

Based on plot-data.py in the supplied contiki-examples python folder. 

TO DO:  Implement FFT with high pass filtering
        Figure out why timing is off
        Thresholding to determine step
"""

import serial
import threading
import time
import readchar
import json
import os
import random
import wx

import matplotlib
matplotlib.use('WXAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg
import numpy as np
import pylab
import math

serial.Serial(port = '/dev/ttyACM0', baudrate = 115200)

class DataSource(object):

    read_data = True

    def __init__(self, graph):
        self.graph = graph
        self.s = serial.Serial(port = '/dev/ttyACM0', baudrate = 115200)

        # start separate thread to read data
        t1 = threading.Thread(target=self.read_loop)
        t1.start()

    def close(self):
      self.read_data = False

    # read loop for data
    def read_loop(self):
        output = ''
        while self.read_data:
            try:
                data = self.s.read();
                if len(data) > 0:
                    if (data[-1] == '\n'):
                        new = output
                        output = ''
                        text = json.loads(new)
                        X = text['X']
            
                        # update plot
                        if isinstance(self.graph, wx.Frame):
                            self.graph.update_data(X) 
                            wx.CallAfter(self.graph.draw_plot)
                    else:
                        output += data

            except Exception, e:
                print "Exception:", e
        # close serial port
        print( "close serial port")
        self.s.close()

class GraphFrame(wx.Frame):
    """ The main frame of the application
    """
    title = 'X Acceleration'
    
    def __init__(self):
        wx.Frame.__init__(self, None, -1, self.title)
        
    # handle window close event    
        self.Bind(wx.EVT_CLOSE, self.on_exit)

        # set data source
        self.source = DataSource(self)
        
        self.data = []
        
        self.create_main_panel()
        

    def create_main_panel(self):

        self.panel = wx.Panel(self)

        self.init_plot()
        self.canvas = FigureCanvasWxAgg(self.panel, -1, self.fig)

        self.vbox = wx.BoxSizer(wx.VERTICAL)
        self.vbox.Add(self.canvas, 1, flag=wx.LEFT | wx.TOP | wx.GROW)        
        
        self.panel.SetSizer(self.vbox)
        self.vbox.Fit(self)
   

    def init_plot(self):
        self.fig = Figure((6.0, 3.0), dpi=100)

        self.axes = self.fig.add_subplot(111)
        self.axes.set_axis_bgcolor('black')
        self.axes.set_title('Sensor data', size=12)
        
        pylab.setp(self.axes.get_xticklabels(), fontsize=8)
        pylab.setp(self.axes.get_yticklabels(), fontsize=8)

        # plot the data as a line series, and save the reference 
        # to the plotted line series
        self.plot_data = self.axes.plot(self.data, linewidth=1, color=(1, 1, 0))[0]


    def update_data(self, sensor):
    
       self.data.append(sensor)


    def draw_plot(self):
        """ Redraws the plot
        """

        xmax = len(self.data)*0.05 if len(self.data)*0.05 > (50*0.05) else (50*0.05)
        xmin = xmax - 50*0.05

        ymax = -0.5
        ymin = -1.5
       
        self.axes.set_xbound(lower=xmin, upper=xmax)
        self.axes.set_ybound(lower=ymin, upper=ymax)
        
        self.axes.grid(True, color='gray')
        pylab.setp(self.axes.get_xticklabels(), visible=True)
       
        self.plot_data.set_xdata(np.arange(len(self.data))*0.05)
        self.plot_data.set_ydata(np.array(self.data))
        
        self.canvas.draw()

    
    def on_exit(self, event):
        self.source.close()
        self.Destroy()
    
    

if __name__ == '__main__':
    app = wx.PySimpleApp()
    app.frame = GraphFrame()
    app.frame.Show()
    app.MainLoop()

