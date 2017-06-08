#!/usr/bin/python
# A script that reads in JSON format accelerometer and gyroscope data from sensor tag
# Applies a low pass filter
# Detects steps and turn events (=-90)
# Caculates 1D velocity in x direction and plots
# Prints 2D position coordinates
# Georgina Siggins

import serial
import threading
import time
import readchar
import json
import os
import random
import wx
from threading import Thread
import sys

import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg
import numpy as np
import scipy.signal
import pylab
import math


xc = 0.0
xlast = 0.0
isRunning = True

# runs when possible
# reads in sensortag data and calculating position (x, y distance and degree of direction)
def stepThread():
    s = serial.Serial(port = '/dev/ttyACM0', baudrate=115200)
    count = 1
    yc = 0.0
    difference = 0.0
    degree = 0
    xPast = -1
    zPast = 0
    yPast = 0
    global xc
    global isRunning
    while isRunning:
        try:
            data = s.readline()
            text = json.loads(data)
            X = text['X']
            Y = text['Y']
            Z = text['Z']
            # gx vertical turning axis
            GX = text['gX']
            #Low pass filter alpha = (dt/(dt+(1/RC))
            LX = xPast + 0.459*(X - xPast)
            LZ = zPast + 0.459*(Z - zPast)
            LY = yPast + 0.459*(Y - yPast)
            
            F = math.sqrt(LX**2 + LY**2) #up and forward
            #S = math.sqrt(LX**2 + LZ**2) #up and sideways

            if count == 0 and F < 1 and GX < 20 and GX > -20:
                    count = 1
            if count == 1:
                if GX > 85:
                    degree = degree + 90
                    if degree > 300:
                        degree = 0
                    print (degree)
                    count = 0
                elif GX < - 85:
                    degree = degree - 90
                    if degree < 0:
                        degree = 270
                    print (degree)
                    count = 0
                difference = F - 1
                if difference > 0.08:
                    count = 0
                    if degree == 0:
                        xc = xc + .6
                        print (xc, yc)
                    elif degree == 90:
                        yc = yc + .6
                        print (xc, yc)
                    elif degree == 180:
                        xc = xc - .6
                        print (xc, yc)
                    elif degree == 270:
                        yc = yc - .6
                        print (xc, yc)
        except Exception, e:
            print "Exception:", e
            # close serial port
            print( "close serial port")
            s.close()

def printThread():
    global xlast
    global xc
    global isRunning
    i = 0
    times = []
    vels = []
    while isRunning:
        velocity = (xc - xlast)/2
        xlast = xc
        
        #print velocity
        times.append(i)
        vels.append(velocity)
        plt.clf()
        plt.plot(times, vels, c='k')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity m/s')
        plt.pause(0.01)
        plt.ylim([-2,2])
        i = i + 1

        # sleep for 2 seconds
        time.sleep(2)

        

# start step thread as a thread
t1 = Thread(target=stepThread)
t1.start()

time.sleep(1)

# start UDP timesync sender as a thread
t2 = Thread(target=printThread)
t2.start()


try:
  while True:
    # wait for application to finish (ctrl-c)
    time.sleep(1)
except KeyboardInterrupt:
  print "Keyboard interrupt received. Exiting."
  isRunning = False