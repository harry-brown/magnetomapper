#!/usr/bin/python

import socket
import time
import datetime
import struct
import StringIO
from threading import Thread
import sys
import numpy as np
import math
import json
import os
import csv
import matplotlib.pyplot as plt
import csv
from scipy.stats.kde import gaussian_kde

# implements a 1D monte carlo localization class,
# each particle is a value pair consisting of a
# position and a velocity along the line
class monte_carlo_localizer:

    # initialises the localizer, stores the map and the number of particles
    # as well as generates the initial random distribution
    def __init__(self, num_particles, env_map):

        self.npart = num_particles

        self.map = env_map
        self.map_length = env_map.shape[0]

        # generate initial random distribution of particles
        self.x_t = np.zeros((self.npart, 2))
        self.w = np.ones((self.npart, 1))

        # generate random starting location
        self.x_t[:, 0] = np.random.rand(self.npart) * (self.map_length-1)
        # generate random heading
        self.x_t[:, 1] = (np.random.rand(self.npart) * 6) - 3

    # updates the state of each particle based on it's state, essentialy moves
    # the particle position by it's velocity value
    def motion_update(self, speed):

        self.x_t[:, 0] = self.x_t[:, 0] + self.x_t[:, 1]

    # updates the weights of each particle, weight is inversely proportional to 
    # the difference between the measurement and the map value at the particle's
    # current position
    def sensor_update(self, measurement):

        # get all invalid indices
        ind1 = np.where(self.x_t[:, 0] < 0) 
        ind2 = np.where(self.x_t[:, 0] > self.map_length-1)   
        invalid = np.union1d(ind1[0],ind2[0])    

        # discard by setting weight to 0
        self.w[invalid] = 0

        # get all valid particles
        valid = np.setdiff1d(range(self.npart),invalid)

        x = np.rint(self.x_t[valid,0].ravel())
        map_vals = self.map[x.astype(int),:]

        # calculate weights
        weights_x = 1/(0.00001 + abs(measurement[0] - map_vals[:,0])**3)
        weights_y = 1/(0.00001 + abs(measurement[1] - map_vals[:,1])**3)
        weights_z = 1/(0.00001 + abs(measurement[2] - map_vals[:,2])**3)

        # update weights
        self.w[valid,0] = (weights_x + 2*weights_y + 0.2*weights_z)/3


    # Performs the resampling of the new particle set from the old particle set
    def resample_particles(self):

        # initialise the new sample array
        new_x_t = np.zeros((self.npart, 2))

        # normalise weights
        w_total = sum(self.w)
        self.w = self.w / w_total

        # get all non-zero weights and matching indices
        indices = np.where(self.w[:,0] != 0)
        indices = indices[0]
        w_pick = self.w[indices,0]

        # randomly resample from old particles based on probability
        index = np.random.choice(indices, size=int(self.npart*0.9), p=w_pick)
        new_x_t[range(int(self.npart*0.9))] = self.x_t[index]

        # add some noise to the newly sampled particles
        new_x_t[range(int(self.npart*0.9)), 0] = new_x_t[range(int(self.npart*0.9)), 0] + (np.random.rand(int(self.npart*0.9)) * 2) - 1
        new_x_t[range(int(self.npart*0.9)), 1] = new_x_t[range(int(self.npart*0.9)), 1] + (np.random.rand(int(self.npart*0.9)) * 2) - 1

        #always regenerate some particles entirely randomly to prevent particle deprivation

        # generate random starting location
        new_x_t[int(self.npart*0.9):self.npart-1, 0] = np.random.rand(int(self.npart*0.1)-1) * (self.map_length-1)
        # generate random heading
        new_x_t[int(self.npart*0.9):self.npart-1, 1] = (np.random.rand(int(self.npart*0.1)-1) * 6) - 3

        # store new particles
        self.x_t = new_x_t


    # updates the distribution based on the motion model and the weight function
    def update(self, measurement, speed):

        # perform the update steps

        # project particles in time
        self.motion_update(speed)

        # calculate weights based on sensor measurement
        self.sensor_update(measurement)

        # resample particles
        self.resample_particles()

        

UDP_TIMESYNC_PORT = 4003 # node sends timesync packets on port 4003
UDP_REPLY_PORT = 7005 # node listens for reply packets on port 7005

# TAG_ADDR = "aaaa::212:4b00:c67:4803"
TAG_ADDR = "aaaa::212:4b00:7b5:1d03"

isRunning = True
headers = False
waiting = False
mag_vals = np.array([0,0,0])

# Initialise Ports #############################################################

sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM, 0)
sock.bind(('', UDP_REPLY_PORT)) #Bind to the receiving port so messages come from here
print "Sending timesync packets on UDP port", UDP_TIMESYNC_PORT

recvSocket = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM, 0)
print "Listening for incoming packets on UDP port", UDP_REPLY_PORT
print "Exit application by pressing (CTRL-C)"

# Define UDP Threads ###########################################################

def udpListenThread():

  # listen on UDP socket port UDP_REPLY_PORT

  global headers
  global waiting
  global mag_vals

  while isRunning:
    
    try:
      rawData, addr = sock.recvfrom( 1024 )
      print "received"
      data = json.loads(rawData)

      waiting = False

      if (data['mx'] < -500) or (data['my'] < -500) or (data['mz'] < -500):
        raise ValueError

      mag_vals = np.array([data['mx'],data['my'],data['mz']])
      
      try:
          csvFile = open(filepath, mode='ab')
          csvWriter = csv.writer(csvFile)
          # if there are no headers written yet
          if not headers:
              csvWriter.writerow(data.keys())
              headers = True

          csvWriter.writerow(data.values())
          csvFile.close()
      except Exception, e:
          print "Error writing data: " + str(e) + "\r\n"
      
    except socket.timeout:
      pass
    except ValueError:
      pass
    
def udpSendThread():

  global waiting
  global mag_vals

  # map of hallway
  hall_map = np.genfromtxt ('1d_map.csv', delimiter=",")
  
  print('Map')
  print(hall_map)
  print(' ')

  # particle filter initialise
  num_particles = 2000
  speed = 1
  mcl = monte_carlo_localizer(num_particles, hall_map)
  
  plt.scatter(mcl.x_t[:,0], range(mcl.npart), s=1, c='k', marker='o', label="particles")
  plt.xlabel('x [m]')
  plt.ylabel('y [m]')
  plt.ylim([-10, num_particles])
  plt.xlim([-1, mcl.map_length])
  plt.pause(1)

  while True:
    
    sock.sendto("poll", (TAG_ADDR, UDP_TIMESYNC_PORT))
    waiting = True

    current = time.time()
    timeout = False;
    while waiting:
      if time.time() - current > 1:
        timeout = True
        break
      #do nothing

    if timeout:
      continue

    # update and get estimate
    mcl.update(mag_vals,speed)
    kde = gaussian_kde( np.transpose(mcl.x_t[:, 0]) )
    pdf = kde(range(mcl.map_length))
    inds = np.argmax(pdf)
    est = np.mean(inds)

    print('Estimate')
    print(est)

    # plot the results
    plt.scatter(mcl.x_t[:,0], range(mcl.npart), s=1, c='k', marker='o', label="particles")
    plt.scatter(est, 0, s=100, c='red', marker='o', label="estimated location")
    plt.plot(range(mcl.map_length),pdf*mcl.npart*10, label='kde')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.ylim([-1, mcl.npart])
    plt.xlim([-1, mcl.map_length])
    plt.pause(0.5)
      

# Start Application ############################################################

print "Starting application..."

time.sleep(1)

filepath = './Sensortag' + str(int(time.time())) + '.csv'
if os.path.isfile(filepath):
    # file already exists, append to it
    csvFile = open(filepath, mode='ab')
    headers = True
else:
    # file does not exist, need to create and write a header row
    csvFile = open(filepath, mode='wb')
    headers = False
csvWriter = csv.writer(csvFile)
csvFile.close()

# start UDP listener as a thread
t1 = Thread(target=udpListenThread)
t1.start()

time.sleep(1)

# start UDP timesync sender as a thread
t2 = Thread(target=udpSendThread)
t2.start()

try:
  while True:
    # wait for application to finish (ctrl-c)
    time.sleep(1)
except KeyboardInterrupt:
  print "Keyboard interrupt received. Exiting."
  isRunning = False
  try:
    csvFile.close()
  except Exception, e:
    pass
