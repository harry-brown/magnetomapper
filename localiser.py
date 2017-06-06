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


UDP_TIMESYNC_PORT = 4003 # node sends timesync packets on port 4003
UDP_REPLY_PORT = 7005 # node listens for reply packets on port 7005

# TAG_ADDR = "aaaa::212:4b00:c67:4803"
TAG_ADDR = "aaaa::212:4b00:7b5:1d03"

isRunning = True
headers = False
waiting = False

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

  while isRunning:
    
    try:
      rawData, addr = sock.recvfrom( 1024 )
      print "received"
      data = json.loads(rawData)

      waiting = False

      if (data['mx'] < -500) or (data['my'] < -500) or (data['mz'] < -500):
        raise ValueError

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
  
  while isRunning:
    if not waiting:
      print "polling"
      sock.sendto("poll", (TAG_ADDR, UDP_TIMESYNC_PORT))
      waiting = True

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
