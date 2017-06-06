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


UDP_TIMESYNC_PORT = 4003 # node sends timesync packets on port 4003
UDP_REPLY_PORT = 7005 # node listens for reply packets on port 7005

# TAG_ADDR = "aaaa::212:4b00:c67:4803"
TAG_ADDR = "aaaa::212:4b00:7b5:1d03"

isRunning = True

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

  while isRunning:
    
    try:
      data, addr = sock.recvfrom( 1024 )
      print data
    except socket.timeout:
      pass
    
def udpSendThread():

  while isRunning:
    sock.sendto("poll", (TAG_ADDR, UDP_TIMESYNC_PORT))
    time.sleep(1)

# Start Application ############################################################

print "Starting application..."

time.sleep(1)

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