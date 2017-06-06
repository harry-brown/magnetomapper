import json
import time
import csv
import os.path
import socket
import struct
import serial



SERIAL = False

WAIT = 1 # wait time in seconds between each poll
NSAMPLES = 4 # number of samples to poll before averaging

# Serial Settings
COM = 'COM5' # COM port of sensortag


# UDP Settings
SEND_PORT = 4003
RECV_PORT = 7005

#TAG_ADDR = "aaaa::212:4b00:c67:4803"
TAG_ADDR = "aaaa::212:4b00:7b5:1d03"

def serialPoll(s):
    s.write("\npoll\x0A")
    try:
        rawData = s.readline()
        # print rawData
    except Exception, e:
        print "Error reading data: " + e + "\r\n"

    return rawData

def udpPoll(sock):
    sock.sendto("poll", (TAG_ADDR, SEND_PORT))
    try:
        rawData, addr = sock.recvfrom(1024)
        print rawData
    except Exception, e:
        print "Error reading data: " + e + "\r\n"

    return rawData

if __name__ == "__main__":
    filepath = './Sensortag.csv'
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

    # setup comms interface
    if SERIAL:
        s = serial.Serial(port=COM, baudrate=115200)
    else:
        sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM, 0)
        sock.bind(('', RECV_PORT))

    while True: # program loop
        posInput = raw_input("Please enter comma seperated x and y position: \r\n")
        x, y = posInput.split(',')
        x = int(x)
        y = int(y)
        dataPoints = []
        # print x, y
        print "Reading sensortag...\r\n"
        for _ in range(NSAMPLES): # polling loop
            if SERIAL:
                rawData = serialPoll(s)
            else: # udp
                rawData = udpPoll(sock)
            data = json.loads(rawData)
            data['posx'] = x
            data['posy'] = y
            dataPoints.append(data)
            time.sleep(WAIT)

        # Average the data
        data = {k: sum(dPoint[k] for dPoint in dataPoints)/len(dataPoints) for k in dataPoints[0]}

        # data has been read and averaged, now write in csv
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
