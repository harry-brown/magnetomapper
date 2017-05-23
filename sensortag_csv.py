import json
import time
import csv
import os.path
import serial

WAIT = 0.1 # wait time in seconds between each poll
NSAMPLES = 5 # number of samples to poll before averaging
COM = 'COM12' # COM port of sensortag


if __name__ == "__main__":
    s = serial.Serial(port=COM, baudrate=115200)

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

    try:
        while True: # program loop
            posInput = raw_input("Please enter comma seperated x and y position: \r\n")
            x, y = posInput.split(',')
            x = int(x)
            y = int(y)
            dataPoints = []
            # print x, y
            print "Reading sensortag...\r\n"
            for _ in range(NSAMPLES): # polling loop
                s.write("\npoll\x0A")
                try:
                    rawData = s.readline()
                    # print rawData
                except Exception, e:
                    print "Error reading data: " + e + "\r\n"

                data = json.loads(rawData)
                data['posx'] = x
                data['posy'] = y
                dataPoints.append(data)
                time.sleep(WAIT)

            # Average the data
            data = {k: sum(dPoint[k] for dPoint in dataPoints)/len(dataPoints) for k in dataPoints[0]}

            # data has been read and averaged, now write in csv
            try:
                # if there are no headers written yet
                if not headers:
                    csvWriter.writerow(data.keys())
                    headers = True

                csvWriter.writerow(data.values())
            except Exception, e:
                print "Error writing data: " + e + "\r\n"
    except KeyboardInterrupt:
        s.close()
        print "Serial port closed"
        csvFile.close()
            

