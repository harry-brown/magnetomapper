
import time
import serial

s = serial.Serial('COM3', 115200)

print "serial port opened"

try:
    while True:
        s.write("\npoll\x0A")

        data = s.readline()

        print data

        time.sleep(1)

except KeyboardInterrupt:
    s.close()
    print "serial port closed"
