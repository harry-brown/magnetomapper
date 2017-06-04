import time
import serial

s = serial.Serial('COM3', 115200)

print "serial port opened"

try:
    while True:
        time.sleep(1)
        
        s.write("\npoll\x0A")

        data = s.readline()

        print data

except KeyboardInterrupt:
    s.close()
    print "serial port closed"
