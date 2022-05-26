import time
import struct
import board
import busio
import adafruit_bno055
from adafruit_motor import motor
from adafruit_motorkit import MotorKit
import serial
import serial.tools.list_ports

PWM_FREQ = 25
DECAY_MODE = motor.SLOW_DECAY

kit = MotorKit()
kit.motor1.decay_mode = DECAY_MODE
kit.motor2.decay_mode = DECAY_MODE

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

ser = serial.Serial('/dev/rfcomm0')

cont = "1"
while cont == "1":
    orientation = None
    path = []
    while True:
        time.sleep(0.5)
        #print("Reading...")
        data = ser.read(size=1)
        #print(data)
        if orientation == None:
            if data == b'U':
                orientation = "U"
            elif data == b'D':
                orientation = "D"
            elif data == b'L':
                orientation = "L"
            elif data == b'R':
                orientation = "R"
        elif data == b'F':
            break
        else:
            if data == b'U':
                path.append("U")
            elif data == b'D':
                path.append("D")
            elif data == b'L':
                path.append("L")
            elif data == b'R':
                path.append("R")
        #ser.flushInput()
        #ser.flushOutput()
        ser.write(b'F')
    ser.write(b'F')


    #orientation = input("Enter initial orientation as a single character(e.x. 'D' for down): ")
    #while True:
    #    nextPath = input("Enter the next direction or 'F' if finished: ")
    #    if nextPath == "F":
    #        break
    #    else:
    #        path.append(nextPath)

    try:
        straight = 1
        for index, direction in enumerate(path):
            # If not facing correct direction then rotate
            if orientation != path[index]:
                if (orientation == "D" and path[index] == "U") or (orientation == "U" and path[index] == "D") or (orientation == "R" and path[index] == "L") or (orientation == "L" and path[index] == "R"):
                    turn = 3
                elif (orientation == "D" and path[index] == "L") or (orientation == "L" and path[index] == "U") or (orientation == "U" and path[index] == "R") or (orientation == "R" and path[index] == "D"):
                    turn = -1.35
                else:
                    turn = 1.35
            
                kit.motor1.throttle = 0
                kit.motor2.throttle = 0
                time.sleep(0.5)
                start = time.time()
                if turn == -1.35:
                    kit.motor1.throttle = -0.25
                    kit.motor2.throttle = -0.25
                else:
                    kit.motor1.throttle = 0.25
                    kit.motor2.throttle = 0.25
                time.sleep(0.05)

                radians = 0.0

                while abs(radians) < abs(turn): #90 degress = 1.5708 radians
                    #time.sleep(0.1)
                    end = time.time()
                    timeElapse = end - start
                    start = time.time()
                    rps = sensor.gyro[2]
                    #print(f"X: {sensor.gyro[0]}  Y: {sensor.gyro[1]}  Z: {sensor.gyro[2]}")
                    newRad = abs(rps) * timeElapse
                    radians = radians + newRad
                    #print(radians)
                kit.motor1.throttle = 0
                kit.motor2.throttle = 0
                
            time.sleep(0.5)
            #ser.flushInput()
            #ser.flushOutput()
            #print("Write gyro ACK")
            ser.write(b'F')
            time.sleep(0.5)
            #print("Reading post gyro pic ACK")
            data = ser.read()

            orientation = path[index]
            #if index != len(path)-1:
            #    if path[index] == path[index+1]:
            #        straight+=1
            #        continue
            #    else:
            #        straight = 1
            # Go straight for ~8 inches
            kit.motor1.throttle = 0
            kit.motor2.throttle = 0
            time.sleep(0.5)
            start = time.time()
            kit.motor1.throttle = -0.25
            kit.motor2.throttle = 0.25
            velocity = 0
            totalDistance = 0
            #start = time.time()
            while totalDistance < (0.2):  #0.2302m = 8in
                time.sleep(0.05)
                newAccel = sensor.linear_acceleration[1]
                if newAccel == None:
                    newAccel = 0
                newAccel = -1 * newAccel + 0.04
                if newAccel < 0:
                    newAccel = 0
                end = time.time()
                timeElapse = end-start
                start = time.time()
                velocity = velocity + newAccel*timeElapse
                distance = velocity*timeElapse + 0.5*newAccel*(timeElapse*timeElapse)
                totalDistance = totalDistance + distance
                #print(f"Inital Move Total Distance: {totalDistance}")
            kit.motor1.throttle = 0
            kit.motor2.throttle = 0
            time.sleep(1)
            #ser.flushInput()
            #ser.flushOutput()
            #print("Writing Move ACK")
            ser.write(b'F')
            #print("Reading H/L")
            time.sleep(0.5)
            pos = ser.read()
            while pos != b'F':
                #ser.flushInput()
                #ser.flushOutput()
                #print("Writing H/L ACK")
                ser.write(b'F')
                #print("Reading Offset")
                time.sleep(0.5)
                data = ser.read(size=4)
                #print("Writing Offset ACK")
                ser.write(b'F')
                ba = struct.unpack('f',data)
                #print(f"Offset is {ba[0]} cm")
                offset = (ba[0] / 100) #cm to m
                start = time.time()
                if pos == b'L':
                    kit.motor1.throttle = -0.25
                    kit.motor2.throttle = 0.25
                elif pos == b'H':
                    kit.motor1.throttle = 0.25
                    kit.motor2.throttle = -0.25
                velocity = 0
                totalDistance = 0
                while totalDistance < (offset*0.8):  #0.2302m = 8in
                    time.sleep(0.05)
                    newAccel = sensor.linear_acceleration[1]
                    if newAccel == None:
                        newAccel = 0
                    if pos == b'L':
                        newAccel = newAccel * -1
                        newAccel = newAccel + 0.05
                    #else:
                        #newAccel = newAccel - 0.05
                    if newAccel < 0:
                        newAccel = 0
                    end = time.time()
                    timeElapse = end-start
                    start = time.time()
                    velocity = velocity + newAccel*timeElapse
                    distance = velocity*timeElapse + 0.5*newAccel*(timeElapse*timeElapse)
                    totalDistance = totalDistance + distance
                    #print(f"Offset Move Total Distance: {totalDistance}")
                kit.motor1.throttle = 0
                kit.motor2.throttle = 0
                #print("Writing Offset Movement ACK")
                time.sleep(0.5)
                ser.write(b'F')
                #print("Reading next H/L")
                time.sleep(0.5)
                pos = ser.read()
                #pos = b'F'
            #ser.write(b'F')
    except (TypeError,KeyboardInterrupt):
        kit.motor1.throttle = 0
        kit.motor2.throttle = 0
        print("Interrupted")
        
    cont = input("Go Again? 1 for yes and 0 for no: ")
    
ser.close()