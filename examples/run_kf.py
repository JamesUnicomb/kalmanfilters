import serial
from serial.serialutil import SerialException
import matplotlib.pyplot as plt
import numpy as np

import kalmanfilters


ser = serial.Serial("/dev/tty.usbmodem205E3072594D1", 9600)
ser.flushInput()

kf = kalmanfilters.cvekf(25.0)

microsprev = 0.0

while True:
    try:
        ser_bytes = ser.readline()
        decoded_bytes = ser_bytes.decode("utf-8")

        sensor, data = decoded_bytes.rstrip().split(":")
        micros, x, y, z = data.split(",")

        micros = int(micros)
        x = float(x)
        y = float(y)
        z = float(z)

        if sensor == "accl":
            dt = (micros - microsprev) * 1e-6
            microsprev = micros

            accel = kalmanfilters.sensors.accel(x, y, z, 0.25, 0.25, 0.25)

            # run kf step
            kf.predict(dt)
            kf.update(accel)

        elif sensor == "gyro":
            dt = (micros - microsprev) * 1e-6
            microsprev = micros

            gyro = kalmanfilters.sensors.gyro(x, y, z, 0.25, 0.25, 0.25)

            # run kf step
            kf.predict(dt)
            kf.update(gyro)

        elif sensor == "mag":
            dt = (micros - microsprev) * 1e-6
            microsprev = micros

            mag = kalmanfilters.sensors.mag(x, y, z, 450.0, 450.0, 450.0)

            # run kf step
            kf.predict(dt)
            kf.update(mag)

        print(kf.state, end="\r")

    except (KeyboardInterrupt, SerialException) as e:
        print(e)
        break
    except Exception as e:
        print(e)
        pass
