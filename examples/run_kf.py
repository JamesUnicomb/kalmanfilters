import serial
from serial.serialutil import SerialException
import kalmanfilters

ser = serial.Serial('/dev/tty.usbmodem205E3072594D1', 9600)
ser.flushInput()

kf = kalmanfilters.ConstantStateExtendedKalmanFilter(0.025, 0.05)

microsprev = 0.0

while True:
    try:
        ser_bytes = ser.readline()
        decoded_bytes = ser_bytes.decode("utf-8")
        
        micros, ax, ay, az, gx, gy, gz, mx, my, mz = decoded_bytes.rstrip().split(",")
        
        micros = int(micros)
        ax = float(ax)
        ay = float(ay)
        az = float(az)

        dt = (micros - microsprev) * 1e-6
        microsprev = micros

        # run kf step
        kf.predict(dt)
        kf.update([ax, ay, az])

        print(kf.state, kf.state_unc, end='\r')

    except (KeyboardInterrupt, SerialException) as e:
        print(e)
        break
    except Exception as e:
        print(e)
        pass
