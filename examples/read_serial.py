import serial

ser = serial.Serial('/dev/tty.usbmodem205E3072594D1', 9600)
ser.flushInput()

with open("examples/data.txt", "w") as f:
    f.write("milli,accel_x,accel_y,accel_x,gyro_x,gyro_y,gyro_z\n")

while True:
    try:
        ser_bytes = ser.readline()
        decoded_bytes = ser_bytes.decode("utf-8")
        with open("examples/data.txt", "a") as f:
            f.writelines(decoded_bytes)
        print(decoded_bytes.rstrip())
    except KeyboardInterrupt as e:
        break
    except Exception as e:
        print(e)
        pass
