import serial

ser = serial.Serial("/dev/tty.usbmodem205E3072594D1", 115200)
ser.flushInput()

with open("examples/data.txt", "w") as f:
    f.write("sensor,micros,x,y,x\n")

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
