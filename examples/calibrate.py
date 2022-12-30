import numpy as np
import matplotlib.pyplot as plt

"""
calibration script for magnetometer
    1. change values in arduino script to zero.
    2. run read_serial.py and save data as calibrate.
    3. run this script to find new values to put into 
       arduino script.
"""

mag = []

with open("examples/calibrate.txt", "r") as f:
    for r in f.readlines()[1:]:
        sensor, data = r.rstrip().split(":")
        micros, x, y, z = data.split(",")

        micros = int(micros)
        x = float(x)
        y = float(y)
        z = float(z)

        if sensor == "mag":
            mag.append([x, y, z])

fig, ax = plt.subplots(1, 1)
ax.set_aspect(1)

ax.scatter([mx for mx, my, mz in mag], [my for mx, my, mz in mag])
ax.scatter([mx for mx, my, mz in mag], [mz for mx, my, mz in mag])
ax.scatter([my for mx, my, mz in mag], [mz for mx, my, mz in mag])

minx = np.min([mx for mx, my, mz in mag])
miny = np.min([my for mx, my, mz in mag])
minz = np.min([mz for mx, my, mz in mag])

maxx = np.max([mx for mx, my, mz in mag])
maxy = np.max([my for mx, my, mz in mag])
maxz = np.max([mz for mx, my, mz in mag])

cx = (maxx + minx) / 2.0
cy = (maxy + miny) / 2.0
cz = (maxz + minz) / 2.0

print(cx,cy,cz)

plt.show()

plt.plot(np.linalg.norm(mag, axis=1))
plt.show()

fig, ax = plt.subplots(1, 1)
ax.set_aspect(1)
ax.scatter([mx - cx for mx, my, mz in mag], [my - cy for mx, my, mz in mag])
ax.scatter([mx - cx for mx, my, mz in mag], [mz - cz for mx, my, mz in mag])
ax.scatter([my - cy for mx, my, mz in mag], [mz - cz for mx, my, mz in mag])

plt.show()

plt.plot(np.linalg.norm([[mx - cx, my - cy, mz - cz] for mx, my, mz in mag], axis=1))
plt.plot(np.arange(len(mag)), np.mean(np.linalg.norm([[mx - cx, my - cy, mz - cz] for mx, my, mz in mag], axis=1)) * np.ones(len(mag)))
plt.show()