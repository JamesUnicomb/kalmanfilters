import numpy as np
import matplotlib.pyplot as plt

import kalmanfilters

kf = kalmanfilters.ConstantPositionExtendedKalmanFilter(2.5, 1.0)

microsprev = 0.0

t = []
acc = []
accp = []
accpunc = []

with open("examples/data/data1.txt", "r") as f:
    for r in f.readlines()[1:]:
        sensor, data = r.rstrip().split(":")
        micros, x, y, z = data.split(",")

        micros = int(micros)
        x = float(x)
        y = float(y)
        z = float(z)

        if sensor == "accl":
            accel = kalmanfilters.sensors.accel(x, y, z)

            dt = (micros - microsprev) * 1e-6
            microsprev = micros

            # run kf step
            kf.predict(dt)
            kf.update(accel)
            
            t.append(micros)
            acc.append([x,y,z])
            accp.append([x - kf.innovation[0], y - kf.innovation[1], z - kf.innovation[2]])
            
            jac = kf.jac

            s = np.dot(jac, np.dot(kf.state_unc, np.transpose(jac)))

            accpunc.append(s)

            print('state: \n', kf.state)
            print('state_unc: \n', kf.state_unc)

acc = np.array(acc)
accp = np.array(accp)
accpunc = np.array(accpunc)

fig, ax = plt.subplots(3,1)

for i in range(3):
    ax[i].scatter(t, acc[:,i])
    ax[i].plot(t, accp[:,i])
    ax[i].fill_between(
        t,
        accp[:,i] - 2.0 * np.sqrt(accpunc[:,i,i]),
        accp[:,i] + 2.0 * np.sqrt(accpunc[:,i,i]),
        alpha=0.2,
        color='C0'
    )

plt.show()