import serial
import numpy as np
import matplotlib.pyplot as plt

import kalmanfilters

kf = kalmanfilters.ConstantStateExtendedKalmanFilter(0.5, 0.5)

microsprev = 0.0

t = []
acc = []
accp = []
accpunc = []

with open("examples/data.txt", "r") as f:
    for r in f.readlines()[1:]:
        micros, ax, ay, az, gx, gy, gz = r.rstrip().split(",")
        
        micros = int(micros)
        ax = float(ax)
        ay = float(ay)
        az = float(az)

        dt = (micros - microsprev) * 1e-6
        microsprev = micros

        # run kf step
        kf.predict(dt)
        kf.update([ax, ay, az])
        
        t.append(micros)
        acc.append([ax,ay,az])
        accp.append([ax - kf.innovation[0], ay - kf.innovation[1], az - kf.innovation[2]])
        
        jac = kf.jac

        s = np.dot(jac, np.dot(kf.state_unc, np.transpose(jac)))

        accpunc.append(s)

        print('state: \n', kf.state)
        print('state_unc: \n', kf.state_unc)

acc = np.array(acc)
accp = np.array(accp)
accpunc = np.array(accpunc)

for i in range(3):
    plt.subplot(311+i)
    plt.scatter(t, acc[:,i])
    plt.plot(t, accp[:,i])
    plt.fill_between(
        t,
        accp[:,i] - 2.0 * np.sqrt(accpunc[:,i,i]),
        accp[:,i] + 2.0 * np.sqrt(accpunc[:,i,i]),
        alpha=0.2,
        color='C0'
    )

plt.show()