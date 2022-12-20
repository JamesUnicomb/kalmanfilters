import numpy as np
import matplotlib.pyplot as plt

import kalmanfilters

kf = kalmanfilters.ConstantPositionExtendedKalmanFilter(10.0, 0.1)

microsprev = 0.0

t = []
acc = []
accp = []
accpunc = []
mag = []

with open("examples/data/data4.txt", "r") as f:
    for r in f.readlines()[1:]:
        micros, ax, ay, az, gx, gy, gz, mx, my, mz = r.rstrip().split(",")

        micros = int(micros)
        ax = float(ax)
        ay = float(ay)
        az = float(az)
        mx = float(mx)
        my = float(my)
        mz = float(mz)

        accel = kalmanfilters.sensors.accel(ax, ay, az)

        dt = (micros - microsprev) * 1e-6
        microsprev = micros

        # run kf step
        kf.predict(dt)
        kf.update(accel)
        
        t.append(micros)
        acc.append([ax,ay,az])
        accp.append([ax - kf.innovation[0], ay - kf.innovation[1], az - kf.innovation[2]])
        
        jac = kf.jac

        s = np.dot(jac, np.dot(kf.state_unc, np.transpose(jac)))

        Rx = [
            [1.0, 0.0, 0.0],
            [0.0, np.cos(kf.state[0]), -np.sin(kf.state[0])],
            [0.0, np.sin(kf.state[0]), np.cos(kf.state[0])]
        ]
        Ry = [
            [np.cos(kf.state[1]), 0.0, np.sin(kf.state[1])],
            [0.0, 1.0, 0.0],
            [-np.sin(kf.state[0]), 0.0, np.cos(kf.state[0])]
        ]
        R = np.dot(Rx, Ry)
        

        accpunc.append(s)
        mag.append(np.dot(np.linalg.inv(R),[mx,my,mz]).tolist())

        print(np.linalg.norm([mx,my,mz]), [mx,my,mz])

acc = np.array(acc)
accp = np.array(accp)
accpunc = np.array(accpunc)
mag = np.array(mag)

fig, ax = plt.subplots(4,2)

for i in range(3):
    ax[i,1].scatter(t, mag[:,i])
    ax[i,0].scatter(t, acc[:,i])
    ax[i,0].plot(t, accp[:,i])
    ax[i,0].fill_between(
        t,
        accp[:,i] - 2.0 * np.sqrt(accpunc[:,i,i]),
        accp[:,i] + 2.0 * np.sqrt(accpunc[:,i,i]),
        alpha=0.2,
        color='C0'
    )

ax[3,1].plot(t,np.linalg.norm(mag, axis=1))

plt.show()