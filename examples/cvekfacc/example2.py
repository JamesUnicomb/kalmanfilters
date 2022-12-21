import numpy as np
import matplotlib.pyplot as plt

import kalmanfilters

kf = kalmanfilters.ConstantVelocityExtendedKalmanFilterAccel(5.0)

microsprev = 0.0

t = []
acc = []
accp = []
accpunc = []

rp = []

dv = []
dvp = []
dvpunc = []

with open("examples/data/data2.txt", "r") as f:
    for r in f.readlines()[1:]:
        micros, ax, ay, az, gx, gy, gz, mx, my, mz = r.rstrip().split(",")

        micros = int(micros)
        ax = float(ax)
        ay = float(ay)
        az = float(az)
        gx = float(gx)
        gy = float(gy)
        gz = float(gz)
        mx = float(mx)
        my = float(my)
        mz = float(mz)

        accel = kalmanfilters.sensors.accel(ax, ay, az)
        gyro = kalmanfilters.sensors.gyro(gx, gy, gz)
        mag = kalmanfilters.sensors.mag(mx, my, mz)

        dt = (micros - microsprev) * 1e-6
        microsprev = micros

        # run kf step
        kf.predict(dt)
        kf.update(accel, 0.25)
        
        t.append(micros)
        acc.append([ax,ay,az])
        accp.append([ax - kf.innovation[0], ay - kf.innovation[1], az - kf.innovation[2]])
        jac = kf.jac
        s = np.dot(jac, np.dot(kf.state_unc, np.transpose(jac)))
        accpunc.append(s)

        R = [
            [1.0, 0.0, np.sin(kf.state[1])],
            [0.0, np.cos(kf.state[0]), np.sin(kf.state[0]) * np.cos(kf.state[1])],
            [0.0, -np.sin(kf.state[0]), np.cos(kf.state[0]) * np.cos(kf.state[1])]
        ]

        rp.append([kf.state[0], kf.state[1]])

        dv.append(np.dot(np.linalg.inv(R), [gx, gy, gz]).tolist())
        dvp.append([kf.state[2], kf.state[3]])
        dvpunc.append([s[2:] for s in kf.state_unc[2:]])
        
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
    ax[i].set_ylim(-14.0,14.0)
    
plt.show()

rp = np.array(rp)
dv = np.array(dv)
dvp = np.array(dvp)
dvpunc = np.array(dvpunc)

fig, ax = plt.subplots(2,1)

for i in range(2):
    ax[i].scatter(t, dv[:,i])
    ax[i].plot(t, dvp[:,i])
    ax[i].fill_between(
        t,
        dvp[:,i] - 2.0 * np.sqrt(dvpunc[:,i,i]),
        dvp[:,i] + 2.0 * np.sqrt(dvpunc[:,i,i]),
        alpha=0.2,
        color='C0'
    )
    ax[i].set_ylim(-6.0,6.0)

plt.show()