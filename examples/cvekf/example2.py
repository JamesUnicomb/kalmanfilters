import numpy as np
import matplotlib.pyplot as plt

import kalmanfilters

kf = kalmanfilters.ConstantVelocityExtendedKalmanFilter(20.0)

microsprev = 0.0

t = []
acc = []
accp = []
accpunc = []

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

        accel = kalmanfilters.sensors.accel(ax, ay, az)
        gyro = kalmanfilters.sensors.gyro(gx, gy, gz)

        dt = (micros - microsprev) * 1e-6
        microsprev = micros

        # run kf step
        kf.predict(dt)
        kf.update(accel, 1.0)
        
        t.append(micros)
        acc.append([ax,ay,az])
        accp.append([ax - kf.innovation[0], ay - kf.innovation[1], az - kf.innovation[2]])
        jac = kf.jac
        s = np.dot(jac, np.dot(kf.state_unc, np.transpose(jac)))
        accpunc.append(s)

        kf.update(gyro, 0.25)

        R = [
            [1.0, 0.0, np.sin(kf.state[1])],
            [0.0, np.cos(kf.state[0]), np.sin(kf.state[0]) * np.cos(kf.state[1])],
            [0.0, -np.sin(kf.state[0]), np.cos(kf.state[0]) * np.cos(kf.state[1])]
        ]

        dv.append(np.dot(np.linalg.inv(R), [gx, gy, gz]).tolist())
        dvp.append([kf.state[2], kf.state[3], kf.state[4]])
        dvpunc.append(np.dot(np.linalg.inv(R), np.dot([s[2:] for s in kf.state_unc[2:]], np.linalg.inv(R).T)).tolist())
        
        print('state: \n', kf.state)
        print('state_unc: \n', kf.state_unc)

acc = np.array(acc)
accp = np.array(accp)
accpunc = np.array(accpunc)

dv = np.array(dv)
dvp = np.array(dvp)
dvpunc = np.array(dvpunc)

fig, ax = plt.subplots(4,2)

for i in range(3):
    ax[i,0].scatter(t, acc[:,i], s=0.2)
    ax[i,0].plot(t, accp[:,i])
    ax[i,0].fill_between(
        t,
        accp[:,i] - 2.0 * np.sqrt(accpunc[:,i,i]),
        accp[:,i] + 2.0 * np.sqrt(accpunc[:,i,i]),
        alpha=0.2,
        color='C0'
    )
    ax[i,0].set_ylim(-12.0,12.0)

    ax[i,1].scatter(t, dv[:,i], s=0.2)
    ax[i,1].plot(t, dvp[:,i])
    ax[i,1].fill_between(
        t,
        dvp[:,i] - 2.0 * np.sqrt(dvpunc[:,i,i]),
        dvp[:,i] + 2.0 * np.sqrt(dvpunc[:,i,i]),
        alpha=0.2,
        color='C0'
    )
    ax[i,1].set_ylim(-3.50,3.50)

ax[3,0].plot(t, np.linalg.norm(acc - accp, axis=1))
ax[3,0].fill_between(
    t, 
    np.zeros_like(t),
    2.0 * np.sqrt(np.square(accpunc[:,0,0]) + np.square(accpunc[:,1,1]) + np.square(accpunc[:,2,2])),
    alpha=0.2,
    color='C0')

ax[3,1].plot(t, np.linalg.norm(dv - dvp, axis=1))
# ax[3,1].fill_between(
#     t, 
#     np.zeros_like(t),
#     2.0 * np.sqrt(np.square(dvpunc[:,0,0]) + np.square(dvpunc[:,1,1]) + np.square(dvpunc[:,2,2])),
#     alpha=0.2,
#     color='C0')

plt.show()