import numpy as np
import matplotlib.pyplot as plt

import kalmanfilters

kf = kalmanfilters.ConstantVelocityExtendedKalmanFilter(5.0)

microsprev = 0.0

tacc = []
acc = []
accp = []
accpunc = []

tdv = []
dv = []
dvp = []
dvpunc = []

with open("examples/data/data2.txt", "r") as f:
    for r in f.readlines()[1:]:
        sensor, data = r.rstrip().split(":")
        micros, x, y, z = data.split(",")

        micros = int(micros)
        x = float(x)
        y = float(y)
        z = float(z)

        dt = (micros - microsprev) * 1e-6
        microsprev = micros

        if sensor == "accl":
            accel = kalmanfilters.sensors.accel(x, y, z)

            # run kf step
            kf.predict(dt)
            kf.update(accel, 1.25)

            tacc.append(micros)
            acc.append([x,y,z])
            accp.append([x - kf.innovation[0], y - kf.innovation[1], z - kf.innovation[2]])
            jac = kf.jac
            s = np.dot(jac, np.dot(kf.state_unc, np.transpose(jac)))
            accpunc.append(s)

        elif sensor == "gyro":
            gyro = kalmanfilters.sensors.gyro(x, y, z)

            # run kf step
            kf.predict(dt)
            kf.update(gyro, 1.0)
            
            R = [
                [1.0, 0.0, np.sin(kf.state[1])],
                [0.0, np.cos(kf.state[0]), np.sin(kf.state[0]) * np.cos(kf.state[1])],
                [0.0, -np.sin(kf.state[0]), np.cos(kf.state[0]) * np.cos(kf.state[1])]
            ]

            tdv.append(micros)
            dv.append(np.dot(np.linalg.inv(R), [x, y, z]).tolist())
            dvp.append([kf.state[2], kf.state[3], kf.state[4]])
            jac = kf.jac

            dvpunc.append(np.dot(np.linalg.inv(R), np.dot([s[2:] for s in kf.state_unc[2:]], np.linalg.inv(R).T)).tolist())

        print('state: \n', kf.state)
        print('state_unc: \n', kf.state_unc)

acc = np.array(acc)
accp = np.array(accp)
accpunc = np.array(accpunc)

fig, ax = plt.subplots(3,1)

for i in range(3):
    ax[i].scatter(tacc, acc[:,i])
    ax[i].plot(tacc, accp[:,i])
    ax[i].fill_between(
        tacc,
        accp[:,i] - 2.0 * np.sqrt(accpunc[:,i,i]),
        accp[:,i] + 2.0 * np.sqrt(accpunc[:,i,i]),
        alpha=0.2,
        color='C0'
    )
    ax[i].set_ylim(-12.0,12.0)

plt.show()


dv = np.array(dv)
dvp = np.array(dvp)
dvpunc = np.array(dvpunc)

fig, ax = plt.subplots(3,1)

for i in range(3):
    ax[i].scatter(tdv, dv[:,i])
    ax[i].plot(tdv, dvp[:,i])
    ax[i].fill_between(
        tdv,
        dvp[:,i] - 2.0 * np.sqrt(dvpunc[:,i,i]),
        dvp[:,i] + 2.0 * np.sqrt(dvpunc[:,i,i]),
        alpha=0.2,
        color='C0'
    )
    ax[i].set_ylim(-4.0,4.0)

plt.show()