import numpy as np
import matplotlib.pyplot as plt

import kalmanfilters

kf = kalmanfilters.Tracking(5.0)

microsprev = 0.0

tacc = []
acc = []
accp = []
accpunc = []

tdv = []
dv = []
dvp = []
dvpunc = []

tmg = []
mg = []
mgp = []
mgp2 = []
mgpunc = []

with open("examples/data/data3.txt", "r") as f:
    for r in f.readlines()[1:]:
        sensor, data = r.rstrip().split(":")
        micros, x, y, z = data.split(",")

        micros = int(micros)
        x = float(x)
        y = float(y)
        z = float(z)

        if sensor == "accl":
            dt = (micros - microsprev) * 1e-6
            microsprev = micros

            accel = kalmanfilters.sensors.accel(x, y, z)

            # run kf step
            kf.predict(dt)
            kf.update(accel, 0.25)

            tacc.append(micros)
            acc.append([x,y,z])
            accp.append([x - kf.innovation[0], y - kf.innovation[1], z - kf.innovation[2]])
            jac = kf.jac
            s = np.dot(jac, np.dot(kf.state_unc, np.transpose(jac)))
            accpunc.append(s)

        elif sensor == "gyro":
            dt = (micros - microsprev) * 1e-6
            microsprev = micros

            gyro = kalmanfilters.sensors.gyro(x, y, z)

            # run kf step
            kf.predict(dt)
            kf.update(gyro, 0.25)

            R = [
                [1.0, 0.0, np.sin(kf.state[1])],
                [0.0, np.cos(kf.state[0]), np.sin(kf.state[0]) * np.cos(kf.state[1])],
                [0.0, -np.sin(kf.state[0]), np.cos(kf.state[0]) * np.cos(kf.state[1])]
            ]

            tdv.append(micros)
            dv.append(np.dot(np.linalg.inv(R), [x, y, z]).tolist())
            dvp.append([kf.state[3], kf.state[4], kf.state[5]])
            jac = kf.jac
            dvpunc.append(np.dot(np.linalg.inv(R), np.dot([s[3:] for s in kf.state_unc[3:]], np.linalg.inv(R).T)).tolist())



        elif sensor == "mag":
            dt = (micros - microsprev) * 1e-6
            microsprev = micros 

            mag = kalmanfilters.sensors.mag(x, y, z)

            roll, pitch, yaw, _, _, _ = kf.state
            Rroll = [
                [1.0, 0.0, 0.0],
                [0.0, np.cos(roll), -np.sin(roll)],
                [0.0, np.sin(roll),  np.cos(roll)]
            ]
            Rpitch = [
                [np.cos(pitch), 0.0, np.sin(pitch)],
                [0.0, 1.0, 0.0],
                [-np.sin(pitch), 0.0, np.cos(pitch)]
            ]
            Ryaw = [
                [np.cos(yaw), -np.sin(yaw), 0.0],
                [np.sin(yaw), np.cos(yaw), 0.0],
                [0.0, 0.0, 1.0]
            ]
            R = np.dot(Ryaw, np.dot(Rpitch, Rroll))
            m = [24.0475, 5.4344, 51.4601]

            # run kf step
            kf.predict(dt)
            kf.update(mag, 450.0)

            jac = kf.jac
            s = np.dot(jac, np.dot(kf.state_unc, np.transpose(jac)))

            tmg.append(micros)
            mg.append([x,y,z])
            mgp2.append([x - kf.innovation[0], y - kf.innovation[1], z - kf.innovation[2]])
            mgp.append(np.dot(R.T, m).tolist())
            mgpunc.append(s)

        # print('state: \n', kf.state)
        # print('state_unc: \n', kf.state_unc)

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
    ax[i].set_ylim(-2.50,2.50)

plt.show()


mg = np.array(mg)
mgp = np.array(mgp)
mgp2 = np.array(mgp2)
mgpunc = np.array(mgpunc)

fig, ax = plt.subplots(3,1)

for i in range(3):
    ax[i].scatter(tmg, mg[:,i])
    ax[i].plot(tmg, mgp[:,i])
    #ax[i].plot(tmg, mgp2[:,i])
    ax[i].fill_between(
        tmg,
        mgp[:,i] - 2.0 * np.sqrt(mgpunc[:,i,i]),
        mgp[:,i] + 2.0 * np.sqrt(mgpunc[:,i,i]),
        alpha=0.2,
        color='C0'
    )

plt.show()