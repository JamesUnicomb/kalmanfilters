import numpy as np
import matplotlib.pyplot as plt

import kalmanfilters

state = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
state_unc = [
    [4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
]

kf = kalmanfilters.cvqekf(2.0, state, state_unc)

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
mgpunc = []

with open("examples/data/data2.txt", "r") as f:
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

            accel = kalmanfilters.sensors.accel(x, y, z, 0.5, 0.5, 0.5)

            # run kf step
            kf.predict(dt)
            kf.update(accel)

            tacc.append(micros)
            acc.append([x, y, z])
            accp.append(
                [x - kf.innovation[0], y - kf.innovation[1], z - kf.innovation[2]]
            )
            jac = kf.dhdx
            s = kf.innovation_unc
            accpunc.append(s)

        elif sensor == "gyro":
            # dt = (micros - microsprev) * 1e-6
            # microsprev = micros

            gyro = kalmanfilters.sensors.gyro(x, y, z, 0.25, 0.25, 0.25)

            # run kf step
            # kf.predict(dt)
            # kf.update(gyro)

            tdv.append(micros)
            dv.append([x, y, z])
            dvp.append([kf.state[4], kf.state[5], kf.state[6]])
            jac = kf.dhdx
            s = kf.innovation_unc
            dvpunc.append(np.eye(3).tolist())

        elif sensor == "mag":
            dt = (micros - microsprev) * 1e-6
            microsprev = micros

            mag = kalmanfilters.sensors.mag(x, y, z, 400.0, 400.0, 400.0)

            # run kf step
            kf.predict(dt)
            kf.update(mag)

            jac = kf.dhdx
            s = kf.innovation_unc

            tmg.append(micros)
            mg.append([x, y, z])
            mgp.append(
                [x - kf.innovation[0], y - kf.innovation[1], z - kf.innovation[2]]
            )
            mgpunc.append(s)

        # print("state:     \n", kf.state)
        # print('state_unc: \n', kf.state_unc)

acc = np.array(acc)
accp = np.array(accp)
accpunc = np.array(accpunc)

fig, ax = plt.subplots(3, 1)

for i in range(3):
    ax[i].scatter(tacc, acc[:, i])
    ax[i].plot(tacc, accp[:, i])
    ax[i].fill_between(
        tacc,
        accp[:, i] - 2.0 * np.sqrt(accpunc[:, i, i]),
        accp[:, i] + 2.0 * np.sqrt(accpunc[:, i, i]),
        alpha=0.2,
        color="C0",
    )
ax[0].set_ylim(-12.0, 12.0)
ax[1].set_ylim(-12.0, 12.0)
ax[2].set_ylim(-12.0, 12.0)

plt.show()


dv = np.array(dv)
dvp = np.array(dvp)
dvpunc = np.array(dvpunc)

fig, ax = plt.subplots(3, 1, figsize=(8, 5))

for i in range(3):
    ax[i].scatter(tdv, dv[:, i])
    ax[i].plot(tdv, dvp[:, i])
    ax[i].fill_between(
        1e-6 * (np.array(tdv) - tdv[0]),
        dvp[:, i] - 2.0 * np.sqrt(dvpunc[:, i, i]),
        dvp[:, i] + 2.0 * np.sqrt(dvpunc[:, i, i]),
        alpha=0.2,
        color="C0",
    )

ax[0].set_ylim(-7.0, 7.0)
ax[1].set_ylim(-7.0, 7.0)
ax[2].set_ylim(-7.0, 7.0)

plt.show()


mg = np.array(mg)
mgp = np.array(mgp)
mgpunc = np.array(mgpunc)

fig, ax = plt.subplots(3, 1)

for i in range(3):
    ax[i].scatter(tmg, mg[:, i])
    ax[i].plot(tmg, mgp[:, i])
    ax[i].fill_between(
        tmg,
        mgp[:, i] - 2.0 * np.sqrt(mgpunc[:, i, i]),
        mgp[:, i] + 2.0 * np.sqrt(mgpunc[:, i, i]),
        alpha=0.2,
        color="C0",
    )
    ax[i].set_ylim()

ax[0].set_ylim(-57.5, 57.5)
ax[1].set_ylim(-57.5, 57.5)
ax[2].set_ylim(-57.5, 57.5)

plt.show()
