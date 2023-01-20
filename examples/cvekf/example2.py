import numpy as np
import matplotlib.pyplot as plt

from kalmanfilters import cvekf
from kalmanfilters.linalg import Vector, Matrix
from kalmanfilters.sensors import accel, gyro, mag

state = Vector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
state_unc = Matrix(
    [
        [10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 10.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 10.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    ]
)

kf = cvekf(5.0, state, state_unc)

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

            Z = accel(x, y, z, 0.025, 0.025, 0.025)

            # run kf step
            kf.predict(dt)
            kf.update(Z)

            tacc.append(micros)
            acc.append([x, y, z])
            Y = kf.get_innovation().tovec()
            accp.append([x - Y[0], y - Y[1], z - Y[2]])
            s = kf.get_innovation_unc().tovec()
            accpunc.append(s)

        elif sensor == "gyro":
            dt = (micros - microsprev) * 1e-6
            microsprev = micros

            Z = gyro(x, y, z, 0.25, 0.25, 0.25)

            # run kf step
            kf.predict(dt)
            kf.update(Z)

            tdv.append(micros)
            dv.append([x, y, z])
            Y = kf.get_innovation().tovec()
            dvp.append([x - Y[0], y - Y[1], z - Y[2]])
            s = kf.get_innovation_unc().tovec()
            dvpunc.append(s)

        elif sensor == "mag":
            dt = (micros - microsprev) * 1e-6
            microsprev = micros

            Z = mag(x, y, z, 45.0, 45.0, 45.0)

            # run kf step
            kf.predict(dt)
            kf.update(Z)

            tmg.append(micros)
            mg.append([x, y, z])
            Y = kf.get_innovation().tovec()
            mgp.append([x - Y[0], y - Y[1], z - Y[2]])
            s = kf.get_innovation_unc().tovec()
            mgpunc.append(s)

        # print('state:     \n', kf.state)
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

fig, ax = plt.subplots(3, 1)

for i in range(3):
    ax[i].scatter(tdv, dv[:, i])
    ax[i].plot(tdv, dvp[:, i])
    ax[i].fill_between(
        tdv,
        dvp[:, i] - 2.0 * np.sqrt(dvpunc[:, i, i]),
        dvp[:, i] + 2.0 * np.sqrt(dvpunc[:, i, i]),
        alpha=0.2,
        color="C0",
    )

ax[0].set_ylim(-8.0, 8.0)
ax[1].set_ylim(-8.0, 8.0)
ax[2].set_ylim(-8.0, 8.0)

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
    ax[i].set_ylim(-57.50, 57.50)

plt.show()
