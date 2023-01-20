import numpy as np
import matplotlib.pyplot as plt

from kalmanfilters import cvqekf
from kalmanfilters.linalg import Vector, Matrix
from kalmanfilters.sensors import accel, gyro, mag

state = Vector([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
state_unc = Matrix(
    [
        [4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    ]
)

kf = cvqekf(2.0, state, state_unc)

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


with open("examples/data/data1.txt", "r") as f:
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

            Z = accel(x, y, z, 0.5, 0.5, 0.5)

            # run kf step
            kf.predict(dt)
            kf.update(Z)

            Y = kf.get_innovation().tovec()

            tacc.append(micros)
            acc.append([x, y, z])
            accp.append([x - Y[0], y - Y[1], z - Y[2]])
            s = kf.get_innovation_unc().tovec()
            accpunc.append(s)

        elif sensor == "gyro":
            # dt = (micros - microsprev) * 1e-6
            # microsprev = micros

            Z = gyro(x, y, z, 0.5, 0.5, 0.5)

            # run kf step
            # kf.predict(dt)
            # kf.update(gyro)

            tdv.append(micros)
            dv.append([x, y, z])
            dvp.append(kf.get_state().tovec()[4:])
            # jac = kf.dhdx
            # s = kf.innovation_unc
            s = np.add(np.array(kf.get_state_unc().tovec())[4:, 4:], 0.5 * np.eye(3))
            dvpunc.append(s)

        elif sensor == "mag":
            dt = (micros - microsprev) * 1e-6
            microsprev = micros

            Z = mag(x, y, z, 40.0, 40.0, 40.0)

            # run kf step
            kf.predict(dt)
            kf.update(Z)

            Y = kf.get_innovation().tovec()

            tmg.append(micros)
            mg.append([x, y, z])
            mgp.append([x - Y[0], y - Y[1], z - Y[2]])
            s = kf.get_innovation_unc().tovec()
            mgpunc.append(s)

        # print("state:     \n", kf.get_state().tovec())
        # print("state_unc: \n", kf.get_state_unc().tovec())

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
        tdv,
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
