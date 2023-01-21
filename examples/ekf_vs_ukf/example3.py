import numpy as np
import matplotlib.pyplot as plt

from kalmanfilters import cvqukf, cvqekf
from kalmanfilters.linalg import Vector, Matrix
from kalmanfilters.sensors import accel, gyro, mag

state = Vector([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
state_unc = Matrix(
    [
        [10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    ]
)

ekf = cvqekf(15.0, state, state_unc)
ukf = cvqukf(15.0, state, state_unc)

microsprev = 0.0

tacc = []
acc = []
accp = []
accpunc = []

tdv = []
dv = []
dvukf = []
dvukfunc = []
dvekf = []
dvekfunc = []


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

            Z = accel(x, y, z, 0.5, 0.5, 0.5)

            # run kf step
            ekf.predict(dt)
            ekf.update(Z)

            ukf.predict(dt)
            ukf.update(Z)

        elif sensor == "gyro":
            Z = gyro(x, y, z, 0.5, 0.5, 0.5)

            tdv.append(micros)
            dv.append([x, y, z])
            dvekf.append(ekf.get_state().tovec()[4:])
            dvukf.append(ukf.get_state().tovec()[4:])

            s1 = np.add(np.array(ekf.get_state_unc().tovec())[4:, 4:], 0.5 * np.eye(3))
            s2 = np.add(np.array(ukf.get_state_unc().tovec())[4:, 4:], 0.5 * np.eye(3))
            dvekfunc.append(s1)
            dvukfunc.append(s2)

        elif sensor == "mag":
            dt = (micros - microsprev) * 1e-6
            microsprev = micros

            Z = mag(x, y, z, 40.0, 40.0, 40.0)

            # run kf step
            ekf.predict(dt)
            ekf.update(Z)

            ukf.predict(dt)
            ukf.update(Z)

        # print("state:     \n", kf.get_state().tovec())
        # print("state_unc: \n", kf.get_state_unc().tovec())


dv = np.array(dv)
dvekf = np.array(dvekf)
dvukf = np.array(dvukf)
dvekfunc = np.array(dvekfunc)
dvukfunc = np.array(dvukfunc)

fig, ax = plt.subplots(3, 1, figsize=(8, 5))

for i in range(3):
    ax[i].scatter(tdv, dv[:, i], color="k", marker="x")
    ax[i].plot(tdv, dvekf[:, i], color="C0")
    ax[i].plot(tdv, dvukf[:, i], color="C1")
    ax[i].fill_between(
        tdv,
        dvekf[:, i] - 2.0 * np.sqrt(dvekfunc[:, i, i]),
        dvekf[:, i] + 2.0 * np.sqrt(dvekfunc[:, i, i]),
        alpha=0.2,
        color="C0",
    )
    ax[i].fill_between(
        tdv,
        dvukf[:, i] - 2.0 * np.sqrt(dvukfunc[:, i, i]),
        dvukf[:, i] + 2.0 * np.sqrt(dvukfunc[:, i, i]),
        alpha=0.2,
        color="C1",
    )

ax[0].set_ylim(-7.0, 7.0)
ax[1].set_ylim(-7.0, 7.0)
ax[2].set_ylim(-7.0, 7.0)

plt.show()

fig, ax = plt.subplots(1, figsize=(8, 5))

ax.plot(tdv, np.linalg.norm(np.subtract(dv, dvekf), axis=1), color="C0")
ax.plot(tdv, np.linalg.norm(np.subtract(dv, dvukf), axis=1), color="C1")
ax.fill_between(
    tdv,
    np.zeros_like(tdv),
    2.0 * np.sqrt(dvekfunc[:, 0, 0] + dvekfunc[:, 1, 1] + dvekfunc[:, 2, 2]),
    alpha=0.2,
    color="C0",
)
ax.fill_between(
    tdv,
    np.zeros_like(tdv),
    2.0 * np.sqrt(dvukfunc[:, 0, 0] + dvukfunc[:, 1, 1] + dvukfunc[:, 2, 2]),
    alpha=0.2,
    color="C1",
)
ax.set_ylim(0, 20.0)

plt.show()
