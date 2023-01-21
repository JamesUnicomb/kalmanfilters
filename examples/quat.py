from sympy import symbols, Matrix, cos, sin, diff, transpose

"""
This is a basic example of how to compute jacobians and 
turn it into simpler c++ code using symbolic programming (sympy).

The equations can be much more complicated and sympy will easily 
calculate the derivatives for you.
"""

print_accel_innovation = False
print_mag_innovation = False
print_accel_jacobian = False
print_mag_jacobian = False
print_process_noise = True
print_state_transition_jacobian = False

qw, qx, qy, qz = symbols("qw qx qy qz")  # unit quaternion
omx, omy, omz = symbols("omx omy omz")  # angular rates of change
g = symbols("g")  # gravity
mx, my, mz = symbols("mx my mz")  # magnetic field vector
delta = symbols("delta")  # time differential

R = Matrix(
    [
        [2 * (qw * qw + qx * qx) - 1, 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 2 * (qw * qw + qy * qy) - 1, 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 2 * (qw * qw + qz * qz) - 1],
    ]
)

G = transpose(R) * Matrix([[0], [0], [g]])
M = transpose(R) * Matrix([[mx], [my], [mz]])


if print_accel_innovation:
    for i, (gr, s) in enumerate(zip(G, ["accel.x", "accel.y", "accel.z"])):
        print("y[", i, "] = ", s, " - (", gr, ");")

if print_mag_innovation:
    for i, (mr, s) in enumerate(zip(M, ["mag.x", "mag.y", "mag.z"])):
        print("y[", i, "] = ", s, " - (", mr, ");")

if print_accel_jacobian:
    for i, s in enumerate([qw, qx, qy, qz]):
        for j, gr in enumerate(diff(G, s)):
            print("jac[", j, "][", i, "] = ", gr, ";")

if print_mag_jacobian:
    for i, s in enumerate([qw, qx, qy, qz]):
        for j, mr in enumerate(diff(M, s)):
            print("jac[", j, "][", i, "] = ", mr, ";")


"""
This section shows how to calculate the state transition equations and their
derivatives for the extended kalman filter quaternion with body angular rates

The input measurements will be accel, gyro and mag
"""


F = Matrix(
    [
        [qw + delta * 0.5 * (0.0 * qw - omx * qx - omy * qy - omz * qz)],
        [qx + delta * 0.5 * (omx * qw + 0.0 * qx + omz * qy - omy * qz)],
        [qy + delta * 0.5 * (omy * qw - omz * qx + 0.0 * qy + omx * qz)],
        [qz + delta * 0.5 * (omz * qw + omy * qx - omx * qy + 0.0 * qz)],
        [omx],
        [omy],
        [omz],
    ]
)

if print_state_transition_jacobian:
    for i, s in enumerate([qw, qx, qy, qz, omx, omy, omz]):
        for j, fr in enumerate(diff(F, s)):
            print("jac[", j, "][", i, "] = ", fr, ";")


dF = Matrix(
    [
        [0.5 * (-qx), 0.5 * (-qy), 0.5 * (-qz)],
        [0.5 * qw, 0.5 * (-qz), 0.5 * qy],
        [0.5 * qz, 0.5 * qw, 0.5 * (-qx)],
        [0.5 * (-qy), 0.5 * qx, 0.5 * qw],
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
    ]
)

if print_process_noise:
    for k, r in enumerate(dF * transpose(dF)):
        i = k // 7
        j = k % 7
        if i < 4 and j < 4:
            print(
                "process_unc[",
                i,
                "][",
                j,
                "] = q/3.0 * delta * delta * delta * (",
                r,
                ");",
            )
        if i < 4 and j > 3:
            print("process_unc[", i, "][", j, "] = q/2.0 * delta * delta * (", r, ");")
        if i > 3 and j < 4:
            print("process_unc[", i, "][", j, "] = q/2.0 * delta * delta * (", r, ");")
        if i > 3 and j > 3:
            if r != 0:
                print("process_unc[", i, "][", j, "] = q * delta;")
            else:
                print("process_unc[", i, "][", j, "] = 0.0;")
