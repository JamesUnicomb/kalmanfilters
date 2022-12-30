from sympy import symbols, Matrix, cos, sin, diff, transpose

"""
This is a basic example of how to compute jacobians and 
turn it into simpler c++ code using symbolic programming (sympy).

The equations can be much more complicated and sympy will easily 
calculate the derivatives for you.
"""

c0, s0, c1, s1, c2, s2 = symbols("c0 s0 c1 s1 c2 s2")
R2 = Matrix([[c2, -s2, 0], [s2, c2, 0], [0, 0, 1]])
R1 = Matrix([[c1, 0, s1], [0, 1, 0], [-s1, 0, c1]])
R0 = Matrix([[1, 0, 0], [0, c0, -s0], [0, s0, c0]])

print(transpose(R2 * R1 * R0))


phi, theta, psi, mx, my, mz = symbols("phi theta psi mx my mz")
R2 = Matrix([[cos(psi), -sin(psi), 0], [sin(psi), cos(psi), 0], [0, 0, 1]])
R1 = Matrix([[cos(theta), 0, sin(theta)], [0, 1, 0], [-sin(theta), 0, cos(theta)]])
R0 = Matrix([[1, 0, 0], [0, cos(phi), -sin(phi)], [0, sin(phi), cos(phi)]])
m = Matrix([mx, my, mz])


print(diff(transpose(R2 * R1 * R0) * m, phi))
print(diff(transpose(R2 * R1 * R0) * m, theta))
print(diff(transpose(R2 * R1 * R0) * m, psi))

j1 = "Matrix([[0], [mx*(sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi)) + my*(-sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi)) + mz*cos(phi)*cos(theta)], [mx*(-sin(phi)*sin(theta)*cos(psi) + sin(psi)*cos(phi)) + my*(-sin(phi)*sin(psi)*sin(theta) - cos(phi)*cos(psi)) - mz*sin(phi)*cos(theta)]])"
j2 = "Matrix([[-mx*sin(theta)*cos(psi) - my*sin(psi)*sin(theta) - mz*cos(theta)], [mx*sin(phi)*cos(psi)*cos(theta) + my*sin(phi)*sin(psi)*cos(theta) - mz*sin(phi)*sin(theta)], [mx*cos(phi)*cos(psi)*cos(theta) + my*sin(psi)*cos(phi)*cos(theta) - mz*sin(theta)*cos(phi)]])"
j3 = "Matrix([[-mx*sin(psi)*cos(theta) + my*cos(psi)*cos(theta)], [mx*(-sin(phi)*sin(psi)*sin(theta) - cos(phi)*cos(psi)) + my*(sin(phi)*sin(theta)*cos(psi) - sin(psi)*cos(phi))], [mx*(sin(phi)*cos(psi) - sin(psi)*sin(theta)*cos(phi)) + my*(sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi))]])"


j1 = j1.replace("sin(phi)", "s0")
j1 = j1.replace("cos(phi)", "c0")
j1 = j1.replace("sin(theta)", "s1")
j1 = j1.replace("cos(theta)", "c1")
j1 = j1.replace("sin(psi)", "s2")
j1 = j1.replace("cos(psi)", "c2")

print(j1)

j2 = j2.replace("sin(phi)", "s0")
j2 = j2.replace("cos(phi)", "c0")
j2 = j2.replace("sin(theta)", "s1")
j2 = j2.replace("cos(theta)", "c1")
j2 = j2.replace("sin(psi)", "s2")
j2 = j2.replace("cos(psi)", "c2")

print(j2)

j3 = j3.replace("sin(phi)", "s0")
j3 = j3.replace("cos(phi)", "c0")
j3 = j3.replace("sin(theta)", "s1")
j3 = j3.replace("cos(theta)", "c1")
j3 = j3.replace("sin(psi)", "s2")
j3 = j3.replace("cos(psi)", "c2")

print(j3)
