import numpy as np
from math import *

import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *

import serial
from serial.serialutil import SerialException
import kalmanfilters


def get_rot_mat(phi, theta, psi):
    R = [
        [
            cos(psi) * cos(theta),
            sin(phi) * sin(theta) * cos(psi) - sin(psi) * cos(phi),
            sin(phi) * sin(psi) + sin(theta) * cos(phi) * cos(psi),
            0.0,
        ],
        [
            sin(psi) * cos(theta),
            sin(phi) * sin(psi) * sin(theta) + cos(phi) * cos(psi),
            -sin(phi) * cos(psi) + sin(psi) * sin(theta) * cos(phi),
            0.0,
        ],
        [-sin(theta), sin(phi) * cos(theta), cos(phi) * cos(theta), 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]

    return R


axis_verts = (
    (-7.5, 0.0, 0.0),
    (7.5, 0.0, 0.0),
    (0.0, -7.5, 0.0),
    (0.0, 7.5, 0.0),
    (0.0, 0.0, -7.5),
    (0.0, 0.0, 7.5),
)

axes = ((0, 1), (2, 3), (4, 5))

axis_colors = (
    (0.0, 1.0, 0.0),
    (1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0),
)

cube_verts = (
    (-3.0, -3.0, 3.0),
    (-3.0, 3.0, 3.0),
    (3.0, 3.0, 3.0),
    (3.0, -3.0, 3.0),
    (-3.0, -3.0, -3.0),
    (-3.0, 3.0, -3.0),
    (3.0, 3.0, -3.0),
    (3.0, -3.0, -3.0),
)

cube_edges = (
    (0, 1),
    (0, 3),
    (0, 4),
    (2, 1),
    (2, 3),
    (2, 6),
    (5, 1),
    (5, 4),
    (5, 6),
    (7, 3),
    (7, 4),
    (7, 6),
)

cube_surfaces = (
    (0, 1, 2, 3),
    (3, 2, 6, 7),
    (7, 6, 5, 4),
    (4, 5, 1, 0),
    (1, 5, 6, 2),
    (4, 0, 3, 7),
)

cube_colors = (
    (0.0, 0.318, 0.729),
    (0.0, 0.62, 0.376),
    (0.0, 0.318, 0.729),
    (0.0, 0.62, 0.376),
    (0.769, 0.118, 0.227),
    (0.769, 0.118, 0.227),
)


def Axis():
    glBegin(GL_LINES)
    for color, axis in zip(axis_colors, axes):
        glColor3fv(color)
        for point in axis:
            glVertex3fv(axis_verts[point])
    glEnd()


def Cube():
    glBegin(GL_QUADS)
    for color, surface in zip(cube_colors, cube_surfaces):
        glColor3fv(color)
        for vertex in surface:
            glVertex3fv(cube_verts[vertex])
    glEnd()

    glBegin(GL_LINES)
    glColor3fv((0.0, 0.0, 0.0))
    for edge in cube_edges:
        for vertex in edge:
            glVertex3fv(cube_verts[vertex])
    glEnd()


def main():
    ser = serial.Serial("/dev/tty.usbmodem205E3072594D1", 9600)
    ser.flushInput()

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

    kf = kalmanfilters.cvqekf(5.0, state, state_unc)

    microsprev = 0.0

    pygame.init()
    display = (1280, 700)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)

    # Using depth test to make sure closer colors are shown over further ones
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LESS)

    # Default view
    glMatrixMode(GL_PROJECTION)
    gluPerspective(45, (display[0] / display[1]), 0.5, 40)
    glTranslatef(0.0, 0.0, -17.5)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        gluLookAt(7.0, 7.0, -7.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0)

        try:
            ser_bytes = ser.readline()
            decoded_bytes = ser_bytes.decode("utf-8")

            sensor, data = decoded_bytes.rstrip().split(":")
            micros, x, y, z = data.split(",")

            micros = int(micros)
            x = float(x)
            y = float(y)
            z = float(z)

            if sensor == "accl":
                dt = (micros - microsprev) * 1e-6
                microsprev = micros

                accel = kalmanfilters.sensors.accel(x, y, z, 0.25, 0.25, 0.25)

                # run kf step
                kf.predict(dt)
                kf.update(accel)

            elif sensor == "gyro":
                dt = (micros - microsprev) * 1e-6
                microsprev = micros

                gyro = kalmanfilters.sensors.gyro(x, y, z, 0.25, 0.25, 0.25)

                # run kf step
                kf.predict(dt)
                kf.update(gyro)

            elif sensor == "mag":
                dt = (micros - microsprev) * 1e-6
                microsprev = micros

                mag = kalmanfilters.sensors.mag(x, y, z, 45.0, 45.0, 45.0)

                # run kf step
                kf.predict(dt)
                kf.update(mag)

            # print(kf.state, end="\r")
            qw, qx, qy, qz, _, _, _ = kf.state

        except (KeyboardInterrupt, SerialException) as e:
            print(e)
            break
        except Exception as e:
            print(sensor, e)
            pass

        glMatrixMode(GL_MODELVIEW)
        rot = np.array(kalmanfilters.quaternion.q_to_mat4([qw, qx, qy, qz]))
        glLoadMatrixf(rot)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        Cube()
        Axis()
        pygame.display.flip()
        pygame.time.wait(10)


main()