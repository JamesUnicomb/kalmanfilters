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
    (1.0, 0.0, 0.0),  # Red
    (0.0, 1.0, 0.0),  # Green
    (0.0, 0.0, 1.0),  # Blue
)


"""
       5____________6
       /           /|
      /           / |
    1/__________2/  |
    |           |   |
    |           |   |
    |           |   7
    |           |  /
    |           | /
    0___________3/
"""

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
    (0, 1, 2, 3),  # Front
    (3, 2, 6, 7),  # Right
    (7, 6, 5, 4),  # Left
    (4, 5, 1, 0),  # Back
    (1, 5, 6, 2),  # Top
    (4, 0, 3, 7),  # Bottom
)

cube_colors = (
    (0.0, 0.318, 0.729),  # Blue
    (0.769, 0.118, 0.227),  # Red
    (0.0, 0.318, 0.729),  # Blue
    (0.769, 0.118, 0.227),  # Red
    (0.0, 0.62, 0.376),  # Green
    (0.0, 0.62, 0.376),  # Green
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

    kf = kalmanfilters.cvekf(5.0)

    microsprev = 0.0

    pygame.init()
    display = (800, 600)
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

                mag = kalmanfilters.sensors.mag(x, y, z, 450.0, 450.0, 450.0)

                # run kf step
                kf.predict(dt)
                kf.update(mag)

            # print(kf.state, end="\r")
            phi, theta, psi, _, _, _ = kf.state

        except (KeyboardInterrupt, SerialException) as e:
            print(e)
            break
        except Exception as e:
            print(sensor, e)
            pass

        glMatrixMode(GL_MODELVIEW)
        rot = np.array(get_rot_mat(phi, -psi, theta))
        glLoadMatrixf(rot)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        Cube()
        Axis()
        pygame.display.flip()
        pygame.time.wait(10)


main()
