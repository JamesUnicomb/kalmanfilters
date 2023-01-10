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


compass_verts = (
    (4.0, 0.0, 0.0),
    (5.0, 0.0, 0.0),
    (3.464102, 2.0, 0.0),
    (4.330127, 2.5, 0.0),
    (2.0, 3.464102, 0.0),
    (2.5, 4.330127, 0.0),
    (0.0, 4.0, 0.0),
    (0.0, 5.0, 0.0),
    (-2.0, 3.464102, 0.0),
    (-2.5, 4.330127, 0.0),
    (-3.464102, 2.0, 0.0),
    (-4.330127, 2.5, 0.0),
    (-4.0, 0.0, 0.0),
    (-5.0, 0.0, 0.0),
    (-3.464102, -2.0, 0.0),
    (-4.330127, -2.5, 0.0),
    (-2.0, -3.464102, 0.0),
    (-2.5, -4.330127, 0.0),
    (-0.0, -4.0, 0.0),
    (-0.0, -5.0, 0.0),
    (2.0, -3.464102, 0.0),
    (2.5, -4.330127, 0.0),
    (3.464102, -2.0, 0.0),
    (4.330127, -2.5, 0.0),
)

compass = (
    (0, 1),
    (2, 3),
    (4, 5),
    (6, 7),
    (8, 9),
    (10, 11),
    (12, 13),
    (14, 15),
    (16, 17),
    (18, 19),
    (20, 21),
    (22, 23),
)

compass_colors = (
    (1.0, 0.0, 0.0),
    (1.0, 1.0, 1.0),
    (1.0, 1.0, 1.0),
    (1.0, 1.0, 1.0),
    (1.0, 1.0, 1.0),
    (1.0, 1.0, 1.0),
    (1.0, 1.0, 1.0),
    (1.0, 1.0, 1.0),
    (1.0, 1.0, 1.0),
    (1.0, 1.0, 1.0),
    (1.0, 1.0, 1.0),
    (1.0, 1.0, 1.0),
)


def Compass():
    glBegin(GL_LINES)
    for color, axis in zip(compass_colors, compass):
        glColor3fv(color)
        for point in axis:
            glVertex3fv(compass_verts[point])
    glEnd()


def main():
    ser = serial.Serial("/dev/tty.usbmodem205E3072594D1", 9600)
    ser.flushInput()

    state = [1.0, 0.0, 0.0, 0.0]
    state_unc = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]

    kf = kalmanfilters.cpqekf(0.01, state, state_unc)

    microsprev = 0.0

    pygame.init()
    display = (360, 360)
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

        gluLookAt(0.0, 0.0, 7.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0)

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

                accel = kalmanfilters.sensors.accel(x, y, z, 0.025, 0.025, 0.025)

                # run kf step
                kf.predict(dt)
                kf.update(accel)

            elif sensor == "mag":
                dt = (micros - microsprev) * 1e-6
                microsprev = micros

                mag = kalmanfilters.sensors.mag(x, y, z, 45.0, 45.0, 45.0)

                # run kf step
                kf.predict(dt)
                kf.update(mag)

        except (KeyboardInterrupt, SerialException) as e:
            print(e)
            break
        except Exception as e:
            print(e)
            pass

        glMatrixMode(GL_MODELVIEW)

        q = np.array(kf.state)
        q *= 1.0 / np.linalg.norm(q)
        kf.state = q.tolist()

        phi, theta, psi = kalmanfilters.quaternion.q_to_euler(q)

        glLoadMatrixf(get_rot_mat(0.0, 0.0, -psi))

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        Compass()

        pygame.display.flip()
        pygame.time.wait(10)


main()
