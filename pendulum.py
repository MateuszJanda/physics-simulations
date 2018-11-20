#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from vpython import *
import math
from copy import copy

import os
if os.name == 'nt':
    import ImageGrab


"""
http://pages.physics.cornell.edu/~sethna/StatMech/ComputerExercises/PythonSoftware/Pendulum.py

http://sciaga.onet.pl/12581,60,162,104,1,22603,1,sciaga.html

http://efizyka.net.pl/wahadlo-matematyczne-i-fizyczne_8435
http://efizyka.net.pl/ruch-harmoniczny-opis_7605
http://efizyka.net.pl/predkosc-i-przyspieszenie-w-ruchu-harmonicznym_7840
http://efizyka.net.pl/sila-w-ruchu-harmonicznym_7981
http://efizyka.net.pl/oscylator-harmoniczny-tlumiony_8530
"""


class PrintScreen:
    def __init__(self):
        self.frameNum = 0

    def grabImage(self):
        if os.name != 'nt':
            return

        fileName = 'img-' + '{fr:03d}'.format(fr=self.frameNum) + '.png'
        im = ImageGrab.grab((0, 0, 400, 400))
        im.save(fileName)

        self.frameNum += 1

GRAVITY = 9.81  # m/s^2

class Rod1():
    def __init__(self, pos_y, dt):
        self.rod = cylinder(pos=vector(0, pos_y, -5), length=10, radius=0.3)
        self.rod.theta = math.radians(60)
        self.rod.axis = self.rod.length * vector(math.sin(self.rod.theta), -math.cos(self.rod.theta), 0)

        self.mbox = box(pos=self.rod.pos + self.rod.axis, length=1, height=1, width=1)
        self.mbox.visible = False
        self.mbox.mass = 0.5  # kg
        self.mbox.vel = vector(0, 0, 0)  # m/s

        self.dt = dt

    def simulate(self):
        # Siła napięcia
        Fn = -self.rod.length * vector(math.sin(self.rod.theta), math.cos(self.rod.theta), 0)
        # Siła grawitacji
        Fg = self.mbox.mass * GRAVITY * vector(0, -1, 0)
        F = Fn + Fg
        self.mbox.acc = F / self.mbox.mass
        self.mbox.vel += self.mbox.acc * self.dt
        self.mbox.pos += self.mbox.vel * self.dt

        self.rod.theta = math.asin(self.mbox.pos.x / self.rod.length)
        self.rod.axis = self.rod.length * vector(math.sin(self.rod.theta), -math.cos(self.rod.theta), 0)
        self.mbox.pos = self.rod.pos + self.rod.axis


class Rod2():
    def __init__(self, pos_y, dt):
        self.rod = cylinder(pos=vector(0, pos_y, -5), length=10, radius=0.3, color=color.red)
        self.rod.theta = math.radians(60)
        self.rod.axis = self.rod.length * vector(math.sin(self.rod.theta), -math.cos(self.rod.theta), 0)

        self.dt = dt
        self.thetaD1 = 0

    def simulate(self):
        thetaD2 = -GRAVITY/self.rod.length * math.sin(self.rod.theta)
        self.thetaD1 += thetaD2 * self.dt
        self.rod.theta += self.thetaD1 * self.dt

        self.rod.axis = self.rod.length * vector(math.sin(self.rod.theta), -math.cos(self.rod.theta), 0)


def main():
    setupDisplay()
    screen = PrintScreen()

    freq = 100
    dt = 1.0 / freq
    t = 0

    rod1 = Rod1(12, dt)
    rod2 = Rod2(-2, dt)

    while t < 3000:
        rate(freq)

        rod1.simulate()
        rod2.simulate()

        t += dt
        screen.grabImage()

    exit()


def setupDisplay():
    canvas(x=0, y=0, width=400, height=400,
        userzoom=False, userspin=True, autoscale=False,
        center=vector(0, 0, 0), foreground=color.white, background=color.black)


if __name__ == '__main__':
    main()