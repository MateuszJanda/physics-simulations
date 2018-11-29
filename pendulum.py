#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
http://pages.physics.cornell.edu/~sethna/StatMech/ComputerExercises/PythonSoftware/Pendulum.py

http://sciaga.onet.pl/12581,60,162,104,1,22603,1,sciaga.html

http://efizyka.net.pl/wahadlo-matematyczne-i-fizyczne_8435
http://efizyka.net.pl/ruch-harmoniczny-opis_7605
http://efizyka.net.pl/predkosc-i-przyspieszenie-w-ruchu-harmonicznym_7840
http://efizyka.net.pl/sila-w-ruchu-harmonicznym_7981
http://efizyka.net.pl/oscylator-harmoniczny-tlumiony_8530
"""

import vpython as vp
import math


GRAVITY = 9.81  # m/s^2


class Rod2():
    def __init__(self, pos_y, dt):
        self.rod = vp.cylinder(pos=vp.vector(0, pos_y, -5), length=10, radius=0.3)
        self.rod.theta = math.radians(60)
        self.rod.axis = self.rod.length * vp.vector(math.sin(self.rod.theta), -math.cos(self.rod.theta), 0)

        self.dt = dt
        self.thetaD1 = 0

    def simulate(self):
        thetaD2 = -GRAVITY/self.rod.length * math.sin(self.rod.theta)
        self.thetaD1 += thetaD2 * self.dt
        self.rod.theta += self.thetaD1 * self.dt

        self.rod.axis = self.rod.length * vp.vector(math.sin(self.rod.theta), -math.cos(self.rod.theta), 0)


def main():
    setupDisplay()

    freq = 100
    dt = 1.0 / freq
    t = 0

    rod2 = Rod2(-2, dt)

    while t < 3000:
        vp.rate(freq)

        rod2.simulate()

        t += dt


def setupDisplay():
    vp.canvas(x=0, y=0, width=400, height=400,
        userzoom=False, userspin=True, autoscale=False,
        center=vp.vector(0, 0, 0), foreground=vp.color.white, background=vp.color.black)


if __name__ == '__main__':
    main()
