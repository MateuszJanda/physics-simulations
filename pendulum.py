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


GRAVITY_ACC = 9.81  # m/s^2


def main():
    setup_display()
    rod = create_rod()

    freq = 100
    dt = 1/freq
    t = 0

    while True:
        vp.rate(freq)

        step_simulation(dt, rod)

        t += dt


def setup_display():
    vp.canvas(x=0, y=0, width=400, height=400,
        userzoom=False, userspin=True, autoscale=False,
        center=vp.vector(0, 0, 0), foreground=vp.color.white, background=vp.color.black)


def create_rod():
    THETA_ANGLE = math.radians(60)
    LENGTH = 10
    rod = vp.cylinder(pos=vp.vector(0, 2, -5), length=LENGTH, radius=0.3,
        axis=LENGTH * vp.vector(math.sin(THETA_ANGLE), -math.cos(THETA_ANGLE), 0),
        theta_d1=0,
        theta=THETA_ANGLE)

    return rod


def step_simulation(dt, rod):
    rod.theta_d2 = -GRAVITY_ACC/rod.length * math.sin(rod.theta)
    rod.theta_d1 += rod.theta_d2 * dt
    rod.theta += rod.theta_d1 * dt
    rod.axis = rod.length * vp.vector(math.sin(rod.theta), -math.cos(rod.theta), 0)


if __name__ == '__main__':
    main()
