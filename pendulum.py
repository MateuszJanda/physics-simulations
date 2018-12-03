#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
https://pl.wikipedia.org/wiki/Wahad%C5%82o#Wahad%C5%82o_matematyczne
https://pl.wikipedia.org/wiki/Wahad%C5%82o#Wahad%C5%82o_fizyczne

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


GRAVITY_ACC = 9.81  # [m/s^2]


def main():
    setup_display()
    rod_math = create_rod()

    freq = 100
    dt = 1/freq
    t = 0

    while True:
        vp.rate(freq)

        step_simulation_math(dt, rod_math)

        t += dt


def setup_display():
    vp.canvas(x=0, y=0, width=400, height=400,
        userzoom=False, userspin=True, autoscale=False,
        center=vp.vector(0, 0, 0), foreground=vp.color.white, background=vp.color.black)


def create_rod():
    theta_angle = math.radians(60)
    length = 10

    rod_math = vp.cylinder(pos=vp.vector(0, 2, -5), length=length, radius=0.3,
        axis=length * vp.vector(math.sin(theta_angle), -math.cos(theta_angle), 0),
        d1_theta=0,
        theta=theta_angle)  # [rad]

    return rod_math


def step_simulation_math(dt, rod):
    """ Mathematical pendulum """
    rod.d2_theta = -GRAVITY_ACC/rod.length * math.sin(rod.theta)
    rod.d1_theta += rod.d2_theta * dt
    rod.theta += rod.d1_theta * dt
    rod.axis = rod.length * vp.vector(math.sin(rod.theta), -math.cos(rod.theta), 0)


if __name__ == '__main__':
    main()
