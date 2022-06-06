#!/usr/bin/env python3

# Author: Mateusz Janda <mateusz janda at gmail com>
# Site: github.com/MateuszJanda/physics-simulations
# Ad maiorem Dei gloriam

"""
References:
http://pages.physics.cornell.edu/~sethna/StatMech/ComputerExercises/PythonSoftware/Pendulum.py

http://efizyka.net.pl/wahadlo-matematyczne-i-fizyczne_8435
http://efizyka.net.pl/ruch-harmoniczny-opis_7605
http://efizyka.net.pl/predkosc-i-przyspieszenie-w-ruchu-harmonicznym_7840
http://efizyka.net.pl/sila-w-ruchu-harmonicznym_7981
http://efizyka.net.pl/oscylator-harmoniczny-tlumiony_8530
"""

import vpython as vp
import math
# import povexport


GRAVITY_ACC = 9.81  # [m/s^2]
DAMPING = 12


def main():
    scene = setup_display()
    rod1 = create_rod_math()
    rod2, bob2 = create_rod_damping()

    freq = 30
    dt = 1/freq
    t = 0

    frame = 0
    while True:
        vp.rate(100)

        step_simulation_math(dt, rod1)
        step_simulation_damping(dt, rod2, bob2)

        # povexport.export(scene, filename='img-%04d.pov' % frame, include_list=['colors.inc', 'stones.inc', 'woods.inc', 'metals.inc'])
        frame += 1
        t += dt


def setup_display():
    scene = vp.canvas(x=0, y=0, width=600, height=400,
        userzoom=False, userspin=True, autoscale=False,
        center=vp.vector(0, 0, 0), foreground=vp.color.white, background=vp.color.black)

    return scene


def create_rod_math():
    theta_angle = math.radians(60)
    length = 10

    rod = vp.cylinder(pos=vp.vector(-8, 2, -5), length=length, radius=0.3,
        axis=length * vp.vector(math.sin(theta_angle), -math.cos(theta_angle), 0),
        d1_theta=0,
        theta=theta_angle)  # [rad]

    return rod


def create_rod_damping():
    theta_angle = math.radians(60)
    length = 10

    rod = vp.cylinder(pos=vp.vector(8, 2, -5), length=length, radius=0.3,
        mass=2,
        axis=length * vp.vector(math.sin(theta_angle), -math.cos(theta_angle), 0),
        d1_theta=0,
        theta=theta_angle)  # [rad]

    ball_pos = vp.vector(rod.pos.x + rod.length * math.sin(rod.theta),
                         rod.pos.y - rod.length * math.cos(rod.theta),
                         rod.pos.z)
    bob = vp.sphere(pos=ball_pos, radius=1.2, color=vp.vector(1, 0, 0))

    return rod, bob


def step_simulation_math(dt, rod):
    """ Mathematical pendulum
    https://pl.wikipedia.org/wiki/Wahad%C5%82o#Wahad%C5%82o_matematyczne
    """
    rod.d2_theta = -GRAVITY_ACC/rod.length * math.sin(rod.theta)
    rod.d1_theta += rod.d2_theta * dt
    rod.theta += rod.d1_theta * dt
    rod.axis = rod.length * vp.vector(math.sin(rod.theta), -math.cos(rod.theta), 0)


def step_simulation_damping(dt, rod, bob):
    """
    Credits:
    https://www.myphysicslab.com/pendulum/moveable-pendulum-en.html
    """
    rod.d2_theta = (-DAMPING/(rod.length**2 + rod.mass)) * rod.d1_theta - \
            ((GRAVITY_ACC/rod.length) * math.sin(rod.theta))
    rod.d1_theta += rod.d2_theta * dt
    rod.theta += rod.d1_theta * dt
    rod.axis = rod.length * vp.vector(math.sin(rod.theta), -math.cos(rod.theta), 0)

    bob.pos = vp.vector(rod.pos.x + rod.length * math.sin(rod.theta),
                        rod.pos.y - rod.length * math.cos(rod.theta),
                        rod.pos.z)


if __name__ == '__main__':
    main()
