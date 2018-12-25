#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
http://stackoverflow.com/questions/18620546/python-gravity-simulator-behaving-strangely
http://spiff.rit.edu/richmond/nbody/OrbitRungeKutta4.pdf
http://showmedo.com/videotutorials/video?name=pythonThompsonVPython8&fromSeriesID=30
"""

import vpython as vp
import itertools as it
import math
# import povexport


GRAVITY_ACC = 1.0  # [m/s^2]


def main():
    scene, plt_x, plt_y = setup_display()
    bodies = create_bodies()

    freq = 100
    dt = 1/freq
    t = 0

    frame = 0
    while True:
        vp.rate(freq)

        step_simulation(dt, bodies)
        visualize_trail_data(t, bodies[1], plt_x, plt_y)

        # povexport.export(scene, filename='img-%04d.pov' % frame, include_list=['colors.inc', 'stones.inc', 'woods.inc', 'metals.inc'])
        frame += 1
        t += dt


def setup_display():
    scene = vp.canvas(x=0, y=0, width=400, height=400,
        userzoom=False, userspin=True, autoscale=False,
        center=vp.vector(0, 0, 3), foreground=vp.color.white, background=vp.color.black)

    vp.arrow(pos=vp.vector(0, 0, 0), axis=vp.vector(10, 0, 0), shaftwidth=0.3)
    vp.arrow(pos=vp.vector(0, 0, 0), axis=vp.vector(0, 10, 0), shaftwidth=0.3)

    plt_x = vp.gcurve(color=vp.color.red, size=2)
    plt_y = vp.gcurve(color=vp.color.red, size=2)

    return scene, plt_x, plt_y


def create_bodies():
    star = vp.sphere(pos=vp.vector(0, 0, 0), radius=1,
        trail=vp.curve(),
        vel=vp.vector(0, 0, 0),
        mass=1000)

    satellite1 = vp.sphere(pos=vp.vector(10, -5, 0), radius=0.5, color=vp.color.green,
        trail=vp.curve(),
        vel=vp.vector(0, 6, 0),
        mass=1.0)

    satellite2 = vp.sphere(pos=vp.vector(-8, -2, 0), radius=0.5, color=vp.color.red,
        trail=vp.curve(),
        vel=vp.vector(3, -9, 0),
        mass=5.0)

    return [star, satellite1, satellite2]


def step_simulation(dt, bodies):
    for body in bodies:
        body.force = vp.vector(0, 0, 0)

    for body1, body2 in it.combinations(bodies, 2):
        calc_forces(dt, body1, body2)

    print(bodies[1].force)
    for body in bodies:
        integrate(dt, body)
        body.trail.append(body.pos)


def calc_forces(dt, body1, body2):
    dist = vp.mag(body1.pos - body2.pos)
    if dist <= body1.radius or dist <= body2.radius:
        print('Impact. The end.')
        exit()

    grav_mag = (GRAVITY_ACC * body1.mass * body2.mass) / (dist**2)

    dir1 = vp.norm(body2.pos - body1.pos)
    dir2 = vp.norm(body1.pos - body2.pos)
    body1.force += dir1 * grav_mag
    body2.force += dir2 * grav_mag


def integrate(dt, body):
    body.acc = body.force / body.mass
    body.vel += body.acc * dt
    body.pos += body.vel * dt


def visualize_trail_data(t, body, plt_x, plt_y):
    plt_x.plot(pos=(t, body.pos.x))
    plt_y.plot(pos=(t, body.pos.y))


if __name__ == '__main__':
    main()
