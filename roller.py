#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
https://www.youtube.com/watch?v=HGcZrHnqjeg
"""

import vpython as vp
import math


GRAVITY_ACC = 9.81  # [m/s^2]


def main():
    setup_display()
    roller, ramp, arrow = create_bodies()

    freq = 100
    dt = 1/freq
    t = 0

    while True:
        vp.rate(freq)
        step_simulation(dt, roller, ramp, arrow)

        # At the end of edge end simulation
        if -roller.pos.y + roller.radius >= math.sin(ramp.alpha) * ramp.length/2:
            break

        t += dt


def setup_display():
    vp.canvas(x=0, y=0, width=400, height=400,
        userzoom=False, userspin=True, autoscale=False,
        center=vp.vector(0, 1, -9), foreground=vp.color.white, background=vp.color.black)


def create_bodies():
    ramp = vp.box(pos=vp.vector(0, -0.5, 0), length=5, width=2.5,
        alpha=vp.radians(15))
    # Minus, because function (vp.rotate) rotate counter-clockwise
    ramp.rotate(angle=-ramp.alpha, axis=vp.vector(0, 0, 1))

    roller = vp.cylinder(pos=vp.vector(-1 * math.cos(ramp.alpha) * 0.5 * ramp.length,
                                       math.sin(ramp.alpha) * 0.5 * ramp.length + 0.5,
                                       0),
        axis=vp.vector(0, 0, 1), radius=0.5, texture=texture(),
        vel=vp.vector(0, 0, 0),
        ang_vel=0,
        mass=1,
        friction_coeff=0.15)

    slope_dir = vp.norm(vp.rotate(vp.vector(1, 0, 0), angle=-ramp.alpha))
    arrow = vp.arrow(pos=vp.vector(0, 2, 1), axis=slope_dir)

    return roller, ramp, arrow


def texture():
    tex = {'file':vp.textures.rug }
    return tex


def step_simulation(dt, roller, ramp, arrow):
    calc_forces(roller, ramp.alpha)
    integrate(dt, roller, ramp.alpha)


def calc_forces(roller, alpha):
    roller.force = roller.friction_coeff * roller.mass * GRAVITY_ACC * math.cos(alpha)
    roller.moment = 0.5 * roller.mass * roller.radius**2


def integrate(dt, roller, alpha):
    slope_dir = vp.norm(vp.rotate(vp.vector(1, 0, 0), angle=-alpha))

    # Rolling with sliding (movie - 13:21:00)
    # a = g(sin(a) - u*cos(a)) - page 104
    # roller.acc = GRAVITY_ACC * (math.sin(alpha) - roller.friction_coeff * math.cos(alpha)) * slope_dir

    # Rolling without sliding - (movie - 10:24:00)
    roller.acc = (roller.mass * GRAVITY_ACC * math.sin(alpha)) / (roller.mass + roller.moment/roller.radius**2) * slope_dir
    roller.vel += roller.acc * dt
    roller.pos += roller.vel * dt

    roller.ang_vel += ((roller.force * roller.radius) / roller.moment) * dt
    angle_diff = roller.ang_vel * dt
    # Minus, because function (vp.rotate) rotate counter-clockwise
    roller.rotate(angle=-angle_diff)


if __name__ == '__main__':
    main()
