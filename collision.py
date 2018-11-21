#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import vpython as vp
import math


LINEAR_DRAG_COEFFICIENT = 0.2
DENSITY_OF_AIR = 1.168  # kg/m^3
COEFFICIENT_OF_RESTITUTION = 0.5


def main():
    setup_display()
    bodies = create_bodies()

    freq = 100
    dt = 1/freq
    t = 0

    while t < 12:
        vp.rate(freq)

        set_thrust(t, bodies)
        visualize_thrust(bodies)
        step_simulation(dt, bodies)

        t += dt


def setup_display():
    vp.canvas(x=0, y=0, width=400, height=400, center=vp.vector(0, 0, 0),
        userzoom=False, userspin=True, autoscale=False,
        foreground=vp.color.white, background=vp.color.black)


def create_bodies():
    body1 = vp.cylinder(pos=vp.vector(-5, 0, 0), axis=vp.vector(0, 0, 1), radius=1,
        mass = 10,  # kg
        area = 10,  # m^2
        vel = vp.vector(0, 0, 0),  # m/s^2
        arrow = vp.arrow(pos=vp.vector(0, 0, 0), shaftwidth=0.5, color=vp.color.red, visible=False))

    body2 = vp.cylinder(pos=vp.vector(3, 0.5, 0), axis=vp.vector(0, 0, 1), radius=1,
        mass = 10,  # kg
        area = 10,  # m^2
        vel = vp.vector(0, 0, 0),  # m/s^2
        arrow = vp.arrow(pos=vp.vector(0, 0, 0), shaftwidth=0.5, color=vp.color.blue, visible=False))

    return [body1, body2]


def set_thrust(t, bodies):
    if t < 1:
        bodies[0].force = vp.vector(100, 0, 0)
    else:
        bodies[0].force = vp.vector(0, 0, 0)

    bodies[1].force = vp.vector(0, 0, 0)


def step_simulation(dt, bodies):
    for body in bodies:
        calc_forces(body)

    for body in bodies:
        body.acc = body.force / body.mass
        body.vel += body.acc * dt
        body.pos += body.vel * dt

    resolve_collisions(bodies[0], bodies[1])


def calc_forces(body):
    VEL_TOLERANCE = 0.2

    if body.vel.mag > VEL_TOLERANCE:
        body.force += -body.vel.norm() * LINEAR_DRAG_COEFFICIENT * 0.5 * \
            DENSITY_OF_AIR * body.vel.mag2 * body.area


def visualize_thrust(bodies):
    """
    Should be call bofore forces calculation, when body.forces is equal to
    thrust only.
    """
    POS_OVER_THE_BODY = vp.vector(0, 0, 1)
    LENGTH_FACTOR = 4

    for body in bodies:
        if body.force.mag > 0:
            body.arrow.pos = POS_OVER_THE_BODY + body.pos
            body.arrow.axis = LENGTH_FACTOR * body.force.norm()
            body.arrow.visible = True
        else:
            body.arrow.visible = False


def resolve_collisions(body1, body2):
    # https://en.wikipedia.org/wiki/Collision_response#Computing_impulse-based_reaction
    DISTANCE_VEL_TOLERANCE = 0.01

    r = body1.radius + body2.radius
    dist = body1.pos - body2.pos
    s = dist.mag - r

    collision_normal = dist.norm()
    relative_vel = body1.vel - body2.vel
    relative_vel_normal = vp.dot(relative_vel, collision_normal)

    if s > DISTANCE_VEL_TOLERANCE or relative_vel_normal > 0:
        return

    impulse = (-(1+COEFFICIENT_OF_RESTITUTION) * vp.dot(relative_vel, collision_normal)) / (1/body1.mass + 1/body2.mass)
    body1.vel += impulse * collision_normal / body1.mass
    body2.vel -= impulse * collision_normal / body2.mass


if __name__ == '__main__':
    main()
