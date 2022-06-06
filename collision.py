#!/usr/bin/env python3

# Author: Mateusz Janda <mateusz janda at gmail com>
# Site: github.com/MateuszJanda/physics-simulations
# Ad maiorem Dei gloriam

import vpython as vp
import itertools as it
import math
# import povexport


LINEAR_DRAG_COEFFICIENT = 0.2
DENSITY_OF_AIR = 1.168  # [kg/m^3]
COEFFICIENT_OF_RESTITUTION = 0.5


def main():
    scene = setup_display()
    bodies = create_bodies()

    freq = 100
    dt = 1/freq
    t = 0

    frame = 0
    while t < 4:
        vp.rate(freq)

        set_thrust(t, bodies)
        visualize_thrust(bodies)
        step_simulation(dt, bodies)

        # povexport.export(scene, filename='img-%04d.pov' % frame, include_list=['colors.inc', 'stones.inc', 'woods.inc', 'metals.inc'])
        frame += 1

        t += dt


def setup_display():
    scene = vp.canvas(x=0, y=0, width=400, height=400, center=vp.vector(0, 0, 0),
        userzoom=False, userspin=True, autoscale=False,
        foreground=vp.color.white, background=vp.color.black)

    return scene


def create_bodies():
    body1 = vp.cylinder(pos=vp.vector(-5, 0, 0), axis=vp.vector(0, 0, 1), radius=1,
        mass=10,  # [kg]
        area=10,  # [m^2]
        vel=vp.vector(0, 0, 0),  # [m/s^2]
        arrow=vp.arrow(pos=vp.vector(0, 0, 0), shaftwidth=0.5, color=vp.color.red, visible=False))

    body2 = vp.cylinder(pos=vp.vector(3, 0.5, 0), axis=vp.vector(0, 0, 1), radius=1,
        mass=10,  # [kg]
        area=10,  # [m^2]
        vel=vp.vector(0, 0, 0),  # [m/s^2]
        arrow=vp.arrow(pos=vp.vector(0, 0, 0), shaftwidth=0.5, color=vp.color.blue, visible=False))

    return [body1, body2]


def set_thrust(t, bodies):
    if t < 1:
        bodies[0].force = vp.vector(100, 0, 0)
    else:
        bodies[0].force = vp.vector(0, 0, 0)

    bodies[1].force = vp.vector(0, 0, 0)


def visualize_thrust(bodies):
    """
    Should be call before forces calculation, when body.forces is equal to
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


def step_simulation(dt, bodies):
    for body in bodies:
        calc_forces(body)

    for body in bodies:
        integrate(dt, body)

    collisions = find_collisions(bodies)
    resolve_collisions(dt, collisions)


def calc_forces(body):
    VEL_TOLERANCE = 0.2

    if body.vel.mag > VEL_TOLERANCE:
        body.force += -body.vel.norm() * LINEAR_DRAG_COEFFICIENT * 0.5 * \
            DENSITY_OF_AIR * body.vel.mag2 * body.area


def integrate(dt, body):
    body.acc = body.force / body.mass
    body.vel += body.acc * dt
    body.pos += body.vel * dt


class Collision:
    def __init__(self, body1, body2, relative_vel, collision_normal):
        self.body1 = body1
        self.body2 = body2
        self.relative_vel = relative_vel
        self.collision_normal = collision_normal


def find_collisions(bodies):
    DISTANCE_TOLERANCE = 0.01
    collisions = []

    for body1, body2 in it.combinations(bodies, 2):
        allowed_dist = body1.radius + body2.radius
        dist = body1.pos - body2.pos
        real_dist = dist.mag - allowed_dist

        # Perpendicular to action line
        collision_normal = dist.norm()
        relative_vel = body1.vel - body2.vel
        # Normal component of relative velocity - does two bodies are on collision course
        relative_vel_n = vp.dot(relative_vel, collision_normal)

        if real_dist < DISTANCE_TOLERANCE and relative_vel_n < 0:
            collisions.append(Collision(body1, body2, relative_vel, collision_normal))

    return collisions


def resolve_collisions(dt, collisions):
    # https://en.wikipedia.org/wiki/Collision_response#Computing_impulse-based_reaction
    for c in collisions:
        impulse = (-(1+COEFFICIENT_OF_RESTITUTION) * vp.dot(c.relative_vel, c.collision_normal)) / \
            (1/c.body1.mass + 1/c.body2.mass)

        c.body1.vel += impulse * c.collision_normal / c.body1.mass
        c.body2.vel -= impulse * c.collision_normal / c.body2.mass


if __name__ == '__main__':
    main()
