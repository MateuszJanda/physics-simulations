#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import vpython as vp
import itertools as it
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

    while t < 6:
        vp.rate(freq)

        set_thrust(t, bodies)
        visualize_thrust(bodies)
        step_simulation(dt, bodies)

        t += dt


def setup_display():
    vp.canvas(x=0, y=0, width=400, height=400,
        userzoom=False, userspin=True, autoscale=False,
        center=vp.vector(0, 0, 0), foreground=vp.color.white, background=vp.color.black)


def create_bodies():
    width = 3
    height = 2
    body1 = vp.box(pos=vp.vector(-5, 0, 0), axis=vp.vector(0, 0, 1), width=width, height=height,
        arrow=vp.arrow(pos=vp.vector(0, 0, 0), shaftwidth=0.5, color=vp.color.red, visible=False),
        radius=1/2 * math.sqrt(width**2 + height**2),
        mass=10,  # kg
        inertia=100,
        area=10,  # m^2
        vel=vp.vector(0, 0, 0),
        ang_vel=vp.vector(0, 0, 0),
        theta=vp.radians(20))

    width = 2
    height = 3
    body2 = vp.box(pos=vp.vector(3, 0.5, 0), axis=vp.vector(0, 0, 1), width=width, height=height,
        arrow=vp.arrow(pos=vp.vector(0, 0, 0), shaftwidth=0.5, color=vp.color.blue, visible=False),
        radius=1/2 * math.sqrt(width**2 + height**2),
        mass=10,  # kg
        inertia=100,
        area=10,  # m^2
        vel=vp.vector(0, 0, 0),
        ang_vel=vp.vector(0, 0, 0),
        theta=vp.radians(0))

    bodies = [body1, body2]
    for body in bodies:
        body.vertices = vertices(body)
        body.rotate(angle=body.theta)

    return bodies


def vertices(body):
    body_vertices = [vp.vector(body.width/2, body.height/2, 0),
                     vp.vector(-body.width/2, body.height/2, 0),
                     vp.vector(-body.width/2, -body.height/2, 0),
                     vp.vector(body.width/2, -body.height/2, 0)]

    for idx, pt in enumerate(body_vertices):
        body_vertices[idx] = vp.rotate(pt, angle=body.theta, axis=vp.vector(0, 0, 1)) + body.pos

    return body_vertices


def set_thrust(t, bodies):
    if t > 0 and t < 1:
        bodies[0].force = 100 * vp.rotate(vp.vector(1, 0, 0), bodies[0].theta, vp.vector(0, 0, 1))
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
        calc_forces(dt, body)

    for body in bodies:
        integrate(dt, body)

    collisions = find_collisions(bodies)
    resolve_collisions(collisions)


def calc_forces(dt, body):
    VEL_TOLERANCE = 0.2

    if body.vel.mag > VEL_TOLERANCE:
        body.force += -body.vel.norm() * LINEAR_DRAG_COEFFICIENT * 0.5 * \
            DENSITY_OF_AIR * body.vel.mag2 * body.area


def integrate(dt, body):
    body.acc = body.force/body.mass
    body.vel += body.acc * dt
    body.pos += body.vel * dt

    M = vp.vector(0, 0, 0)
    body.ang_acc = M/body.inertia
    body.ang_vel += body.ang_acc * dt
    angle_growth = body.ang_vel.z * dt

    body.rotate(angle=angle_growth, axis=vp.vector(0, 0, 1))
    body.theta += angle_growth
    body.vertices = vertices(body)


class Collision:
    def __init__(self, body1, body2, collision_pt1, collision_pt2, relative_vel, collision_normal):
        self.body1 = body1
        self.body2 = body2
        self.collision_pt1 = collision_pt1
        self.collision_pt2 = collision_pt2
        self.relative_vel = relative_vel
        self.collision_normal = collision_normal


def find_collisions(bodies):
    DISTANCE_TOLERANCE = 0.01
    collisions = []

    for body1, body2 in it.combinations(bodies, 2):
        allowed_dist = body1.radius + body2.radius
        dist = body2.pos - body1.pos
        real_dist = dist.mag - allowed_dist

        if real_dist > DISTANCE_TOLERANCE:
            continue

        c = collision_node_node(body1, body2)
        if c:
            collisions.append(c)
            continue

        c = collision_node_edge(body1, body2)
        if c:
            collisions.append(c)
            continue

        c = penetration_by_node(body1, body2)
        if c:
            collisions.append(c)

    return collisions


def collision_node_node(body1, body2):
    for pt1, pt2 in it.product(body1.vertices, body2.vertices):
        if not equal_points(pt1, pt2):
            continue

        collision_pt1 = pt1 - body1.pos
        collision_pt2 = pt1 - body2.pos

        collision_normal = vp.norm(body1.pos - body2.pos)

        vel1 = body1.vel + vp.cross(body1.ang_vel, collision_pt1)
        vel2 = body2.vel + vp.cross(body2.ang_vel, collision_pt2)

        relative_vel = vel1 - vel2
        relative_vel_n = vp.dot(relative_vel, collision_normal)

        if relative_vel_n < 0:
            return Collision(body1, body2, collision_pt1, collision_pt2, relative_vel, collision_normal)

    return None


def equal_points(pt1, pt2, delta=0.3):
    return (pt1 - pt2).mag <= delta


def collision_node_edge(body1, body2):
    CTOL = 0.03

    for pt1, (pt2, pt2_next) in it.product(body1.vertices, zip(body2.vertices, body2.vertices[1:]+body2.vertices[:1])):
        edge = pt2_next - pt2

        u = vp.norm(edge)
        p = pt1 - pt2
        proj = u * vp.dot(p, u)

        if vp.mag(proj + edge) <= edge.mag or proj.mag > edge.mag:
            continue

        # Daje taki sam wynik jak dist = (proj - p).mag
        dist = vp.mag(vp.cross(p, u))
        if dist > CTOL:
            continue

        collision_pt1 = pt1 - body1.pos
        collision_pt2 = pt1 - body2.pos

        collision_normal = vp.norm(vp.cross(vp.cross(u, p), u))

        vel1 = body1.vel + vp.cross(body1.ang_vel, collision_pt1)
        vel2 = body2.vel + vp.cross(body2.ang_vel, collision_pt2)

        relative_vel = vel1 - vel2
        relative_vel_n = vp.dot(relative_vel, collision_normal)

        if relative_vel_n < 0:
            return Collision(body1, body2, collision_pt1, collision_pt2, relative_vel, collision_normal)

    return None


def penetration_by_node(body1, body2):
    for pt1 in body1.vertices:
        penetration = True
        for idx, pt2 in enumerate(body2.vertices):
            edge = body2.vertices[(idx + 1) % 4] - body2.vertices[idx]

            p = pt1 - pt2
            dott = vp.dot(p, edge)
            if dott < 0:
                penetration = False
                break

        if penetration:
            # TODO: Czy można to lepiej obliczyć?
            collision_normal = vp.norm(body1.pos - body2.pos)
            relative_vel = body1.vel - body2.vel
            return Collision(body1, body2, pt1, pt1, relative_vel, collision_normal)

    return None


def resolve_collisions(collisions):
    for c in collisions:
        impulse = (-(1+COEFFICIENT_OF_RESTITUTION) * (vp.dot(c.relative_vel, c.collision_normal))) / \
            ((1/c.body1.mass + 1/c.body2.mass) + \
             vp.dot(c.collision_normal, vp.cross(vp.cross(c.collision_pt1, c.collision_normal) / c.body1.inertia, c.collision_pt1)) + \
             vp.dot(c.collision_normal, vp.cross(vp.cross(c.collision_pt2, c.collision_normal) / c.body2.inertia, c.collision_pt2)))

        c.body1.vel += impulse * c.collision_normal / c.body1.mass
        c.body1.ang_vel += vp.cross(c.collision_pt1, (impulse * c.collision_normal)) / c.body1.inertia

        c.body2.vel -= impulse * c.collision_normal / c.body2.mass
        c.body2.ang_vel -= vp.cross(c.collision_pt2, (impulse * c.collision_normal)) / c.body2.inertia


if __name__ == '__main__':
    main()
