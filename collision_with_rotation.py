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

    body1.vertices = vertices(body1)
    body2.vertices = vertices(body2)

    body1.rotate(angle=body1.theta)
    body2.rotate(angle=body2.theta)

    return [body1, body2]


def vertices(body):
    body_vertices = [vp.vector(body.width/2, body.height/2, 0),
                     vp.vector(-body.width/2, body.height/2, 0),
                     vp.vector(-body.width/2, -body.height/2, 0),
                     vp.vector(body.width/2, -body.height/2, 0)]

    for idx, vx in enumerate(body_vertices):
        body_vertices[idx] = vp.rotate(vx, angle=body.theta, axis=vp.vector(0, 0, 1)) + body.pos

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
    def __init__(self, body1, body2, relative_vel, collision_normal):
        self.body1 = body1
        self.body2 = body2
        self.relative_vel = relative_vel
        self.collision_normal = collision_normal


def find_collisions(bodies):
    CTOL = 0.01
    collisions = []

    for body1, body2 in it.combinations(bodies, 2):
        r = body1.radius + body2.radius
        d = body2.pos - body1.pos
        s = d.mag - r

        if s > CTOL:
            continue

        c = checkNodeNode(body1, body2)
        if c:
            collisions.append(c)
            continue

        c = checkNodeEdge(body1, body2)
        if c:
            collisions.append(c)
            continue

        c = checkNodePenetration(body1, body2)
        if c:
            collisions.append(c)

    return collisions


def checkNodeNode(body1, body2):
    for vx1, vx2 in it.product(body1.vertices, body2.vertices):
        if not arePointsEqual(vx1, vx2):
            continue

        body1.collision_pt = vx1 - body1.pos
        body2.collision_pt = vx1 - body2.pos

        collision_normal = vp.norm(body1.pos - body2.pos)

        v1 = body1.vel + vp.cross(body1.ang_vel, body1.collision_pt)
        v2 = body2.vel + vp.cross(body2.ang_vel, body2.collision_pt)

        relative_vel = v1 - v2
        vrn = vp.dot(relative_vel, collision_normal)

        if vrn < 0:
            return Collision(body1, body2, relative_vel, collision_normal)

    return None


def arePointsEqual(pt1, pt2):
    CTOL = 0.3
    return (pt1 - pt2).mag <= CTOL


def checkNodeEdge(body1, body2):
    CTOL = 0.03

    # for vx1 in body1.vertices:
    for vx1, vxs2 in it.product(body1.vertices, zip(body2.vertices, body2.vertices[1:]+body2.vertices[:1])):
        vx2, vx2_end = vxs2
        # for idx, vx2 in enumerate(body2.vertices):
            # edge = body2.vertices[(idx + 1) % 4] - body2.vertices[idx]
        edge = vx2_end - vx2

        u = vp.norm(edge)

        p = vx1 - vx2
        proj = u * vp.dot(p, u)
        if vp.mag(proj + edge) <= edge.mag or proj.mag > edge.mag:
            continue

        # Daje taki sam wynik jak dist = (proj - p).mag
        dist = vp.mag(vp.cross(p, u))
        if dist > CTOL:
            continue

        body1.collision_pt = vx1 - body1.pos
        body2.collision_pt = vx1 - body2.pos

        collision_normal = vp.norm(vp.cross(vp.cross(u, p), u))

        v1 = body1.vel + vp.cross(body1.ang_vel, body1.collision_pt)
        v2 = body2.vel + vp.cross(body2.ang_vel, body2.collision_pt)

        relative_vel = v1 - v2
        vrn = vp.dot(relative_vel, collision_normal)

        if vrn < 0:
            return Collision(body1, body2, relative_vel, collision_normal)

    return None


def checkNodePenetration(body1, body2):
    for vx1 in body1.vertices:
        penetration = True
        for idx, vx2 in enumerate(body2.vertices):
            edge = body2.vertices[(idx + 1) % 4] - body2.vertices[idx]

            p = vx1 - vx2
            dott = vp.dot(p, edge)
            if dott < 0:
                penetration = False
                break

        if penetration:
            # TODO: Czy można to lepiej obliczyć?
            body1.collision_pt = vx1
            body2.collision_pt = vx1
            collision_normal = body1.pos - body2.pos
            collision_normal = collision_normal.norm()
            relative_vel = body1.vel - body2.vel
            return Collision(body1, body2, relative_vel, collision_normal)

    return None


def resolve_collisions(collisions):
    for c in collisions:
        j = (-(1+COEFFICIENT_OF_RESTITUTION) * (vp.dot(c.relative_vel, c.collision_normal))) / \
            ((1/c.body1.mass + 1/c.body2.mass) + \
             vp.dot(c.collision_normal, vp.cross(vp.cross(c.body1.collision_pt, c.collision_normal) / c.body1.inertia, c.body1.collision_pt)) + \
             vp.dot(c.collision_normal, vp.cross(vp.cross(c.body2.collision_pt, c.collision_normal) / c.body2.inertia, c.body2.collision_pt)))

        c.body1.vel += j * c.collision_normal / c.body1.mass
        c.body1.ang_vel += vp.cross(c.body1.collision_pt, (j * c.collision_normal)) / c.body1.inertia

        c.body2.vel -= j * c.collision_normal / c.body2.mass
        c.body2.ang_vel -= vp.cross(c.body2.collision_pt, (j * c.collision_normal)) / c.body2.inertia


if __name__ == '__main__':
    main()
