#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import vpython as vp
import itertools as it
import math


LINEAR_DRAG_COEFFICIENT = 0.2
DENSITY_OF_AIR = 1.168  # [kg/m^3]
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
        moment_inertia=100,  # [kg*m^2]
        area=10,  # [m^2]
        vel=vp.vector(0, 0, 0),  # [m/s]
        ang_vel=vp.vector(0, 0, 0),  # [rad/s^2]
        theta=vp.radians(20))  # [rad]

    width = 2
    height = 3
    body2 = vp.box(pos=vp.vector(3, 0.5, 0), axis=vp.vector(0, 0, 1), width=width, height=height,
        arrow=vp.arrow(pos=vp.vector(0, 0, 0), shaftwidth=0.5, color=vp.color.blue, visible=False),
        radius=1/2 * math.sqrt(width**2 + height**2),
        mass=10,  # [kg]
        moment_inertia=100,  # [kg*m^2]
        area=10,  # [m^2]
        vel=vp.vector(0, 0, 0),  # [m/s]
        ang_vel=vp.vector(0, 0, 0),  # [rad/s^2]
        theta=vp.radians(0))  # [rad]

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
    resolve_collisions(dt, collisions)


def calc_forces(dt, body):
    VEL_TOLERANCE = 0.2

    if body.vel.mag > VEL_TOLERANCE:
        body.force += -body.vel.norm() * LINEAR_DRAG_COEFFICIENT * 0.5 * \
            DENSITY_OF_AIR * body.vel.mag2 * body.area

    body.moment_force = vp.vector(0, 0, 0)


def integrate(dt, body):
    body.acc = body.force/body.mass
    body.vel += body.acc * dt
    body.pos += body.vel * dt

    body.ang_acc = body.moment_force/body.moment_inertia
    body.ang_vel += body.ang_acc * dt
    angle_diff = body.ang_vel.z * dt
    body.theta += angle_diff
    body.rotate(angle=angle_diff, axis=vp.vector(0, 0, 1))

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
        if not close_points(pt1, pt2):
            continue

        collision_pt1 = pt1 - body1.pos
        collision_pt2 = pt1 - body2.pos

        collision_normal = vp.norm(body1.pos - body2.pos)

        vel1 = body1.vel + vp.cross(body1.ang_vel, collision_pt1)
        vel2 = body2.vel + vp.cross(body2.ang_vel, collision_pt2)
        relative_vel = vel1 - vel2

        if is_collision_course(relative_vel, collision_normal):
            return Collision(body1, body2, collision_pt1, collision_pt2, relative_vel, collision_normal)

    return None


def close_points(pt1, pt2, delta=0.3):
    return (pt1 - pt2).mag <= delta


def collision_node_edge(body1, body2):
    DISTANCE_TOLERANCE = 0.03

    for pt1, (pt2, pt2_next) in it.product(body1.vertices, zip(body2.vertices, body2.vertices[1:]+body2.vertices[:1])):
        edge = pt2_next - pt2
        edge_normal = vp.norm(edge)

        p = pt1 - pt2
        proj_on_edge = edge_normal * vp.dot(p, edge_normal)

        if vp.mag(proj_on_edge + edge) <= edge.mag or proj_on_edge.mag > edge.mag:
            continue

        # Perpendicular distance to edge
        dist_to_edge = vp.mag(vp.cross(p, edge_normal))
        if dist_to_edge > DISTANCE_TOLERANCE:
            continue

        collision_pt1 = pt1 - body1.pos
        collision_pt2 = pt1 - body2.pos

        collision_normal = vp.norm(vp.cross(vp.cross(edge_normal, p), edge_normal))

        vel1 = body1.vel + vp.cross(body1.ang_vel, collision_pt1)
        vel2 = body2.vel + vp.cross(body2.ang_vel, collision_pt2)
        relative_vel = vel1 - vel2

        if is_collision_course(relative_vel, collision_normal):
            return Collision(body1, body2, collision_pt1, collision_pt2, relative_vel, collision_normal)

    return None


def is_collision_course(relative_vel, collision_normal):
    relative_vel_n = vp.dot(relative_vel, collision_normal)
    return relative_vel_n < 0


def penetration_by_node(body1, body2):
    for pt1 in body1.vertices:
        penetration = True
        for pt2, pt2_next in zip(body2.vertices, body2.vertices[1:]+body2.vertices[:1]):
            edge = pt2_next - pt2

            p = pt1 - pt2
            mag_proj_on_edge = vp.dot(p, vp.norm(edge))

            if mag_proj_on_edge < 0:
                penetration = False
                break

        if penetration:
            collision_normal = vp.norm(body1.pos - body2.pos)
            relative_vel = body1.vel - body2.vel
            return Collision(body1, body2, pt1, pt1, relative_vel, collision_normal)

    return None


def resolve_collisions(dt, collisions):
    # https://en.wikipedia.org/wiki/Collision_response#Computing_impulse-based_reaction
    for c in collisions:
        impulse = (-(1+COEFFICIENT_OF_RESTITUTION) * vp.dot(c.relative_vel, c.collision_normal)) / \
            (1/c.body1.mass + 1/c.body2.mass + \
             vp.dot(c.collision_normal, vp.cross(vp.cross(c.collision_pt1, c.collision_normal) / c.body1.moment_inertia, c.collision_pt1)) + \
             vp.dot(c.collision_normal, vp.cross(vp.cross(c.collision_pt2, c.collision_normal) / c.body2.moment_inertia, c.collision_pt2)))

        c.body1.vel += impulse * c.collision_normal / c.body1.mass
        c.body1.pos += c.body1.vel * dt

        c.body2.vel -= impulse * c.collision_normal / c.body2.mass
        c.body2.pos += c.body2.vel * dt

        c.body1.ang_vel += vp.cross(c.collision_pt1, (impulse * c.collision_normal)) / c.body1.moment_inertia
        angle_diff = c.body1.ang_vel.z * dt
        c.body1.theta += angle_diff
        c.body1.rotate(angle=angle_diff, axis=vp.vector(0, 0, 1))

        c.body2.ang_vel -= vp.cross(c.collision_pt2, (impulse * c.collision_normal)) / c.body2.moment_inertia
        angle_diff = c.body2.ang_vel.z * dt
        c.body2.theta += angle_diff
        c.body2.rotate(angle=angle_diff, axis=vp.vector(0, 0, 1))


if __name__ == '__main__':
    main()
