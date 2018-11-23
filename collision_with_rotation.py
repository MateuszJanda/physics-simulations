#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import vpython as vp
import math

# tolerancja błędu
CTOL = 0.01

TOLERANCE = 0.2
LINEAR_DRAG_COEFFICIENT = 0.2
DENSITY_OF_AIR = 1.168  # kg/m^3
OVER_THE_BODY = vp.vector(0, 0, 1)

COLLISION = 1
PENETRATION = -1
NO_COLLISION = 0


def main():
    setup_display()
    body1, body2 = create_bodies()
    arrRed, arrBlue = createArrows()

    freq = 100
    dt = 1/freq
    t = 0

    while t < 6:
        vp.rate(freq)

        if t > 0 and t < 1:
            F1 = 100 * vp.rotate(vp.vector(1, 0, 0), body1.theta, vp.vector(0, 0, 1))
        else:
            F1 = vp.vector(0, 0, 0)

        F2 = vp.vector(0, 0, 0)

        updateBody(body1, F1, arrRed, dt)
        updateBody(body2, F2, arrBlue, dt)

        data = checkCollision(body1, body2)
        if data:
            collision(body1, body2, *data)

        t += dt


def setup_display():
    vp.canvas(x=0, y=0, width=400, height=400,
        userzoom=False, userspin=True, autoscale=False,
        center=vp.vector(0, 0, 0), foreground=vp.color.white, background=vp.color.black)


def create_bodies():
    body1 = vp.box(pos=vp.vector(-5, 0, 0), axis=vp.vector(0, 0, 1), width=3, height=2)
    body1.radius = 1/2 * math.sqrt(body1.width**2 + body1.height**2)
    body1.mass = 10  # kg
    body1.inertia = 100
    body1.projectedArea = 10  # m^2
    body1.vel = vp.vector(0, 0, 0)
    body1.angularVel = vp.vector(0, 0, 0)
    body1.theta = vp.radians(20)
    body1.vertices = vertices(body1)

    body2 = vp.box(pos=vp.vector(3, 0.5, 0), axis=vp.vector(0, 0, 1), width=2, height=3)
    body2.radius = 1/2 * math.sqrt(body2.width**2 + body2.height**2)
    body2.mass = 10  # kg
    body2.inertia = 100
    body2.projectedArea = 10  # m^2
    body2.vel = vp.vector(0, 0, 0)
    body2.angularVel = vp.vector(0, 0, 0)
    body2.theta = vp.radians(0)
    body2.vertices = vertices(body2)

    body1.rotate(angle=body1.theta)
    body2.rotate(angle=body2.theta)

    return body1, body2


def vertices(body):
    bodyVertices = [vp.vector(body.width/2, body.height/2, 0),
                    vp.vector(-body.width/2, body.height/2, 0),
                    vp.vector(-body.width/2, -body.height/2, 0),
                    vp.vector(body.width/2, -body.height/2, 0)]

    for idx, vx in enumerate(bodyVertices):
        bodyVertices[idx] = vp.rotate(vx, angle=body.theta, axis=vp.vector(0, 0, 1)) + body.pos

    return bodyVertices


def createArrows():
    arrRed = vp.arrow(pos=vp.vector(0, 0, 0), shaftwidth=0.5, color=vp.color.red)
    arrRed.visible = False
    arrBlue = vp.arrow(pos=vp.vector(0, 0, 0), shaftwidth=0.5, color=vp.color.blue)
    arrBlue.visible = False

    return arrRed, arrBlue


def updateBody(body, Ftrust, arr, dt):
    F = vp.vector(0, 0, 0)
    M = vp.vector(0, 0, 0)

    # Poniżej pewnego progu nie obliczamy prędkości stycznej
    if body.vel.mag > TOLERANCE:
        R = -body.vel.norm() * LINEAR_DRAG_COEFFICIENT * 0.5 * DENSITY_OF_AIR * body.vel.mag**2 * body.projectedArea
        F += R

    if Ftrust.mag > 0:
        arr.pos = OVER_THE_BODY + body.pos
        arr.axis = 4 * Ftrust.norm()
        arr.visible = True
    else:
        arr.visible = False

    F += Ftrust

    body.acc = F/body.mass
    body.vel += body.acc * dt
    body.pos += body.vel * dt

    body.angularAcc = M/body.inertia
    body.angularVel += body.angularAcc * dt
    angleGrowth = body.angularVel.z * dt

    body.rotate(angle=angleGrowth, axis=vp.vector(0, 0, 1))
    body.theta += angleGrowth
    body.vertices = vertices(body)


def checkCollision(body1, body2):
    r = body1.radius + body2.radius
    d = body2.pos - body1.pos
    s = d.mag - r

    if s > CTOL:
        return None

    data = checkNodeNode(body1, body2)
    if data:
        return data

    data = checkNodeEdge(body1, body2)
    if data:
        return data

    data = checkNodePenetration(body1, body2)
    if data:
        return data

    return None


def checkNodeNode(body1, body2):
    for vx1 in body1.vertices:
        for vx2 in body2.vertices:
            if not arePointsEqual(vx1, vx2): continue

            body1.collisionPoint = vx1 - body1.pos
            body2.collisionPoint = vx1 - body2.pos

            collisionNorm = body1.pos - body2.pos
            collisionNorm = collisionNorm.norm()

            v1 = body1.vel + cross(body1.angularVel, body1.collisionPoint)
            v2 = body2.vel + cross(body2.angularVel, body2.collisionPoint)

            # Jest w książce, ale wydaje mi się, że wektor prędkości jest już obrócony
            # v1 = rotate(v1, angle=body1.theta, axis=(0, 0, 1))
            # v2 = rotate(v2, angle=body2.theta, axis=(0, 0, 1))

            relativVel = v1 - v2
            vrn = dot(relativVel, collisionNorm)

            if vrn < 0.0:
                return collisionNorm, relativVel

    return None


def arePointsEqual(pt1, pt2):
    CTOL = 0.3
    return (pt1 - pt2).mag <= CTOL


def checkNodeEdge(body1, body2):
    CTOL = 0.03

    for vx1 in body1.vertices:
        for idx, vx2 in enumerate(body2.vertices):
            edge = body2.vertices[(idx + 1) % 4] - body2.vertices[idx]

            u = edge
            u = u.norm()

            p = vx1 - vx2
            proj = u * vp.dot(p, u)
            if (proj + edge).mag <= edge.mag or proj.mag > edge.mag: continue

            d = vp.cross(p, u)
            # Daje taki sam wynik jak dist = (proj - p).mag
            dist = d.mag
            if dist > CTOL: continue

            body1.collisionPoint = vx1 - body1.pos
            body2.collisionPoint = vx1 - body2.pos

            collisionNorm = vp.cross(vp.cross(u, p), u)
            collisionNorm = collisionNorm.norm()

            v1 = body1.vel + vp.cross(body1.angularVel, body1.collisionPoint)
            v2 = body2.vel + vp.cross(body2.angularVel, body2.collisionPoint)

            # Jest w książce, ale wydaje mi się, że wektor prędkości jest już obrócony
            # v1 = rotate(v1, angle=body1.theta, axis=(0, 0, 1))
            # v2 = rotate(v2, angle=body2.theta, axis=(0, 0, 1))

            relativVel = v1 - v2
            vrn = vp.dot(relativVel, collisionNorm)

            if vrn < 0.0:
                return collisionNorm, relativVel

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
            body1.collisionPoint = vx1
            body2.collisionPoint = vx1
            collisionNorm = body1.pos - body2.pos
            collisionNorm = collisionNorm.norm()
            relativVel = body1.vel - body2.vel
            return collisionNorm, relativVel

    return None


def collision(body1, body2, collisionNorm, relativVel):
    e = 0.5

    j = (-(1+e) * (vp.dot(relativVel, collisionNorm))) / \
        ((1/body1.mass + 1/body2.mass) + \
         vp.dot(collisionNorm, vp.cross(vp.cross(body1.collisionPoint, collisionNorm) / body1.inertia, body1.collisionPoint)) + \
         vp.dot(collisionNorm, vp.cross(vp.cross(body2.collisionPoint, collisionNorm) / body2.inertia, body2.collisionPoint)))

    body1.vel += j * collisionNorm / body1.mass
    body1.angularVel += vp.cross(body1.collisionPoint, (j * collisionNorm)) / body1.inertia

    body2.vel -= j * collisionNorm / body2.mass
    body2.angularVel -= vp.cross(body2.collisionPoint, (j * collisionNorm)) / body2.inertia


if __name__ == '__main__':
    main()
