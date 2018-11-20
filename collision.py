#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import vpython as vp
import math


TOLERANCE = 0.2
LINEAR_DRAG_COEFFICIENT = 0.2
DENSITY_OF_AIR = 1.168  # kg/m^3
POS_OVER_THE_BODY = vp.vector(0, 0, 1)


def main():
    setup_display()

    body1, body2 = create_bodies()
    red_arrow, blue_arrow = create_arrows()

    t = 0
    freq = 100
    dt = 1/freq

    while t < 12:
        vp.rate(freq)

        if t > 0 and t < 1:
            F1 = 100 * vp.vector(1, 0, 0)
        else:
            F1 = vp.vector(0, 0, 0)

        F2 = vp.vector(0, 0, 0)

        updateBody(body1, F1, red_arrow, dt)
        updateBody(body2, F2, blue_arrow, dt)

        if checCollision(body1, body2):
            collision(body1, body2)

        t += dt


def setup_display():
    vp.canvas(x=0, y=0, width=400, height=400, center=vp.vector(0, 0, 0),
        userzoom=False, userspin=True, autoscale=False,
        foreground=vp.color.white, background=vp.color.black)


def create_bodies():
    body1 = vp.cylinder(pos=vp.vector(-5, 0, 0), axis=vp.vector(0, 0, 1), radius=1)
    body1.mass = 10  # kg
    body1.area = 10  # m^2
    body1.vel = vp.vector(0, 0, 0)  # m/s^2

    body2 = vp.cylinder(pos=vp.vector(3, 0.5, 0), axis=vp.vector(0, 0, 1), radius=1)
    body2.mass = 10  # kg
    body2.area = 10  # m^2
    body2.vel = vp.vector(0, 0, 0)  # m/s^2

    return body1, body2


def create_arrows():
    red_arrow = vp.arrow(pos=vp.vector(0, 0, 0), shaftwidth=0.5, color=vp.color.red)
    red_arrow.visible = False

    blue_arrow = vp.arrow(pos=vp.vector(0, 0, 0), shaftwidth=0.5, color=vp.color.blue)
    blue_arrow.visible = False

    return red_arrow, blue_arrow


def updateBody(body, thrust, arrow, dt):
    force = vp.vector(0, 0, 0)

    # Poniżej pewnego progu nie obliczamy prędkości stycznej
    if body.vel.mag > TOLERANCE:
        R = -body.vel.norm() * LINEAR_DRAG_COEFFICIENT * 0.5 * DENSITY_OF_AIR * body.vel.mag**2 * body.area
        force += R

    if thrust.mag > 0:
        arrow.pos = POS_OVER_THE_BODY + body.pos
        arrow.axis = 4 * thrust.norm()
        arrow.visible = True
    else:
        arrow.visible = False

    force += thrust

    body.acc = force/body.mass
    body.vel += body.acc * dt
    body.pos += body.vel * dt


def checCollision(body1, body2):
    CTOL = 0.01

    r = body1.radius + body2.radius
    d = body1.pos - body2.pos
    s = d.mag - r

    collisionNorm = d.norm()
    relativVel = body1.vel - body2.vel

    vrn = vp.dot(relativVel, collisionNorm)
    if math.fabs(s) <= CTOL and vrn < 0.0:
        # zderzenie
        return 1
    elif s < -CTOL:
        # penetracja
        return -1
    # bez zderzenia
    return 0


def collision(body1, body2):
    e = 0.5
    relativVel = body1.vel - body2.vel
    d = body1.pos - body2.pos
    collisionNorm = d.norm()

    # Wersja z książki
    # j = ( -(1+e) * (dot(relativVel, collisionNorm))) / ( dot(collisionNorm, collisionNorm) * (1/body1.mass + 1/body2.mass))
    # Ale dot(collisionNorm, collisionNorm) i tak zawsze da wynik 1.0
    # Moja wersja
    j = ( -(1+e) * (vp.dot(relativVel, collisionNorm))) / ( (1/body1.mass + 1/body2.mass))
    body1.vel += j * collisionNorm / body1.mass
    body2.vel -= j * collisionNorm / body2.mass


if __name__ == '__main__':
    main()
