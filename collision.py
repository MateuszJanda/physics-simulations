#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from vpython import *
import math
#from vpython.graph import *


TOLERANCE = 0.2
LINEAR_DRAG_COEFFICIENT = 0.2
DENSITY_OF_AIR = 1.168  # kg/m^3
OVER_THE_BODY = vector(0, 0, 1)

def main():
    setupDisplay()

    body1 = cylinder(pos=vector(-5, 0, 0), axis=vector(0, 0, 1), radius=1)
    body1.mass = 10  # kg
    body1.projectedArea = 10  # m^2
    body1.vel = vector(0, 0, 0)

    body2 = cylinder(pos=vector(3, 0.5, 0), axis=vector(0, 0, 1), radius=1)
    body2.mass = 10  # kg
    body2.projectedArea = 10  # m^2
    body2.vel = vector(0, 0, 0)


    arrRed, arrBlue = getArrows()

    t = 0
    freq = 100
    dt = 1.0 / freq

    while t < 12:
        rate(freq)

        if t > 0 and t < 1:
            F1 = 100 * vector(1, 0, 0)
        else:
            F1 = vector(0, 0, 0)

        F2 = vector(0, 0, 0)

        updateBody(body1, F1, arrRed, dt)
        updateBody(body2, F2, arrBlue, dt)

        if checCollision(body1, body2):
            collision(body1, body2)

        t += dt


def setupDisplay():
    canvas(x=0, y=0, width=400, height=400,
        userzoom=False, userspin=True, autoscale=False,
        center=vector(0, 0, 0), foreground=color.white, background=color.black)


def getArrows():
    arrRed = arrow(pos=vector(0, 0, 0), shaftwidth=0.5, color=color.red)
    arrRed.visible = False
    arrBlue = arrow(pos=vector(0, 0, 0), shaftwidth=0.5, color=color.blue)
    arrBlue.visible = False

    return arrRed, arrBlue


def updateBody(body, Ftrust, arr, dt):
    F = vector(0, 0, 0)

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


def checCollision(body1, body2):
    CTOL = 0.01

    r = body1.radius + body2.radius
    d = body1.pos - body2.pos
    s = d.mag - r

    collisionNorm = d.norm()
    relativVel = body1.vel - body2.vel

    vrn = dot(relativVel, collisionNorm)
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
    j = ( -(1+e) * (dot(relativVel, collisionNorm))) / ( (1/body1.mass + 1/body2.mass))
    body1.vel += j * collisionNorm / body1.mass
    body2.vel -= j * collisionNorm / body2.mass


if __name__ == '__main__':
    main()
