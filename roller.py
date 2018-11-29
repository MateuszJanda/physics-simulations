#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
https://www.youtube.com/watch?v=HGcZrHnqjeg
"""

import vpython as vp
import math
import os
from datetime import time


GRAVITY = 9.81  # m/s^2


def texture():
    tex = {'file':vp.textures.rug }
    return tex


def main():
    setupDisplay()

    tiltAngle = 15 * (math.pi / 180)
    posInit = vp.vector(0, -0.5, 0)
    slipper = vp.box(pos=posInit, length=5)
    # Musi być minus, bo funkcja obraca odwrotnie do ruchu wskazówek zegara
    slipper.rotate(angle=-tiltAngle, axis=vp.vector(0, 0, 1))

    tex = texture()
    vInit = vp.vector(0, 0, 0)
    roller = vp.cylinder(pos=vp.vector(-1 * math.cos(tiltAngle) * 0.5 * slipper.length,
                                 math.sin(tiltAngle) * 0.5 * slipper.length + 0.5,
                                 0),
                      axis=vp.vector(0, 0, 1), radius=0.5, texture=tex)
    roller.vel = vInit
    roller.velAng = 0
    roller.mass = 1.0
    roller.coeffOfFriction = 0.15  # Współczynnik tarcia

    freq = 100
    dt = 1.0 / freq
    t = 0

    slipDir = vp.norm(vp.rotate(vp.vector(1, 0, 0), angle=-tiltAngle))
    vp.arrow(pos=vp.vector(0, 2, 1), axis=slipDir)

    totalAngle = 0
    while t < 3000:
        vp.rate(freq)

        # Ruch obrotowy
        # Moment bezwładności dla walca ze wzoru
        Icg = 0.5 * roller.mass * roller.radius**2
        Ff = roller.coeffOfFriction * roller.mass * GRAVITY * math.cos(tiltAngle)
        roller.velAng += ((Ff * roller.radius) / Icg) * dt
        # Musi być minus, bo funkcja obraca odwrotnie do ruchu wskazówek zegara
        dAngle = roller.velAng * dt
        roller.rotate(angle=-dAngle)

        # Ruch środka masy
        # a = g(sin(a) - u*cos(a)) - str. 104
        # również na filmiku - "toczenie z poślizgiem" - 13:21:00
        roller.acc2 = GRAVITY * (math.sin(tiltAngle) - roller.coeffOfFriction * math.cos(tiltAngle)) * slipDir
        # z filmiku "toczenie bez poślizgu" - 10:24:00
        roller.acc = (roller.mass * GRAVITY * math.sin(tiltAngle)) / (roller.mass + Icg/roller.radius**2) * slipDir
        roller.vel += roller.acc * dt
        roller.pos += roller.vel * dt + 0.5 * roller.acc * dt**2

        # Jeżeli dotrze do krawędzi pochylni to koniec symulacji
        if -roller.pos.y + roller.radius >= math.sin(tiltAngle) * slipper.length/2:
            exit()
            break

        # Sprawdzenie, czy obrót wokół własnej osi jest równy przebytej drodze (ruch bez poślizgu)
        totalAngle += dAngle
        if totalAngle >= 2 * math.pi:
            circuit = 2 * math.pi * roller.radius
            pathLen = ((-roller.pos.y - roller.radius) / math.sin(tiltAngle)) % circuit
            print('obwod: %f, droga: %f' % (circuit, pathLen))
            totalAngle -= (2 * math.pi)

        t += dt


def setupDisplay():
    vp.canvas(x=0, y=0, width=400, height=400,
        userzoom=False, userspin=True, autoscale=False,
        center=vp.vector(0, 0, -9), foreground=vp.color.white, background=vp.color.black)


if __name__ == '__main__':
    main()
