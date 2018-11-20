#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
https://www.youtube.com/watch?v=HGcZrHnqjeg
"""

from vpython import *
import math

import os

from datetime import time

if os.name == 'nt':
    import ImageGrab


class PrintScreen:
    def __init__(self):
        self.frameNum = 0

    def grabImage(self):
        if os.name != 'nt':
            return

        fileName = 'img-' + '{fr:04d}'.format(fr=self.frameNum) + '.png'
        im = ImageGrab.grab((0, 0, 400, 400))
        im.save(fileName)

        self.frameNum += 1

GRAVITY = 9.81  # m/s^2


def texture():
    checkerboard = ( (color.blue, color.red),
                     (color.red, color.blue) )
    # tex = texture(data=checkerboard,
    #     mapping="rectangular",
    #     interpolate=False)

    tex = {'file':textures.rug }

    return tex


def main():
    setupDisplay()
    screen = PrintScreen()

    tiltAngle = 15 * (math.pi / 180)
    posInit = vector(0, -0.5, 0)
    slipper = box(pos=posInit, length=5)
    # Musi być minus, bo funkcja obraca odwrotnie do ruchu wskazówek zegara
    slipper.rotate(angle=-tiltAngle, axis=vector(0, 0, 1))

    tex = texture()
    vInit = vector(0, 0, 0)
    roller = cylinder(pos=vector(-1 * math.cos(tiltAngle) * 0.5 * slipper.length,
                                 math.sin(tiltAngle) * 0.5 * slipper.length + 0.5,
                                 0),
                      axis=vector(0, 0, 1), radius=0.5, texture=tex)
    roller.vel = vInit
    roller.velAng = 0
    roller.mass = 1.0
    roller.coeffOfFriction = 0.15  # Współczynnik tarcia

    freq = 100
    dt = 1.0 / freq
    t = 0

    slipDir = norm(rotate(vector(1, 0, 0), angle=-tiltAngle))
    arrow(pos=vector(0, 2, 1), axis=slipDir)

    totalAngle = 0
    while t < 3000:
        rate(freq)

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
        screen.grabImage()


def setupDisplay():
    canvas(x=0, y=0, width=400, height=400,
        userzoom=False, userspin=True, autoscale=False,
        center=vector(0, 0, -9), foreground=color.white, background=color.black)


if __name__ == '__main__':
    main()
