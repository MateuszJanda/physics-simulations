#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
http://showmedo.com/videotutorials/video?name=pythonThompsonVPython9&fromSeriesID=30
http://fizyka.org/?teoria,25,2
http://sciaga.onet.pl/12581,60,162,104,1,22603,1,sciaga.html
http://efizyka.net.pl/sila-w-ruchu-harmonicznym_7981
http://www.iwiedza.net/wiedza/114.html
https://github.com/gcschmit/vpython-physics/blob/master/energy%20of%20mass%20on%20spring/massOnSpring.py
"""

from vpython import *
import math

import os
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


def main():
    plt1, plt2 = setupDisplay()
    screen = PrintScreen()

    anchorY = 4
    # Sufit, do którego podwieszamy sprężynę
    box(pos=vector(0, anchorY + 0.5, 0), length=6, height=1, width=6)

    springLengthInitial = 3
    # Na początku spręzyna nie jest ani naciągnięta ani ściśnięta
    spring = helix(pos=vector(0, anchorY, 0), axis=vector(0, -springLengthInitial + 1, 0),
                   thickness=1/10, radius=1)
    # Jedna wartość dla stałej sprężystości i tłumienia
    spring.constant = 5  # N/m

    h = 2
    sInit = vector(0, anchorY - spring.length - h/2, 0)
    mbox = box(pos=sInit, length=2, height=h, width=2)
    mbox.mass = 2  # kg
    mbox.vel = vector(0, 0, 0)

    freq = 100
    dt = 1.0 / freq
    t = 0

    while t < 3000:
        rate(freq)

        Fg = GRAVITY * mbox.mass * vector(0, -1, 0)

        springDisplacement = spring.length - springLengthInitial
        Fk = -spring.constant * springDisplacement * vector(0, -1, 0)
        F = Fg + Fk
        mbox.acc = F / mbox.mass
        mbox.vel += mbox.acc * dt
        # mbox.pos += mbox.vel * dt + 0.5 * mbox.acc * dt**2
        mbox.pos += mbox.vel * dt # TODO: który wzór ma mieć tutaj zastosowanie

        plt1.plot(pos=(t, Fk.y))
        plt2.plot(pos=(t, F.y))

        # Nowa długość sprężyny
        # spring.y == anchorY
        spring.length = spring.pos.y - mbox.pos.y - mbox.height/2

        # Warunek końca do zapętlenie gif-a
        if t > 1 and mag(mbox.pos - sInit) < 0.1 and mbox.vel.y < 0:
            pass

        t += dt
        screen.grabImage()

    exit()


def setupDisplay():
    canvas(x=0, y=0, width=400, height=400,
        userzoom=False, userspin=True, autoscale=False,
        center=vector(0, 0, 0), foreground=color.white, background=color.black)

    plt1 = gcurve(color=color.green, size=2)
    plt2 = gcurve(color=color.red, size=2)

    return plt1, plt2


if __name__ == '__main__':
    main()
