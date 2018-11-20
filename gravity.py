#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
http://stackoverflow.com/questions/18620546/python-gravity-simulator-behaving-strangely
http://spiff.rit.edu/richmond/nbody/OrbitRungeKutta4.pdf
http://showmedo.com/videotutorials/video?name=pythonThompsonVPython8&fromSeriesID=30
"""

from vpython import *

import os
if os.name == 'nt':
    import ImageGrab


def main():
    pltX1, pltX2, pltY1, pltY2 = setupDisplay()
    screen = PrintScreen()

    sInit = vector(10, -5, 0)
    vInit = vector(0, 6, 0)

    star = sphere(pos=vector(0, 0, 0), radius=1)
    star.mass = 1000.0

    trail1 = curve()
    satellite1 = sphere(pos=sInit, radius=0.5, color=color.green)
    satellite1.visible = False
    satellite1.vel = vInit
    satellite1.mass = 1.0

    trail2 = curve()
    satellite2 = sphere(pos=sInit, radius=0.5, color=color.red)
    satellite2.vel = vInit
    satellite2.mass = 1.0

    G = 1.0
    t = 0

    freq = 100
    dt = 1.0 / freq

    while t < 3000:
        rate(freq)

        d1 = distance(star, satellite1)
        d2 = distance(star, satellite2)
        if d1 <= star.radius or d2 <= star.radius:
            break

        """
        Pierwsza metoda - nie działa - na wyliczenie położenia w momenecit czasu t, trzeba obliczyć
        całkę oznaczoną od t0 do t
        """
        # wersorE = norm(star.pos - satellite1.pos)
        # satellite1.pos = sInit + vInit * t + 0.5 * ((G * star.mass) / d1**2) * wersorE * t**2

        # trail1.append(satellite1.pos)
        # pltX1.plot(pos=(t, satellite1.x))
        # pltY1.plot(pos=(t, satellite1.y))

        """ Druga metoda - metoda Eulera """

        # wartość wektora
        FgravMag = (G * star.mass * satellite2.mass) / (d2**2)
        # wektor skierowany w stronę gwiazdy
        Fgrav = norm(star.pos - satellite2.pos) * FgravMag
        satellite2.acc = Fgrav / satellite2.mass
        satellite2.vel += satellite2.acc * dt
        satellite2.pos += satellite2.vel * dt
        # Ciekawy efekt spadania
        # satellite2.pos += satellite2.vel * dt + 0.5 * satellite2.acc * dt**2

        trail2.append(satellite2.pos)
        pltX2.plot(pos=(t, satellite2.pos.x))
        pltY2.plot(pos=(t, satellite2.pos.y))

        # Warunek końca do zapętlenie gif-a
        if t > 1 and mag(satellite2.pos - sInit) < 0.1:
            break

        t += dt
        screen.grabImage()

    exit()


def distance(star, satellite):
    # return mag(star.pos - satellite.pos)
    # d = math.sqrt((star.x - satellite.x)**2 +
    #               (star.y - satellite.y)**2 +
    #               (star.z - satellite.z)**2)
    # return d
    return mag(star.pos - satellite.pos)


def setupDisplay():
    canvas(x=0, y=0, width=400, height=400,
        userzoom=False, userspin=True, autoscale=False,
        center=vector(0, 0, 0), foreground=color.white, background=color.black)

    arrow(pos=vector(0, 0, 0), axis=vector(10, 0, 0), shaftwidth=0.3)
    arrow(pos=vector(0, 0, 0), axis=vector(0, 10, 0), shaftwidth=0.3)

    pltX1 = gcurve(color=color.green, size=2)
    pltX2 = gcurve(color=color.red, size=2)
    pltY1 = gcurve(color=color.green, size=2)
    pltY2 = gcurve(color=color.red, size=2)

    return pltX1, pltX2, pltY1, pltY2


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


if __name__ == '__main__':
    main()
