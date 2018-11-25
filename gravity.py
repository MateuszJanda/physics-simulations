#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
http://stackoverflow.com/questions/18620546/python-gravity-simulator-behaving-strangely
http://spiff.rit.edu/richmond/nbody/OrbitRungeKutta4.pdf
http://showmedo.com/videotutorials/video?name=pythonThompsonVPython8&fromSeriesID=30
"""

import vpython as vp


GRAVITY_ACC = 1.0


def main():
    pltX1, pltX2, pltY1, pltY2 = setupDisplay()

    sInit = vp.vector(10, -5, 0)
    vInit = vp.vector(0, 6, 0)

    star = vp.sphere(pos=vp.vector(0, 0, 0), radius=1)
    star.mass = 1000.0

    trail1 = vp.curve()
    satellite1 = vp.sphere(pos=sInit, radius=0.5, color=vp.color.green)
    satellite1.visible = False
    satellite1.vel = vInit
    satellite1.mass = 1.0

    trail2 = vp.curve()
    satellite2 = vp.sphere(pos=sInit, radius=0.5, color=vp.color.red)
    satellite2.vel = vInit
    satellite2.mass = 1.0

    freq = 100
    dt = 1/freq
    t = 0

    while t < 3000:
        vp.rate(freq)

        d1 = distance(star, satellite1)
        d2 = distance(star, satellite2)
        if d1 <= star.radius or d2 <= star.radius:
            break

        # wartość wektora
        FgravMag = (GRAVITY_ACC * star.mass * satellite2.mass) / (d2**2)
        # wektor skierowany w stronę gwiazdy
        Fgrav = vp.norm(star.pos - satellite2.pos) * FgravMag
        satellite2.acc = Fgrav / satellite2.mass
        satellite2.vel += satellite2.acc * dt
        satellite2.pos += satellite2.vel * dt
        # Ciekawy efekt spadania
        # satellite2.pos += satellite2.vel * dt + 0.5 * satellite2.acc * dt**2

        trail2.append(satellite2.pos)
        pltX2.plot(pos=(t, satellite2.pos.x))
        pltY2.plot(pos=(t, satellite2.pos.y))

        # Warunek końca do zapętlenie gif-a
        if t > 1 and vp.mag(satellite2.pos - sInit) < 0.1:
            break

        t += dt


def distance(star, satellite):
    return vp.mag(star.pos - satellite.pos)


def setupDisplay():
    vp.canvas(x=0, y=0, width=400, height=400,
        userzoom=False, userspin=True, autoscale=False,
        center=vp.vector(0, 0, 3), foreground=vp.color.white, background=vp.color.black)

    vp.arrow(pos=vp.vector(0, 0, 0), axis=vp.vector(10, 0, 0), shaftwidth=0.3)
    vp.arrow(pos=vp.vector(0, 0, 0), axis=vp.vector(0, 10, 0), shaftwidth=0.3)

    pltX1 = vp.gcurve(color=vp.color.green, size=2)
    pltX2 = vp.gcurve(color=vp.color.red, size=2)
    pltY1 = vp.gcurve(color=vp.color.green, size=2)
    pltY2 = vp.gcurve(color=vp.color.red, size=2)

    return pltX1, pltX2, pltY1, pltY2


if __name__ == '__main__':
    main()
