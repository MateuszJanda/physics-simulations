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
    pltX1, pltX2, pltY1, pltY2 = setup_display()
    star, satellite1, satellite2 = create_bodies()

    freq = 100
    dt = 1/freq
    t = 0

    while True:
        vp.rate(freq)

        d1 = distance(star, satellite1)
        d2 = distance(star, satellite2)
        if d1 <= star.radius or d2 <= star.radius:
            break

        FgravMag = (GRAVITY_ACC * star.mass * satellite2.mass) / (d2**2)
        Fgrav = vp.norm(star.pos - satellite2.pos) * FgravMag
        satellite2.acc = Fgrav / satellite2.mass
        satellite2.vel += satellite2.acc * dt
        satellite2.pos += satellite2.vel * dt

        satellite2.trail.append(satellite2.pos)
        pltX2.plot(pos=(t, satellite2.pos.x))
        pltY2.plot(pos=(t, satellite2.pos.y))

        t += dt


def distance(star, satellite):
    return vp.mag(star.pos - satellite.pos)


def setup_display():
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


def create_bodies():
    star = vp.sphere(pos=vp.vector(0, 0, 0), radius=1,
        mass = 1000)

    satellite1 = vp.sphere(pos=vp.vector(10, -5, 0), radius=0.5, color=vp.color.green,
        visible = True,
        vel = vp.vector(0, 6, 0),
        mass = 1.0,
        trail = vp.curve())

    satellite2 = vp.sphere(pos=vp.vector(10, -5, 0), radius=0.5, color=vp.color.red,
        vel = vp.vector(0, 6, 0),
        mass = 1.0,
        trail = vp.curve())

    return star, satellite1, satellite2


if __name__ == '__main__':
    main()
