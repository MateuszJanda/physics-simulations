#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
https://www.youtube.com/watch?v=HGcZrHnqjeg
"""

import vpython as vp
import math


GRAVITY = 9.81  # m/s^2


def main():
    setup_display()
    roller, ramp, arrow = create_bodies()

    freq = 100
    dt = 1/freq
    t = 0

    totalAngle = 0
    while t < 3000:
        vp.rate(freq)
        step_simulation(dt, roller, ramp, arrow)

        # Sprawdzenie, czy obrót wokół własnej osi jest równy przebytej drodze (ruch bez poślizgu)
        # totalAngle += dAngle
        # if totalAngle >= 2 * math.pi:
        #     circuit = 2 * math.pi * roller.radius
        #     pathLen = ((-roller.pos.y - roller.radius) / math.sin(ramp.alpha)) % circuit
        #     print('obwod: %f, droga: %f' % (circuit, pathLen))
        #     totalAngle -= (2 * math.pi)

        t += dt


def setup_display():
    vp.canvas(x=0, y=0, width=400, height=400,
        userzoom=False, userspin=True, autoscale=False,
        center=vp.vector(0, 0, -9), foreground=vp.color.white, background=vp.color.black)


def create_bodies():
    ramp = vp.box(pos=vp.vector(0, -0.5, 0), length=5,
        alpha=15 * (math.pi / 180))
    # Musi być minus, bo funkcja obraca odwrotnie do ruchu wskazówek zegara
    ramp.rotate(angle=-ramp.alpha, axis=vp.vector(0, 0, 1))

    roller = vp.cylinder(pos=vp.vector(-1 * math.cos(ramp.alpha) * 0.5 * ramp.length,
                                       math.sin(ramp.alpha) * 0.5 * ramp.length + 0.5,
                                       0),
        axis=vp.vector(0, 0, 1), radius=0.5, texture=texture(),
        vel=vp.vector(0, 0, 0),
        ang_vel=0,
        mass=1,
        coeffOfFriction=0.15)  # Współczynnik tarcia

    slope_dir = vp.norm(vp.rotate(vp.vector(1, 0, 0), angle=-ramp.alpha))
    arrow = vp.arrow(pos=vp.vector(0, 2, 1), axis=slope_dir)

    return roller, ramp, arrow


def texture():
    tex = {'file':vp.textures.rug }
    return tex


def step_simulation(dt, roller, ramp, arrow):
    calc_forces(roller, ramp.alpha)
    integrate(dt, roller, ramp.alpha)

    # Jeżeli dotrze do krawędzi pochylni to koniec symulacji
    if -roller.pos.y + roller.radius >= math.sin(ramp.alpha) * ramp.length/2:
        exit()


def calc_forces(roller, alpha):
    # Ruch obrotowy
    # Moment bezwładności dla walca ze wzoru
    roller.force = roller.coeffOfFriction * roller.mass * GRAVITY * math.cos(alpha)
    roller.moment = 0.5 * roller.mass * roller.radius**2


def integrate(dt, roller, alpha):
    slope_dir = vp.norm(vp.rotate(vp.vector(1, 0, 0), angle=-alpha))

    # Ruch środka masy
    # a = g(sin(a) - u*cos(a)) - str. 104
    # również na filmiku - "toczenie z poślizgiem" - 13:21:00
    roller.acc2 = GRAVITY * (math.sin(alpha) - roller.coeffOfFriction * math.cos(alpha)) * slope_dir
    # z filmiku "toczenie bez poślizgu" - 10:24:00
    roller.acc = (roller.mass * GRAVITY * math.sin(alpha)) / (roller.mass + roller.moment/roller.radius**2) * slope_dir
    roller.vel += roller.acc * dt
    roller.pos += roller.vel * dt + 0.5 * roller.acc * dt**2

    roller.ang_vel += ((roller.force * roller.radius) / roller.moment) * dt
    # Musi być minus, bo funkcja obraca odwrotnie do ruchu wskazówek zegara
    dAngle = roller.ang_vel * dt
    roller.rotate(angle=-dAngle)


if __name__ == '__main__':
    main()
