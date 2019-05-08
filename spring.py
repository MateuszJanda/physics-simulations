#!/usr/bin/env python3

"""
Author: Mateusz Janda <mateusz janda at gmail com>
Site: github.com/MateuszJanda
Ad maiorem Dei gloriam
"""

"""
References:
http://showmedo.com/videotutorials/video?name=pythonThompsonVPython9&fromSeriesID=30
http://fizyka.org/?teoria,25,2
http://efizyka.net.pl/sila-w-ruchu-harmonicznym_7981
http://www.iwiedza.net/wiedza/114.html
https://github.com/gcschmit/vpython-physics/blob/master/energy%20of%20mass%20on%20spring/massOnSpring.py
"""

import vpython as vp
import math
# import povexport

GRAVITY_ACC = 9.81  # [m/s^2]


def main():
    lock = True
    scene, plt1, plt2 = setup_display()
    spring, load = create_bodies()

    freq = 100
    dt = 1/freq
    t = 0

    frame = 0
    while True:
        vp.rate(freq)

        step_simulation(dt, spring, load)
        plot_data(t, load, plt1, plt2)

        # camera {
        #     right <-image_width/image_height, 0, 0>
        #     location <0.000000, 0.000000, 220.000000>
        #     up <0.000000, 1.000000, 0.000000>
        #     look_at <0.000000, 0.000000, 0.000000>
        #     angle 0.000000
        # }
        # povexport.export(scene, filename='img-%04d.pov' % frame, include_list=['colors.inc', 'stones.inc', 'woods.inc', 'metals.inc'])
        frame += 1

        t += dt


def setup_display():
    scene = vp.canvas(x=0, y=0, width=400, height=400,
        userzoom=False, userspin=True, autoscale=False,
        center=vp.vector(0, 0, 4), foreground=vp.color.white, background=vp.color.black)

    plt1 = vp.gcurve(color=vp.color.green, size=2)
    plt2 = vp.gcurve(color=vp.color.red, size=2)

    return scene, plt1, plt2


def create_bodies():
    # Celling
    celling_y = 4
    vp.box(pos=vp.vector(0, celling_y + 0.5, 0), length=6, height=1, width=6)

    SPRING_INIT_LENGTH = 3
    spring = vp.helix(pos=vp.vector(0, celling_y, 0), axis=vp.vector(0, -SPRING_INIT_LENGTH + 1, 0),
        thickness=1/10,
        radius=1,
        init_length=SPRING_INIT_LENGTH,  # [m]
        k_const=5)  # spring constant [kg/s^2]

    height = 2
    LOAD_INIT_POS = vp.vector(0, celling_y - spring.length - height/2, 0)
    load = vp.box(pos=LOAD_INIT_POS,
        init_pos=LOAD_INIT_POS,
        height=height,  # [m]
        width=2,  # [m]
        length=2,  # [m]
        mass=2,  # [kg]
        vel=vp.vector(0, 0, 0))  # [m/s^2]

    return spring, load


def step_simulation(dt, spring, load):
    calc_forces(dt, spring, load)
    integrate(dt, load)
    # New spring length
    spring.length = spring.pos.y - load.pos.y - load.height/2


def calc_forces(dt, spring, load):
    force_g = GRAVITY_ACC * load.mass * vp.vector(0, -1, 0)

    displacement = spring.length - spring.init_length
    force_k = -spring.k_const * displacement * vp.vector(0, -1, 0)

    load.force_k = force_k
    load.force = force_g + force_k


def integrate(dt, load):
    load.acc = load.force / load.mass
    load.vel += load.acc * dt
    load.pos += load.vel * dt


def plot_data(t, load, plt1, plt2):
    plt1.plot(pos=(t, load.force_k.y))
    plt2.plot(pos=(t, load.force.y))


if __name__ == '__main__':
    main()
