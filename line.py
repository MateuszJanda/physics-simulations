#!/usr/bin/env python3

"""
Author: Mateusz Janda <mateusz janda at gmail com>
Site: github.com/MateuszJanda
Ad maiorem Dei gloriam
"""

import math
import vpython as vp
import random
import numpy as np
# import povexport


NUM_CHAIN_LINKS = 10

GRAVITY_ACC = -9.81             # [m/s^2]

LINK_MASS = 2                   # [kg]

AIR_DENSITY = 1.225             # [kg/m^3]

SPRING_DAMPING_CONSTANT = 50    # [kg/s]
SPRING_TENSION_CONSTANT = 900   # [kg/s^2]
SPRING_SHEAR_CONSTANT = 900     # [kg/s^2]

DRAG_COEFFICIENT = 0.01
E_RESTITUTION = 0.25
COLLISION_TOLERANCE = 0.05      # [m]
VELOCITY_TOLERANCE = 0.0001     # [m/s^2]



class Link:
    def __init__(self, mass, pos, locked):
        self.locked = locked
        self.mass = mass  # [kg]

        # Set initial position of this link/particle
        self.pos = pos

        # Set initial velocity, acceleration and force to zero
        self.vel = vp.vector(0, 0, 0)  # [m/s]
        self.acc = vp.vector(0, 0, 0)  # [m/s^2]
        self.force = vp.vector(0, 0, 0)  # [N]


class Spring():
    """
    k - spring constant [kg/s^2]
    d - damping coefficient [kg/s]
    length - (normal) length of unstretched spring [m]
    """
    def __init__(self, particle1, particle2, k):
        self.particle1 = particle1
        self.particle2 = particle2
        self.length = (particle1.pos - particle2.pos).mag  # [m]
        self.k = k  # [kg/s^2]
        self.d = SPRING_DAMPING_CONSTANT  # [kg/s]


class Collision():
    def __init__(self, particle, normal):
        self.particle = particle
        self.normal = normal


def main():
    scene = setup_display()

    links = create_chain_links(anchor1=-8, anchor2=8)
    create_chain(links)
    springs = create_springs(links)

    t = 0
    freq = 100
    dt = 1/freq

    frame = 0
    while True:
        vp.rate(freq)

        step_simulation(dt, links, springs)

        # povexport.export(scene, filename='img-%04d.pov' % frame,
        #     include_list=['colors.inc', 'stones.inc', 'woods.inc', 'metals.inc'])
        frame += 1
        t += dt


def setup_display():
    scene = vp.canvas(x=0, y=0, width=400, height=400,
                userzoom=False, userspin=True, autoscale=False,
                center=vp.vector(1, 8, 0), foreground=vp.color.white, background=vp.color.black)

    return scene


def create_chain_links(anchor1, anchor2, height=10):
    links = []
    for x in np.arange(anchor1, anchor2, (anchor2 - anchor1) / NUM_CHAIN_LINKS):
        links.append(Link(
            mass=LINK_MASS * random.uniform(0.5, 1),
            pos=vp.vector(x, height, 0),
            locked=(x == anchor1)))

    # Add last link
    links.append(Link(
        mass=LINK_MASS,
        pos=vp.vector(anchor2, height, 0),
        locked=True))

    return links


def create_springs(links):
    # Setup the structural springs
    # Connect springs between each adjacent vertex
    springs = []

    for l1, l2 in zip(links[:-1], links[1:]):
        springs.append(Spring(
            particle1=l1,
            particle2=l2,
            k=SPRING_TENSION_CONSTANT))

    return springs


def create_chain(links):
    # Create VPython cylinders for each chain link
    for l1, l2 in zip(links[:-1], links[1:]):
        l1.cylinder = vp.cylinder(pos=l1.pos, axis=l2.pos - l1.pos, radius=0.2)


def step_simulation(dt, links, springs):
    # Calculate all of the forces
    calc_forces(links, springs)

    # Integrate
    for particle in links:
        particle.acc = particle.force / particle.mass
        particle.vel += particle.acc * dt
        particle.pos += particle.vel * dt

    # Update line geometry
    update_chain(links)


def calc_forces(links, springs):
    # Process gravity and drag forces
    for particle in links:
        if particle.locked:
            continue

        # Gravity
        particle.force = vp.vector(0, GRAVITY_ACC * particle.mass, 0)

    # Process spring forces - page 82
    for spring in springs:
        l = spring.particle1.pos - spring.particle2.pos
        relative_vel = spring.particle1.vel - spring.particle2.vel

        f1 = -(spring.k * (l.mag - spring.length) + spring.d * (vp.dot(relative_vel, l)/l.mag)) * l.norm()
        f2 = -f1

        if not spring.particle1.locked:
            spring.particle1.force += f1

        if not spring.particle2.locked:
            spring.particle2.force += f2


def update_chain(links):
    # Update chain elements
    for l1, l2 in zip(links[:-1], links[1:]):
        if not l1.locked:
            l1.cylinder.pos = l1.pos
        l1.cylinder.axis = l2.pos - l1.pos


if __name__ == '__main__':
    main()
