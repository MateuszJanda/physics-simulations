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

SPRING_DAMPING_CONSTANT = 2     # [kg/s]
SPRING_TENSION_CONSTANT = 500   # [kg/s^2]
SPRING_SHEAR_CONSTANT = 600     # [kg/s^2]

DRAG_COEFFICIENT = 0.01
E_RESTITUTION = 0.25
COLLISION_TOLERANCE = 0.05      # [m]
VELOCITY_TOLERANCE = 0.0001     # [m/s^2]



class Particle:
    def __init__(self, mass, pos, locked):
        self.locked = locked
        self.mass = mass  # [kg]

        # Set initial position of this particle
        self.pos = pos

        # Set initial velocity, acceleration and force to zero
        self.vel = vp.vector(0, 0, 0)  # [m/s]
        self.acc = vp.vector(0, 0, 0)  # [m/s^2]
        self.force = vp.vector(0, 0, 0)  # [N]


class StructuralSpring():
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

    particles = create_particles(anchor1=-5, anchor2=5)
    create_line(particles)
    struct_springs = create_structural_springs(particles)

    t = 0
    freq = 100
    dt = 1/freq

    frame = 0
    while True:
        vp.rate(freq)

        step_simulation(dt, particles, struct_springs)

        # povexport.export(scene, filename='img-%04d.pov' % frame,
        # include_list=['colors.inc', 'stones.inc', 'woods.inc', 'metals.inc'])
        frame += 1
        t += dt


def setup_display():
    scene = vp.canvas(x=0, y=0, width=400, height=400,
                userzoom=False, userspin=True, autoscale=False,
                center=vp.vector(1, 8, 0), foreground=vp.color.white, background=vp.color.black)

    return scene


def create_particles(anchor1, anchor2, height=10):
    particles = []
    for x in np.arange(anchor1, anchor2, (anchor2 - anchor1) / NUM_CHAIN_LINKS):
        particles.append(Particle(
            mass=LINK_MASS,
            pos=vp.vector(x, height, 0),
            locked=(x == anchor1)))

    particles.append(Particle(
        mass=LINK_MASS,
        pos=vp.vector(anchor2, height, 0),
        locked=True))

    return particles


def create_structural_springs(particles):
    # Setup the structural springs
    # Connect springs between each adjacent vertex
    struct_springs = []

    for p1, p2 in zip(particles[:-1], particles[1:]):
        struct_springs.append(StructuralSpring(
            particle1=p1,
            particle2=p2,
            k=SPRING_TENSION_CONSTANT))

    return struct_springs


def create_line(particles):
    for p1, p2 in zip(particles[:-1], particles[1:]):
        # p1.spring = vp.helix(pos=p1.pos, axis=p2.pos - p1.pos, radius=0.3)
        p1.spring = vp.cylinder(pos=p1.pos, axis=p2.pos - p1.pos, radius=0.1)


def step_simulation(dt, particles, struct_springs):
    # Calculate all of the forces
    calc_forces(particles, struct_springs)

    # Integrate
    for particle in particles:
        particle.acc = particle.force / particle.mass
        particle.vel += particle.acc * dt
        particle.pos += particle.vel * dt

    # Update line geometry
    update_line_geometry(particles)


def calc_forces(particles, struct_springs):
    # Process gravity and drag forces
    for particle in particles:
        if particle.locked:
            continue

        # Gravity
        particle.force = vp.vector(0, GRAVITY_ACC * particle.mass, 0)

        # Viscous drag - page 17. Surface without rotation calculation.
        # https://pl.wikipedia.org/wiki/Op%C3%B3r_aero(hydro)dynamiczny#Formu%C5%82y_na_wielko%C5%9B%C4%87_oporu_aero(hydro)dynamicznego
        drag_vector = -vp.norm(particle.vel)
        # particle.force += drag_vector * DRAG_COEFFICIENT * AIR_DENSITY * 0.5 \
        #         * particle.vel.mag2 * particle.surface

        # Wind force
        # particle.force += wind_force()

    # Process spring forces - page 82
    for spring in struct_springs:
        l = spring.particle1.pos - spring.particle2.pos
        relative_vel = spring.particle1.vel - spring.particle2.vel

        f1 = -(spring.k * (l.mag - spring.length) + spring.d * (vp.dot(relative_vel, l)/l.mag)) * l.norm()
        f2 = -f1

        if not spring.particle1.locked:
            spring.particle1.force += f1

        if not spring.particle2.locked:
            spring.particle2.force += f2


def update_line_geometry(particles):
    # Update line elements

    for p1, p2 in zip(particles[:-1], particles[1:]):
        if not p1.locked:
            p1.spring.pos = p1.pos
        p1.spring.axis = p2.pos - p1.pos

if __name__ == '__main__':
    main()
