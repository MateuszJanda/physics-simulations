#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import vpython as vp
from itertools import chain
import random


NUM_COLUMNS = 10
NUM_ROWS = 7

GRAVITY_ACC = -9.81             # [m/s^2]

CLOTH_MASS = 15                 # [kg]
CLOTH_WIDTH = 10                # [m]
CLOTH_HEIGHT = 7                # [m]

FLAG_POLE_HEIGHT = 15           # [m]
FLAG_POLE_RADIUS = 0.1          # [m]
SEAM_RADIUS = 0.05              # [m]

AIR_DENSITY = 1.225             # [kg/m^3]

SPRING_DAMPING_CONSTANT = 2     # [kg/s]
SPRING_TENSION_CONSTANT = 500   # [kg/s^2]
SPRING_SHEAR_CONSTANT = 600     # [kg/s^2]

DRAG_COEFFICIENT = 0.01
E_RESTITUTION = 0.25
COLLISION_TOLERANCE = 0.05      # [m]
VELOCITY_TOLERANCE = 0.0001     # [m/s^2]

WIND_FACTOR = 30


class Particle:
    def __init__(self, mass, pos, surface, locked):
        self.locked = locked
        self.mass = mass  # [kg]
        self.surface = surface  # [m^2]

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
        self.d = SPRING_DAMPING_CONSTANT # [kg/s]


class Collision():
    def __init__(self, particle, normal):
        self.particle = particle
        self.normal = normal


class Seam():
    def __init__(self):
        self.horiz = None
        self.vertic = None


def main():
    scene = setup_display()

    create_pole()
    particles = create_particles()
    flag = create_flag(particles)
    struct_springs = create_structural_springs(particles)

    t = 0
    freq = 100
    dt = 1/freq

    while True:
        vp.rate(freq)

        step_simulation(dt, particles, struct_springs, flag)

        t += dt


def setup_display():
    return vp.canvas(x=0, y=0, width=640, height=360,
                userzoom=False, userspin=True, autoscale=False,
                center=vp.vector(1, 8, 0), foreground=vp.color.white, background=vp.color.black)


def create_particles():
    particles = [[0 for y in range(NUM_COLUMNS)] for x in range(NUM_ROWS)]

    column_step = CLOTH_WIDTH/NUM_COLUMNS
    row_step = CLOTH_HEIGHT/NUM_ROWS
    y_offset = FLAG_POLE_HEIGHT - CLOTH_HEIGHT

    all_faces = 4*1 + 2*(NUM_ROWS-2)*2 + 2*(NUM_COLUMNS-2)*2 + (NUM_ROWS-2)*(NUM_COLUMNS-2)*4
    mass_per_face = CLOTH_MASS/all_faces  # [kg]
    surface_per_face = (CLOTH_WIDTH/(NUM_ROWS*4)) * (CLOTH_HEIGHT/(NUM_COLUMNS*4))  # [m^2]

    for r in range(NUM_ROWS):
        for c in range(NUM_COLUMNS):
            if r == 0 and (c == 0 or c == NUM_COLUMNS-1):
                faces = 1
            elif r == NUM_ROWS-1 and (c == 0 or c == NUM_COLUMNS-1):
                faces = 1
            elif r == 0 or r == NUM_ROWS-1 or c == 0 or c == NUM_COLUMNS-1:
                faces = 2
            else:
                faces = 4

            particles[r][c] = Particle(
                mass=faces*mass_per_face,
                pos=vp.vector(c*column_step, (CLOTH_HEIGHT - (r*row_step)) + y_offset, 0),
                surface=faces*surface_per_face,
                locked=((c == 0) and (r == 0 or r == NUM_ROWS-1)))
            particles[r][c].faces = faces

    print('Units:')
    for rp in particles:
        print(' '.join([str(p.faces) for p in rp]))

    units = sum([p.faces for p in chain.from_iterable(particles)])
    print('Units sum: ', units)
    print('Mass per unit: ', CLOTH_MASS/units)
    print('Mass sum: ', sum([p.mass for p in chain.from_iterable(particles)]))

    return particles


def create_structural_springs(particles):
    # Setup the structural springs
    # Connect springs between each adjacent vertex
    struct_springs = []

    for r in range(NUM_ROWS):
        for c in range(NUM_COLUMNS):
            if c < NUM_COLUMNS-1:
                struct_springs.append(StructuralSpring(
                    particle1=particles[r][c],
                    particle2=particles[r][c+1],
                    k=SPRING_TENSION_CONSTANT))
            if r < NUM_ROWS-1:
                struct_springs.append(StructuralSpring(
                    particle1=particles[r][c],
                    particle2=particles[r+1][c],
                    k=SPRING_TENSION_CONSTANT))
            if c < NUM_COLUMNS-1 and r < NUM_ROWS-1:
                struct_springs.append(StructuralSpring(
                    particle1=particles[r][c],
                    particle2=particles[r+1][c+1],
                    k=SPRING_SHEAR_CONSTANT))
            if c > 0 and r < NUM_ROWS-1:
                struct_springs.append(StructuralSpring(
                    particle1=particles[r][c],
                    particle2=particles[r+1][c-1],
                    k=SPRING_SHEAR_CONSTANT))

    return struct_springs


def create_pole():
    pole = vp.cylinder(pos=vp.vector(0, 0, 0), axis=vp.vector(0, FLAG_POLE_HEIGHT, 0), radius=FLAG_POLE_RADIUS)
    return pole


def create_flag(particles):
    flag = [[Seam() for y in range(NUM_COLUMNS)] for x in range(NUM_ROWS)]

    for r in range(NUM_ROWS):
        for c in range(NUM_COLUMNS):
            seam = flag[r][c]
            if r+1 < NUM_ROWS and c+1 < NUM_COLUMNS:
                seam.horiz = vp.cylinder(pos=particles[r][c].pos,
                        axis=particles[r+1][c].pos - particles[r][c].pos, radius=SEAM_RADIUS)
                seam.vertic = vp.cylinder(pos=particles[r][c].pos,
                        axis=particles[r][c+1].pos - particles[r][c].pos, radius=SEAM_RADIUS)
            elif r+1 < NUM_ROWS:
                seam.horiz = vp.cylinder(pos=particles[r][c].pos,
                        axis=particles[r+1][c].pos - particles[r][c].pos, radius=SEAM_RADIUS)
            elif c+1 < NUM_COLUMNS:
                seam.vertic = vp.cylinder(pos=particles[r][c].pos,
                        axis=particles[r][c+1].pos - particles[r][c].pos, radius=SEAM_RADIUS)

    return flag


def step_simulation(dt, particles, struct_springs, flag):
    # Calculate all of the forces
    calc_forces(particles, struct_springs)

    # Integrate
    for particle in chain.from_iterable(particles):
        particle.acc = particle.force / particle.mass
        particle.vel += particle.acc * dt
        particle.pos += particle.vel * dt

    # Check for collisions
    collisions = check_for_collisions(particles)
    resolve_collisions(collisions)

    # Update cloth object's geometry
    update_cloth_geometry(particles, flag)


def calc_forces(particles, struct_springs):
    # Process gravity and drag forces
    for particle in chain.from_iterable(particles):
        if particle.locked:
            continue

        # Gravity
        particle.force = vp.vector(0, GRAVITY_ACC * particle.mass, 0)

        # Viscous drag - page 17. Surface without rotation calculation.
        # https://pl.wikipedia.org/wiki/Op%C3%B3r_aero(hydro)dynamiczny#Formu%C5%82y_na_wielko%C5%9B%C4%87_oporu_aero(hydro)dynamicznego
        drag_vector = -vp.norm(particle.vel)
        particle.force += drag_vector * DRAG_COEFFICIENT * AIR_DENSITY * 0.5 \
                * particle.vel.mag2 * particle.surface

        # Wind force
        particle.force += wind_force()

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


def wind_force():
    f = vp.norm(vp.vector(random.randrange(10), 0, random.randrange(-10, 10))) * random.randrange(WIND_FACTOR)
    return f


def check_for_collisions(particles):
    collisions = []

    for particle in chain.from_iterable(particles):
        if particle.locked:
            continue

        # Check for collisions with ground
        if (particle.pos.y <= COLLISION_TOLERANCE) and (particle.vel.y < VELOCITY_TOLERANCE):
            collisions.append(Collision(particle=particle, normal=vp.norm(vp.vector(0, 1, 0))))

        # Check for collisions with flag pole
        # Center of mass of a pole is at the same height as particle. So distance (vector) between two
        # mass centers will be also vector perpendicular to the edge of collision - page 248
        dist = particle.pos - vp.vector(0, particle.pos.y, 0)
        n = dist.norm()
        relative_vel_n = vp.dot(particle.vel, n)

        if dist.mag <= (FLAG_POLE_RADIUS + COLLISION_TOLERANCE) and \
           (0 < particle.pos.y < FLAG_POLE_HEIGHT) and (relative_vel_n < VELOCITY_TOLERANCE):
            collisions.append(Collision(particle=particle, normal=n))

    return collisions


def resolve_collisions(collisions):
    for c in collisions:
        # Impulse - page 115
        # https://en.wikipedia.org/wiki/Collision_response#Impulse-based_reaction_model
        vel_relative = c.particle.vel
        impluse = (-(E_RESTITUTION+1) * vp.dot(vel_relative, c.normal)) / (1/c.particle.mass)
        c.particle.vel += (impluse*c.normal)/c.particle.mass


def update_cloth_geometry(particles, flag):
    # Update flag elements
    for r in range(NUM_ROWS):
        for c in range(NUM_COLUMNS):
            seam = flag[r][c]
            if r+1 < NUM_ROWS and c+1 < NUM_COLUMNS:
                seam.horiz.pos = particles[r][c].pos
                seam.horiz.axis = particles[r+1][c].pos - particles[r][c].pos

                seam.vertic.pos = particles[r][c].pos
                seam.vertic.axis = particles[r][c+1].pos - particles[r][c].pos
            elif r+1 < NUM_ROWS:
                seam.horiz.pos = particles[r][c].pos
                seam.horiz.axis = particles[r+1][c].pos - particles[r][c].pos
            elif c+1 < NUM_COLUMNS:
                seam.vertic.pos = particles[r][c].pos
                seam.vertic.axis = particles[r][c+1].pos - particles[r][c].pos


if __name__ == '__main__':
    main()
