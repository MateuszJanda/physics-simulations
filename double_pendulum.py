#!/usr/bin/env python3

"""
Author: Mateusz Janda <mateusz janda at gmail com>
Site: github.com/MateuszJanda
Ad maiorem Dei gloriam
"""


import vpython as vp
import math
# import povexport


GRAVITY_ACC = 9.81  # [m/s^2]
DAMPING = 12


def main():
    scene = setup_display()
    rod1, rod2 = create_bodies()

    freq = 30
    dt = 1/freq
    t = 0

    frame = 0
    while True:
        vp.rate(100)

        step_simulation_sympy(dt, rod1, rod2)

        # povexport.export(scene, filename='img-%04d.pov' % frame,
        #   include_list=['colors.inc', 'stones.inc', 'woods.inc', 'metals.inc'])
        frame += 1
        t += dt


def setup_display():
    scene = vp.canvas(x=0, y=0, width=400, height=400,
        userzoom=False, userspin=True, autoscale=False,
        center=vp.vector(0, 0, 0), foreground=vp.color.white,
        background=vp.color.black)

    return scene


def create_bodies(shift=vp.vector(0, 0, 0)):
    length1 = 9
    angle1 = math.radians(60)
    rod1 = vp.cylinder(pos=vp.vector(0, 6, -10) + shift,
        length=length1,
        radius=0.3,
        mass=4,
        axis=length1 * vp.vector(math.sin(angle1), -math.cos(angle1), 0),
        ang=angle1,
        ang_vel=0)

    length2 = 6
    angle2 = math.radians(90)
    rod2 = vp.cylinder(pos=rod1.pos + rod1.axis,
        length=length2,
        radius=0.3,
        mass=4,
        axis=length2 * vp.vector(math.sin(angle2), -math.cos(angle2), 0),
        ang=angle2,
        ang_vel=0)

    return rod1, rod2


def step_simulation(dt, rod1, rod2):
    """
    Credits:
    https://www.myphysicslab.com/pendulum/double-pendulum-en.html
    """
    from math import sin, cos

    G = GRAVITY_ACC
    m1, m2 = rod1.mass, rod2.mass
    L1, L2 = rod1.length, rod2.length
    t1, t2 = rod1.ang, rod2.ang
    t1_d1, t2_d1 = rod1.ang_vel, rod2.ang_vel

    t1_d2 = (-G*(2*m1 + m2)*sin(t1) - m2*G*sin(t1 - 2*t2) - 2*sin(t1 - t2)*m2*(t2_d1**2*L2 + t1_d1**2*L1*cos(t1 - t2))) / \
        (L1*(2*m1 + m2 - m2*cos(2*t1 - 2*t2)))
    t1_d1 += t1_d2 * dt
    t1 += t1_d1 * dt

    t2_d2 = (2*sin(t1 - t2)*(t1_d1**2*L1*(m1 + m2) + G*(m1 + m2)*cos(t1) + t2_d1**2*L2*m2*cos(t1 - t2))) / \
        (L2*(2*m1 + m2 - m2*cos(2*t1 - 2*t2)))
    t2_d1 += t2_d2 * dt
    t2 += t2_d1 * dt

    rod1.ang_acc = t1_d2
    rod1.ang_vel = t1_d1
    rod1.ang = t1

    rod2.ang_acc = t2_d2
    rod2.ang_vel = t2_d1
    rod2.ang = t2

    rod1.axis = rod1.length * vp.vector(math.sin(rod1.ang), -math.cos(rod1.ang), 0)
    rod2.pos = rod1.pos + rod1.axis
    rod2.axis = rod2.length * vp.vector(math.sin(rod2.ang), -math.cos(rod2.ang), 0)


def step_simulation_sympy(dt, rod1, rod2):
    """
    Equations calculated by SymPy. Simulation match with original.
    Credits:
    https://www.myphysicslab.com/pendulum/double-pendulum-en.html
    """
    from math import sin, cos

    G = GRAVITY_ACC
    m1, m2 = rod1.mass, rod2.mass
    L1, L2 = rod1.length, rod2.length
    t1, t2 = rod1.ang, rod2.ang
    t1_d1, t2_d1 = rod1.ang_vel, rod2.ang_vel

    t1_d2 = -(L1*m2*t1_d1**2*sin(2*t1 - 2*t2)/2 + L2*m2*t2_d1**2*sin(t1 - t2) + G*m1*sin(t1) + G*m2*sin(t1)/2 + G*m2*sin(t1 - 2*t2)/2)/(L1*(m1 - m2*cos(t1 - t2)**2 + m2))
    t1_d1 += t1_d2 * dt
    t1 += t1_d1 * dt

    t2_d2 = (-(m1 + m2)*(-L1*t1_d1**2*sin(t1 - t2) + G*sin(t2)) + (L2*m2*t2_d1**2*sin(t1 - t2) + G*m1*sin(t1) + G*m2*sin(t1))*cos(t1 - t2))/(L2*(m1 - m2*cos(t1 - t2)**2 + m2))
    t2_d1 += t2_d2 * dt
    t2 += t2_d1 * dt

    rod1.ang_acc = t1_d2
    rod1.ang_vel = t1_d1
    rod1.ang = t1

    rod2.ang_acc = t2_d2
    rod2.ang_vel = t2_d1
    rod2.ang = t2

    rod1.axis = rod1.length * vp.vector(math.sin(rod1.ang), -math.cos(rod1.ang), 0)
    rod2.pos = rod1.pos + rod1.axis
    rod2.axis = rod2.length * vp.vector(math.sin(rod2.ang), -math.cos(rod2.ang), 0)


def step_simulation_notes(dt, rod1, rod2):
    """
    Hand notes.
    Credits:
    https://www.myphysicslab.com/pendulum/double-pendulum-en.html
    """
    from math import sin, cos

    G = GRAVITY_ACC
    m1, m2 = rod1.mass, rod2.mass
    L1, L2 = rod1.length, rod2.length
    t1, t2 = rod1.ang, rod2.ang
    t1_d1, t2_d1 = rod1.ang_vel, rod2.ang_vel

    t1_d2 = -(L1*m2*t1_d1**2*sin(2*t1 - 2*t2)/2 + L2*m2*t2_d1**2*sin(t1 - t2) + \
        G*m1*sin(t1) + G*m2*sin(t1)/2 + G*m2*sin(t1 - 2*t2)/2)/ \
        (L1*(m1 - m2*cos(t1 - t2)**2 + m2))
    t1_d1 += t1_d2 * dt
    t1 += t1_d1 * dt

    t2_d2 = ((-sin(t2)*t1_d1**2*L1*sin(t1) - sin(t2)*t2_d1**2*L2*cos(t2) - sin(t2)*G + \
        cos(t2)*t1_d1**2*L1*sin(t1) + cos(t2)*t2_d1**2*L2*sin(t2))*(m1 + m2) + \
        G*(m2*sin(t1) + m1*sin(t1))*(cos(t2)*cos(t1) + sin(t2)*sin(t1))) / \
        ((m1 + m2) - m2*(cos(t2)*cos(t1) + sin(t2)*sin(t1))**2)
    t2_d1 += t2_d2 * dt
    t2 += t2_d1 * dt

    rod1.ang_acc = t1_d2
    rod1.ang_vel = t1_d1
    rod1.ang = t1

    rod2.ang_acc = t2_d2
    rod2.ang_vel = t2_d1
    rod2.ang = t2

    rod1.axis = rod1.length * vp.vector(math.sin(rod1.ang), -math.cos(rod1.ang), 0)
    rod2.pos = rod1.pos + rod1.axis
    rod2.axis = rod2.length * vp.vector(math.sin(rod2.ang), -math.cos(rod2.ang), 0)


def resolve_formula():
    """
    Resolve formula using SymPy. Unfortunately verification doesn't work
    right now.
    """
    from sympy import symbols, sin, cos, solve, simplify, Eq

    m1, m2 = symbols('m1, m2')
    G = symbols('G')
    t1, t2, t1_d1, t1_d2, t2_d1, t2_d2 = symbols('t1, t2, t1_d1, t1_d2, t2_d1, t2_d2')
    L1, L2 = symbols('L1, L2')

    x1_d2 = -t1_d1**2*L1*sin(t1) + t1_d2*L1*cos(t1)
    y1_d2 =  t1_d1**2*L1*cos(t1) + t1_d2*L1*sin(t1)
    x2_d2 =  x1_d2 - t2_d1**2*L2*sin(t2) + t2_d2*L2*cos(t2)
    y2_d2 =  y1_d2 + t2_d1**2*L2*cos(t2) + t2_d2*L2*sin(t2)
    eq13 = Eq(sin(t1)*(m1*y1_d2 + m2*y2_d2 + m2*G + m1*G) + cos(t1)*(m1*x1_d2 + m2*x2_d2), 0)
    eq16 = Eq(sin(t2)*(m2*y2_d2 + m2*G) + cos(t2)*(m2*x2_d2), 0)

    result = solve([eq13, eq16], [t1_d2, t2_d2], dict=True)
    print(result)

    # [{t1_d2: -(L1*m2*t1_d1**2*sin(2*t1 - 2*t2)/2 + L2*m2*t2_d1**2*sin(t1 - t2) + G*m1*sin(t1) + G*m2*sin(t1)/2 + G*m2*sin(t1 - 2*t2)/2)/(L1*(m1 - m2*cos(t1 - t2)**2 + m2)),
    #   t2_d2: (-(m1 + m2)*(-L1*t1_d1**2*sin(t1 - t2) + G*sin(t2)) + (L2*m2*t2_d1**2*sin(t1 - t2) + G*m1*sin(t1) + G*m2*sin(t1))*cos(t1 - t2))/(L2*(m1 - m2*cos(t1 - t2)**2 + m2))}]

    # Verify
    res_t1_d2 = (-G*(2*m1 + m2)*sin(t1) - m2*G*sin(t1 - 2*t2) - 2*sin(t1 - t2)*m2*(t2_d1**2*L2 + t1_d1**2*L1*cos(t1 - t2))) / \
        (L1*(2*m1 + m2 - m2*cos(2*t1 - 2*t2)))

    res_t2_d2 = (2*sin(t1 - t2)*(t1_d1**2*L1*(m1 + m2) + G*(m1 + m2)*cos(t1) + t2_d1**2*L2*m2*cos(t1 - t2))) / \
        (L2*(2*m1 + m2 - m2*cos(2*t1 - 2*t2)))

    print('t1_d2 match:', simplify(result[0][t1_d2] - res_t1_d2) == 0)
    print('t2_d2 match:', simplify(result[0][t2_d2] - res_t2_d2) == 0)


if __name__ == '__main__':
    main()
    # resolve_formula()
