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
    r1, r2 = create_bodies(vp.vector(3, 0, 0))

    freq = 30
    dt = 1/freq
    t = 0

    frame = 0
    while True:
        vp.rate(100)

        step_simulation(dt, rod1, rod2)
        step_simulation2(dt, r1, r2)

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
    theta1_angle = math.radians(60)
    rod1 = vp.cylinder(pos=vp.vector(0, 6, -10) + shift,
        length=length1,
        radius=0.3,
        mass=4,
        axis=length1 * vp.vector(math.sin(theta1_angle), -math.cos(theta1_angle), 0),
        theta=theta1_angle,
        d1_theta=0)

    length2 = 6
    theta2_angle = math.radians(90)
    rod2 = vp.cylinder(pos=rod1.pos + rod1.axis,
        length=length2,
        radius=0.3,
        mass=4,
        axis=length2 * vp.vector(math.sin(theta2_angle), -math.cos(theta2_angle), 0),
        theta=theta2_angle,
        d1_theta=0)

    return rod1, rod2


def step_simulation(dt, rod1, rod2):
    """
    Credits:
    https://www.myphysicslab.com/pendulum/double-pendulum-en.html
    """
    rod1.d2_theta = (-GRAVITY_ACC * (2 * rod1.mass + rod2.mass) * math.sin(rod1.theta) \
                     -GRAVITY_ACC * rod2.mass * math.sin(rod1.theta - 2 * rod2.theta) \
                     -2 * math.sin(rod1.theta - rod2.theta) * rod2.mass * \
                     (rod2.d1_theta**2 * rod2.length + rod1.d1_theta**2 * rod1.length * math.cos(rod1.theta - rod2.theta)) \
                     ) / \
                     (rod1.length * (2 * rod1.mass + rod2.mass - rod2.mass * math.cos(2 * rod1.theta - 2 * rod2.theta)))
    rod1.d1_theta += rod1.d2_theta * dt
    rod1.theta += rod1.d1_theta * dt
    rod1.axis = rod1.length * vp.vector(math.sin(rod1.theta), -math.cos(rod1.theta), 0)

    rod2.d2_theta = (2 * math.sin(rod1.theta - rod2.theta) * \
                     (
                        rod1.d1_theta**2 * rod1.length * (rod1.mass + rod2.mass) + \
                        GRAVITY_ACC*(rod1.mass + rod2.mass)*math.cos(rod1.theta) + \
                        rod2.d1_theta**2 * rod2.length * rod2.mass * math.cos(rod1.theta - rod2.theta) \
                     ) \
                    ) / \
                    (rod2.length * (2 * rod1.mass + rod2.mass - rod2.mass * math.cos(2 * rod1.theta - 2 * rod2.theta)))
    rod2.d1_theta += rod2.d2_theta * dt
    rod2.theta += rod2.d1_theta * dt

    rod2.pos = rod1.pos + rod1.axis
    rod2.axis = rod2.length * vp.vector(math.sin(rod2.theta), -math.cos(rod2.theta), 0)



def step_simulation2(dt, rod1, rod2):
    """
    Credits:
    https://www.myphysicslab.com/pendulum/double-pendulum-en.html
    """
    from math import sin, cos

    g = GRAVITY_ACC
    m1 = rod1.mass
    m2 = rod2.mass
    L1 = rod1.length
    L2 = rod2.length
    t1 = rod1.theta
    t2 = rod2.theta

    t1_d1 = rod1.d1_theta
    t2_d1 = rod2.d1_theta


    t1_d2 = (-g*(2*m1 + m2)*sin(t1) - m2*g*sin(t1 - 2*t2) - 2*sin(t1 - t2)*m2*(t2_d1**2*L2 + t1_d1**2*L1*cos(t1 - t2))) / \
        (L1*(2*m1 + m2 - m2*cos(2*t1 - 2*t2)))
    t1_d1 += t1_d2 * dt
    t1 += t1_d1 * dt

    t2_d2 = (2*sin(t1 - t2)*(t1_d1**2*L1*(m1 + m2) + g*(m1 + m2)*cos(t1) + t2_d1**2*L2*m2*cos(t1 - t2))) / \
        (L2*(2*m1 + m2 - m2*cos(2*t1 - 2*t2)))
    t2_d1 += t2_d2 * dt
    t2 += t2_d1 * dt

    rod1.d2_theta = t1_d2
    rod1.d1_theta = t1_d1
    rod1.theta = t1

    rod2.d2_theta = t2_d2
    rod2.d1_theta = t2_d1
    rod2.theta = t2

    rod1.axis = rod1.length * vp.vector(math.sin(rod1.theta), -math.cos(rod1.theta), 0)
    rod2.pos = rod1.pos + rod1.axis
    rod2.axis = rod2.length * vp.vector(math.sin(rod2.theta), -math.cos(rod2.theta), 0)


if __name__ == '__main__':
    main()
