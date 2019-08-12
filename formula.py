import sympy as s


# def main():

m1, m2 = s.symbols('m1, m2')
g = s.symbols('g')
x1_d2, y1_d2, x2_d2, y2_d2 = s.symbols('x1_d2, y1_d2, x2_d2, y2_d2')
t1, t2, t1_d1, t1_d2, t2_d1, t2_d2 = s.symbols('t1, t2, t1_d1, t1_d2, t2_d1, t2_d2')
L1, L2 = s.symbols('L1, L2')

eq1 = x1_d2 + t1_d1**2 * L1 * s.sin(t1) - t1_d2 * L1 * s.cos(t1)
eq2 = y1_d2 - t1_d1**2 * L1 * s.cos(t1) - t1_d2 * L1 * s.sin(t1)
eq3 = x2_d2 - x1_d2 + t2_d1**2 * L2 * s.sin(t2) - t2_d2 * L2 * s.cos(t2)
eq4 = y2_d2 - y1_d2 - t2_d1**2 * L2 * s.cos(t2) - t2_d2 * L2 * s.sin(t2)

eq16 = s.sin(t2) * (m2 * y2_d2 + m2 * g) + s.cos(t2) * (m2 * x2_d2)
result1 = s.solve([eq1, eq2, eq3, eq4, eq16], \
    [m1, m2, g, x1_d2, y1_d2, x2_d2, y2_d2, t1, t2, t1_d1, t1_d2, t2_d1, t2_d2, L1, L2], \
    dict=True)

for r in result1:
    print(r.keys())

eq13 = s.sin(t1) * (m1 * y1_d2 + m2 * y2_d2 + m2 * g + m1 * g) + s.cos(t1) * (m1 * x1_d2 + m2 * x2_d2)
result2 = s.solve([eq1, eq2, eq3, eq4, eq13], \
    [m1, m2, g, x1_d2, y1_d2, x2_d2, y2_d2, t1, t2, t1_d1, t1_d2, t2_d1, t2_d2, L1, L2], \
    dict=True)

for r in result2:
    print(r.keys())

# result2 = s.solve([eq1, eq2, eq3, eq4, eq16], (m1, m2, g, x1_d2, y1_d2, x2_d2, y2_d2, \
#     t1, t2, t1_d1, t1_d2, t2_d1, t2_d2, L1, L2), dict=True)

# for r in result:
#     print(r.keys())

    # print(r[0].keys())
    # print(r[0][t2])

    # r1 = (2 * s.sin(t1 - t2) * (t1_d1**2 * L1 * (m1 + m2) + g * (m1 + m2) * s.cos(t1) + t2_d1**2 * L2 * m2 * s.cos(t1 - t2))) / \
    #     (L2 * (2 * m1 + m2 - m2 * s.cos(2 * t1 - 2 * t2))) - t2_d2

    # print(r['t2_d1'] == r1)


# if __name__ == '__main__':
#     main()