from sympy import *


def seek(eqs, do, sol=[], strict=True):
    from sympy.solvers.solvers import _invert as f
    from sympy.core.compatibility import ordered
    while do and eqs:
        for x in do:
            for e in eqs:
                i, d = f(e.lhs - e.rhs, x)
                if d != x:
                    continue
                break
            else:
                if strict:
                    assert None  # no eq could be solved for x
                continue
            sol.append((d, i))
            eqs.remove(e)
            break
        do.remove(x)
        if not strict:
            do.extend(i.free_symbols)
            do = list(ordered(do))
        for _ in range(len(eqs)):
            eqs[_] = eqs[_].xreplace({x: i})
    return sol

def focus(eqs, *syms, **kwargs):
    """Given Equality instances in ``eqs``, solve for symbols in
    ``syms`` and resolve as many of the free symbols in the solutions
    as possible. When ``evaluate=True`` a dictionary with keys being
    ``syms`` is returned, otherwise a list of identified symbols
    leading to the desired symbols is given.

    Examples
    ========
    >>> focus((Eq(a, b), Eq(b + 2, c)), a)
    {a: c - 2}
    >>> focus((Eq(a, b), Eq(b + 2, c)), a, evaluate=False)
    [(b, c - 2), (a, b)]
    """
    from sympy.solvers.solvers import _invert as f
    from sympy.core.compatibility import ordered
    evaluate = kwargs.get('evaluate', True)
    sol = []
    free = Tuple(*eqs).free_symbols
    do = set(syms) & free
    if not do:
        return sol
    eqs = list(eqs)
    seek(eqs, do, sol)
    assert not do
    for x, i in sol:
        do |= i.free_symbols
    do = list(ordered(do))  # make it canonical
    seek(eqs, do, sol, strict=False)
    if evaluate:
        while len(sol) > len(syms):
            x, s = sol.pop()
            for i in range(len(sol)):
                sol[i] = (sol[i][0], sol[i][1].xreplace({x: s}))
        for i in reversed(range(1, len(syms))):
            x, s = sol[i]
            for j in range(i):
                y, t = sol[j]
                sol[j] = y, f(y - t.xreplace({x: s}), y)[0]
    if evaluate:
        sol = dict(sol)
    else:
        sol = list(reversed(sol))
    return sol


def main():
    m1, m2 = symbols('m1, m2')
    g = symbols('g')
    x1_d2, y1_d2, x2_d2, y2_d2 = symbols('x1_d2, y1_d2, x2_d2, y2_d2')
    t1, t2, t1_d1, t1_d2, t2_d1, t2_d2 = symbols('t1, t2, t1_d1, t1_d2, t2_d1, t2_d2')
    L1, L2 = symbols('L1, L2')

    eq1 = x1_d2 + t1_d1**2 * L1 * sin(t1) - t1_d2 * L1 * cos(t1)
    eq2 = y1_d2 - t1_d1**2 * L1 * cos(t1) - t1_d2 * L1 * sin(t1)
    eq3 = x2_d2 - x1_d2 + t2_d1**2 * L2 * sin(t2) - t2_d2 * L2 * cos(t2)
    eq4 = y2_d2 - y1_d2 - t2_d1**2 * L2 * cos(t2) - t2_d2 * L2 * sin(t2)

    eq13 = sin(t1) * (m1 * y1_d2 + m2 * y2_d2 + m2 * g + m1 * g) + cos(t1) * (m1 * x1_d2 + m2 * x2_d2)
    eq16 = sin(t2) * (m2 * y2_d2 + m2 * g) + cos(t2) * (m2 * x2_d2)

    # F = (y1_d2, y2_d2)
    F = (t1_d2, t2_d2)

    f = focus([Eq(i, 0) for i in [eq1, eq2, eq3, eq4, eq13, eq16]], *F)
    print([f[i].has(*F) for i in F])
    print(count_ops(f))

    r, e = cse([Eq(*i) for i in f.items()])

    print(r)
    print(e)


    res13 = Eq(t1_d2, (-g*(2*m1 + m2)*sin(t1) - m2*g*sin(t1 - 2*t2) - 2*sin(t1 - t2)*m2*(t2_d1**2*L2 + t1_d1**2*L1*cos(t1 - t2))) / \
        (L1*(2*m1 + m2 - m2*cos(2*t1 * 2*t2))))

    res16 = Eq(t2_d2, (2 * sin(t1 - t2) * (t1_d1**2 * L1 * (m1 + m2) + g * (m1 + m2) * cos(t1) + t2_d1**2 * L2 * m2 * cos(t1 - t2))) / \
        (L2 * (2 * m1 + m2 - m2 * cos(2 * t1 - 2 * t2))))

    print(e[1] == res13)
    print(e[0] == res16)

if __name__ == '__main__':
    main()