import math

def line_args(pt1, pt2):
    num = pt2[1] - pt1[1]
    den = pt2[0] - pt1[0]
    if den == 0:
        den = 1e-9
    m = num / den
    c = pt2[1] - m * pt2[0]
    return m, c

def intercept(m1, c1, m2, c2):
    m = m2 - m1
    c = c1 - c2
    if m == 0:
        m = 1e-9
    x = c / m
    y = m1 * x + c1
    return x, y


def angle(pt1, pt2):
    a = -math.atan2((pt2[1] - pt1[1]), (pt2[0] - pt1[0]))
    if a < 0:
        a += 2.0 * math.pi
    return a

def diff_angles(a1, a2):
    return math.pi - abs(abs(a1 - a2) - math.pi)

def avr_angles(a1, a2):
    if a2 > a1:
        a2 -= 2 * math.pi
    return (a1 + a2 + math.pi / 2) / 2.0; 