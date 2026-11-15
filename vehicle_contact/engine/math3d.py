"""Minimal 3D vector / quaternion / rotation-matrix helpers.

Everything uses plain tuples of floats so the engine has no third-party
dependency and remains deterministic across Python versions.
"""

from math import sqrt, sin, cos, atan2, asin, pi

EPS = 1e-12


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def length(a):
    return sqrt(dot(a, a))


def norm(a):
    n = length(a)
    if n < EPS:
        return (0.0, 0.0, 0.0)
    return (a[0] / n, a[1] / n, a[2] / n)


def lerp(a, b, t):
    return (a[0] + (b[0] - a[0]) * t,
            a[1] + (b[1] - a[1]) * t,
            a[2] + (b[2] - a[2]) * t)


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


# ---------------------------------------------------------------- quaternion
# Quaternion stored as (w, x, y, z), body->world rotation.

def q_identity():
    return (1.0, 0.0, 0.0, 0.0)


def q_mul(p, q):
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return (pw * qw - px * qx - py * qy - pz * qz,
            pw * qx + px * qw + py * qz - pz * qy,
            pw * qy - px * qz + py * qw + pz * qx,
            pw * qz + px * qy - py * qx + pz * qw)


def q_from_axis_angle(axis, angle):
    axis = norm(axis)
    h = angle * 0.5
    s = sin(h)
    return (cos(h), axis[0] * s, axis[1] * s, axis[2] * s)


def q_normalize(q):
    n = sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2)
    if n < EPS:
        return q_identity()
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def q_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def q_rotate(q, v):
    """Rotate vector v by quaternion q (body -> world)."""
    _, x, y, z = q
    u = (x, y, z)
    t = scale(cross(u, v), 2.0)
    return add(add(v, scale(t, q[0])), cross(u, t))


def q_integrate(q, omega_world, dt):
    """Advance orientation given world-space angular velocity."""
    spin = q_from_axis_angle(omega_world, length(omega_world) * dt)
    if length(omega_world) < EPS:
        return q
    return q_normalize(q_mul(spin, q))


def q_to_euler(q):
    """Return (roll, pitch, yaw) in radians, ZYX convention."""
    w, x, y, z = q
    roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = 2 * (w * y - z * x)
    pitch = asin(clamp(sinp, -1.0, 1.0))
    yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


# --------------------------------------------------------------------- matrix
def rot_matrix(q):
    """3x3 body->world rotation matrix as nested tuples."""
    w, x, y, z = q
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))


def m_vec(m, v):
    return (m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2])


def m_transpose(m):
    return tuple(tuple(m[c][r] for c in range(3)) for r in range(3))


def world_to_body(q, v_world):
    return m_vec(m_transpose(rot_matrix(q)), v_world)


def body_to_world(q, v_body):
    return m_vec(rot_matrix(q), v_body)


def deg(a):
    return a * 180.0 / pi


def rad(a):
    return a * pi / 180.0
